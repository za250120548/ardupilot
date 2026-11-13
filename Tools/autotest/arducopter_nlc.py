#!/usr/bin/env python3
'''
Safety validation of a custom attitude/rate controller (AC_CustomControl)
in ArduCopter SITL.

Every CustomControllerNLC_* test flies the same failsafe scenario twice with
identical checks: once with the main ArduCopter rate controller (P+PID) and
once with the custom controller active (CC_TYPE, RC6 high). A test passes when
both behave the same: failsafe triggered, expected mode, no loss of control,
landed and disarmed.

Run from the ArduPilot root directory:
    Tools/autotest/autotest.py --no-clean --speedup=10 test.CopterNLC
    Tools/autotest/autotest.py test.CopterNLC.CustomControllerNLC_Battery

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from arducopter import AutoTestCopter
from vehicle_test_suite import NotAchievedException


class AutoTestCopterNLC(AutoTestCopter):
    '''Copter tests for the custom controller. Inherits every helper and
    every existing test of AutoTestCopter, so existing tests (for example
    BatteryFailsafeTwoStage) can be reused unchanged as scenarios.'''

    # allowed lean beyond ATC_ANGLE_MAX before the test fails (deg)
    CC_TILT_MARGIN_DEG = 5

    def cc_setup(self, cc_type, active):
        '''select the custom controller backend and switch it on or off with RC6'''
        self.set_parameters({
            'CC_TYPE': cc_type,
            'CC_AXIS_MASK': 7,
            'RC6_OPTION': 109,
        })
        self.set_rc(6, 1000)
        self.reboot_sitl()      # CC_TYPE needs a reboot
        if self.get_parameter('CC_TYPE') != cc_type:
            raise NotAchievedException("CC_TYPE is not %u after reboot" % cc_type)
        if active:
            self.context_collect('STATUSTEXT')
            self.set_rc(6, 2000)
            self.wait_statustext("Custom controller is ON", check_context=True)

    def cc_flight_monitor(self, results, tilt_limit_deg):
        '''message hook: fails the test if the vehicle leans more than
        tilt_limit_deg while armed and records the data used for the comparison report'''
        def hook(mav, m):
            m_type = m.get_type()
            if m_type == 'ATTITUDE':
                # Check if motors are ARMED to compare the tilt with the limit, otherwise don't compare
                if results['armed']:
                    tilt = math.degrees(max(abs(m.roll), abs(m.pitch)))
                    results['max_tilt_deg'] = max(results['max_tilt_deg'], tilt)
                    if tilt > tilt_limit_deg:
                        raise NotAchievedException(
                            "lean %.1f deg exceeds the %.1f deg limit" % (tilt, tilt_limit_deg))
            elif m_type == 'STATUSTEXT':
                results['texts'].append((self.get_sim_time_cached(), m.text))
            elif m_type == 'HEARTBEAT' and m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if results['armed'] and not armed:
                    results['disarm_t'] = self.get_sim_time_cached()
                results['armed'] = armed
        return hook

    def cc_ab_test(self, scenario, cc_type=3, cc_label="STA", cc_stays_on=True, expect_flight=True):
        '''run scenario() with the main controller and with the custom
        controller, apply the common checks to both and report the comparison.
        scenario() must take off, trigger the failsafe and end landed and disarmed
        (use expect_flight=False for scenarios that never arm, e.g. pre-arm checks).
        cc_stays_on: fail if the custom controller is switched off during the scenario.'''
        report = []
        # Execute the scenario twice, once with the main controller and once with the custom controller
        for label, active in (("P+PID", False), (cc_label, True)):
            self.start_subtest("%s with %s" % (scenario.__name__, label))
            self.context_push()
            self.cc_setup(cc_type, active)
            results = {'max_tilt_deg': 0.0, 'texts': [], 'armed': False, 'disarm_t': None}
            # if during flight the ATTITUTDE control exceeds the ATC_ANGLE_MAX + CC_TILT_MARGIN_DEG, the test fails
            tilt_limit = self.get_parameter('ATC_ANGLE_MAX') + self.CC_TILT_MARGIN_DEG
            self.install_message_hook_context(self.cc_flight_monitor(results, tilt_limit))
            t_start = self.get_sim_time()
            # executes the test scenario passed
            scenario()
            results['duration_s'] = self.get_sim_time() - t_start
            self.context_pop()      # removes the monitor hook
            self.set_rc(6, 1000)    # after the hook, so this OFF is not counted as a failure

            # common checks, register what the hook recorded in the dictionary for the comparison report
            texts = [text for _, text in results['texts']]
            crash = [text for text in texts if 'Crash' in text]
            if crash:
                raise NotAchievedException("%s: crash detected: %s" % (label, crash))
            if active and cc_stays_on and "Custom controller is OFF" in texts:
                raise NotAchievedException("%s: custom controller switched off during the failsafe" % label)
            if expect_flight and results['disarm_t'] is None:
                raise NotAchievedException("%s: vehicle did not disarm" % label)
            if texts is None or not texts:
                raise NotAchievedException("%s: no STATUSTEXT messages recorded" % label)
            report.append((label, results))

        self.reboot_sitl()      # leave CC_TYPE=0 active for the following tests
        self.progress("Custom controller A/B report for %s:" % scenario.__name__)
        for label, r in report:
            disarm = "%7.1f s" % r['disarm_t'] if r['disarm_t'] is not None else "      -"
            self.progress("  %-6s max lean %5.1f deg, scenario %6.1f s, disarmed at %s (sim time)" % (
                label, r['max_tilt_deg'], r['duration_s'], disarm))
        return report

    def cc_prearm_scenario(self):
        '''Pre-arm checks throttle with P+PID and with the NLC (STA) custom controller'''
        # The arm check confirm that UAV is healthy if failsafe is triggered.
        self.wait_ready_to_arm()
        self.change_mode("STABILIZE")

        # cause 1: Set failure RC3 high to trigger the prearm failsafe
        self.set_rc(3, 1800)
        self.assert_arm_failure("Throttle too high")
        # remove failure cause 1: Set RC3 low to clear the prearm failsafe
        self.set_rc(3, 1000)  # set RC3 low to clear the prearm failsafe

        # cause 2: forbidden mode
        self.change_mode('RTL')
        self.assert_arm_failure("RTL mode not armable")
        # remove failure cause 2: Set mode to STABILIZE to clear the prearm failsafe
        self.change_mode("STABILIZE")

        # Cause 3: low battery
        self.set_parameter('BATT_ARM_VOLT', 13.0)
        self.assert_arm_failure("below minimum arming voltage")
        # Remove failure cause 3: Disable  BATT_ARM_VOLT check
        self.set_parameter('BATT_ARM_VOLT', 0)

        # Cause 4: motors miss configuration
        # Read the current values of MOT_PWM_MIN and MOT_PWM_MAX to restore them later
        pwm_min = self.get_parameter('MOT_PWM_MIN')
        pwm_max = self.get_parameter('MOT_PWM_MAX')
        self.set_parameters({'MOT_PWM_MIN': 100, 'MOT_PWM_MAX': 50})
        self.assert_arm_failure("Motors: Check MOT_PWM_MIN and MOT_PWM_MAX")
        # Remove failure cause 4: Set MOT_PWM_MIN and MOT_PWM_MAX to valid values to clear the prearm failsafe
        self.set_parameters({'MOT_PWM_MIN': pwm_min, 'MOT_PWM_MAX': pwm_max})

        # Positive test: Arm and disarm the vehicle to confirm that the prearm checks are cleared
        self.arm_vehicle()
        self.delay_sim_time(3, reason="armed on the ground")
        self.disarm_vehicle()


    def CustomControllerNLC_BatteryFailsafeBrakeLand(self):
        '''Battery failsafe brake/land action triggers BRAKE with P+PID and with the NLC (STA) custom controller'''
        # Trigger low battery condition with failsafe set to brake/land, Brake mode brings copter to an immediate stop
        report = self.cc_ab_test(self.BatteryFailsafeBrakeLand)

        # time from the low battery warning to disarm, for the thesis report
        for label, r in report:
            t_low = [t for t, text in r['texts'] if "Battery 1 is low" in text]
            if not t_low:
                raise NotAchievedException("%s: no low battery warning" % label)
            self.progress("  %-6s low battery -> disarmed in %.1f s" % (label, r['disarm_t'] - t_low[0]))

    def CustomControllerNLC_Battery(self):
        '''Two stage battery failsafe with P+PID and with the NLC (STA) custom controller'''
        # The existing BatteryFailsafeTwoStage test is used unchanged as the
        # scenario: low battery -> RTL, critical battery -> LAND, disarm.
        report = self.cc_ab_test(self.BatteryFailsafeTwoStage)

        # time from the critical battery warning to disarm, for the thesis report
        for label, r in report:
            t_crit = [t for t, text in r['texts'] if "Battery 1 is critical" in text]
            if not t_crit:
                raise NotAchievedException("%s: no critical battery warning" % label)
            self.progress("  %-6s critical battery -> disarmed in %.1f s" % (label, r['disarm_t'] - t_crit[0]))

    def CustomControllerNLC_VibrationFailsafe(self):
        '''Vibration failsafe with P+PID and with the NLC (STA) custom controller'''
        # The existing VibrationFailsafe test is used unchanged as the
        # scenario: high vibration -> LAND, disarm (forced).
        report = self.cc_ab_test(self.VibrationFailsafe)

        # time from the vibration warning to disarm, for the thesis report
        for label, r in report:
            t_vib = [t for t, text in r['texts'] if "Vibration compensation ON" in text]
            if not t_vib:
                raise NotAchievedException("%s: no vibration warning" % label)
            self.progress("  %-6s vibration warning -> disarmed in %.1f s" % (label, r['disarm_t'] - t_vib[0]))

    def CustomControllerNLC_Prearm(self):
        '''Pre-arm checks with P+PID and with the NLC (STA) custom controller'''
        report = self.cc_ab_test(self.cc_prearm_scenario, cc_stays_on=False, expect_flight=False)


    def tests(self):
        '''tests run by "test.CopterNLC"'''
        return [
            self.CustomControllerNLC_BatteryFailsafeBrakeLand,
            self.CustomControllerNLC_Battery,
            self.CustomControllerNLC_VibrationFailsafe,
            self.CustomControllerNLC_Prearm,
        ]
