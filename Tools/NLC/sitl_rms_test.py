#!/usr/bin/env python3
"""
SITL comparison: ArduCopter main attitude controller (P + PID) vs the
AC_CustomControl_NLC Super-Twisting (HOSM) controller.

The same deterministic stick profile is flown twice in ALT_HOLD:
  phase A: custom controller OFF (RC6 low)  -> main P+PID controller
  phase B: custom controller ON  (RC6 high) -> NLC / STA controller
and the RMS body-rate tracking error is computed from the DataFlash log.

Rate error metric (same definition for both phases):
  RATE.RDes - RATE.R, RATE.PDes - RATE.P, RATE.YDes - RATE.Y   [deg/s]
RATE.*Des is the rate reference produced by ArduCopter's attitude loop,
which runs in both phases, so both controllers are judged against the same
reference signal. When CC3_OPT bit 0 is set, the NLC phase additionally
reports its own sliding variable from the CNLC message.

Usage (from the ArduPilot root, after ./waf copter):
    python3 Tools/NLC/sitl_rms_test.py --speedup 5
    python3 Tools/NLC/sitl_rms_test.py --param CC3_K1_R=0.12 --param CC3_K2_R=0.4

Each run also writes separate PID/STA attitude and rate CSV files, suitable for
plotting measured and desired roll, pitch, and yaw over time.
"""

import argparse
import csv
import glob
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
os.environ.setdefault('MAVLINK20', '1')
try:
    # pymavlink installed with pip (normal ArduPilot / MAVProxy setup)
    from pymavlink import mavutil, DFReader  # noqa: E402
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
except (ImportError, AttributeError):
    # fallback: copy inside the ArduPilot tree (needs the generated dialect)
    sys.path.insert(0, os.path.join(ROOT, 'modules', 'mavlink'))
    for k in [k for k in sys.modules if k.startswith('pymavlink')]:
        del sys.modules[k]
    from pymavlink import mavutil, DFReader  # noqa: E402

HOME = '-35.363261,149.165230,584,353'

# deterministic stick profile: (seconds, roll_pwm, pitch_pwm, yaw_pwm)
PROFILE = [
    # time, roll, pitch, yaw
    (3.0, 1500, 1500, 1500),    # hover
    (1.0, 1700, 1500, 1500),    # roll right
    (1.0, 1300, 1500, 1500),    # roll left
    (2.0, 1500, 1500, 1500),    # settle
    (1.0, 1500, 1700, 1500),    # pitch forward
    (1.0, 1500, 1300, 1500),    # pitch backward
    (2.0, 1500, 1500, 1500),    # settle
    (1.0, 1500, 1500, 1700),    # yaw clockwise
    (1.0, 1500, 1500, 1300),    # yaw counter-clockwise
    (2.0, 1500, 1500, 1500),    # settle
    (4.0, 1650, 1650, 1500),    # combined roll right + pitch forward
    (4.0, 1350, 1350, 1500),    # combined roll left + pitch backward
    (3.0, 1500, 1500, 1500),     # settle
]


class SITL:
    def __init__(self, binary, workdir, speedup):
        self.binary = binary
        self.workdir = workdir
        self.speedup = speedup
        self.proc = None
        self.mav = None

    def start(self, wipe=False):
        cmd = [self.binary, '--model', 'quad', '--speedup', str(self.speedup),
               '--home', HOME, '-I0',
               '--defaults', os.path.join(ROOT, 'Tools/autotest/default_params/copter.parm')]
        if wipe:
            cmd.append('--wipe')
        self.log = open(os.path.join(self.workdir, 'sitl_stdout.txt'), 'a')
        self.proc = subprocess.Popen(cmd, cwd=self.workdir, stdout=self.log, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        self.mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, retries=60)
        self.mav.wait_heartbeat(timeout=60)
        self.last = {}
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def stop(self):
        if self.mav is not None:
            self.mav.close()
        if self.proc is not None:
            self.proc.terminate()
            self.proc.wait(timeout=10)
        self.proc = None
        self.mav = None

    # ---------------------------------------------------------------- helpers
    def set_param(self, name, value):
        for _ in range(5):
            self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component,
                                        name.encode(), float(value),
                                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            while m is not None and m.param_id != name:
                m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if m is not None and abs(m.param_value - float(value)) < 1e-4:
                return
        raise RuntimeError('could not set %s' % name)

    def rc(self, roll=1500, pitch=1500, thr=1500, yaw=1500, ch6=1000):
        chans = [roll, pitch, thr, yaw, 0, ch6, 0, 0]
        self.mav.mav.rc_channels_override_send(self.mav.target_system, self.mav.target_component, *chans)

    def pump(self, seconds, **rc):
        """keep RC override alive and drain messages for 'seconds' of wall time"""
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.rc(**rc)
            self.drain()
            time.sleep(0.02)

    def drain(self):
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                return
            self.last[m.get_type()] = m
            if m.get_type() == 'STATUSTEXT' and 'PreArm' not in m.text and 'Arm:' not in m.text:
                print('  SITL:', m.text)

    def mode(self, name):
        self.mav.set_mode(name)

    def boot_ms(self):
        self.drain()
        return self.last['SYSTEM_TIME'].time_boot_ms

    def alt(self):
        self.drain()
        m = self.last.get('GLOBAL_POSITION_INT')
        return m.relative_alt * 1e-3 if m is not None else 0.0

    def command(self, cmd, *p):
        p = list(p) + [0] * (7 - len(p))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, cmd, 0, *p)


def fly_profile(sitl, ch6, speedup):
    t0 = sitl.boot_ms()
    for dur, r, p, y in PROFILE:
        sitl.pump(dur / speedup, roll=r, pitch=p, yaw=y, ch6=ch6)
    t1 = sitl.boot_ms()
    return t0 * 1000, t1 * 1000


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float('nan')


def analyse(logfile, windows):
    """windows: dict name -> (t0_us, t1_us)"""
    res = {k: {'R': [], 'P': [], 'Y': [], 'aR': [], 'aP': [], 'sR': [], 'sP': [], 'sY': [],
               'oR': [], 'oP': [], 'oY': []} for k in windows}
    samples = {k: {'attitude': [], 'rate': []} for k in windows}
    log = DFReader.DFReader_binary(logfile, zero_time_base=False)
    while True:
        m = log.recv_match(type=['RATE', 'ATT', 'CNLC'])
        if m is None:
            break
        t = m.TimeUS
        for k, (a, b) in windows.items():
            if a <= t <= b:
                d = res[k]
                if m.get_type() == 'RATE':
                    d['R'].append(m.RDes - m.R)
                    d['P'].append(m.PDes - m.P)
                    d['Y'].append(m.YDes - m.Y)
                    d['oR'].append(m.ROut)
                    d['oP'].append(m.POut)
                    d['oY'].append(m.YOut)
                    samples[k]['rate'].append(
                        (t - a, m.R, m.P, m.Y, m.RDes, m.PDes, m.YDes))
                elif m.get_type() == 'ATT':
                    d['aR'].append(m.DesRoll - m.Roll)
                    d['aP'].append(m.DesPitch - m.Pitch)
                    samples[k]['attitude'].append(
                        (t - a, m.Roll, m.Pitch, m.Yaw, m.DesRoll, m.DesPitch, m.DesYaw))
                else:
                    d[['sR', 'sP', 'sY'][m.I]].append(math.degrees(m.S))
    out = {}
    for k, d in res.items():
        out[k] = {key: rms(v) for key, v in d.items() if not key.startswith('o')}
        # control-effort roughness: RMS of the sample-to-sample change of the
        # mixer input (RATE.ROut/POut/YOut = output of whichever controller is active)
        for ax in 'RPY':
            o = d['o' + ax]
            out[k]['dU' + ax] = rms([o[i+1] - o[i] for i in range(len(o) - 1)])
        out[k]['n'] = len(d['R'])
    return out, samples


def write_samples(samples, output_dir):
    """Write measured and desired RPY samples to one CSV per phase and message."""
    columns = {
        'attitude': ('time_s', 'roll_deg', 'pitch_deg', 'yaw_deg',
                     'des_roll_deg', 'des_pitch_deg', 'des_yaw_deg'),
        'rate': ('time_s', 'roll_deg_s', 'pitch_deg_s', 'yaw_deg_s',
                 'des_roll_deg_s', 'des_pitch_deg_s', 'des_yaw_deg_s'),
    }
    os.makedirs(output_dir, exist_ok=True)
    paths = []
    for phase, messages in samples.items():
        for message, header in columns.items():
            path = os.path.join(output_dir, '%s_%s.csv' % (phase, message))
            with open(path, 'w', newline='') as output:
                writer = csv.writer(output)
                writer.writerow(header)
                for sample in messages[message]:
                    writer.writerow((sample[0] / 1e6,) + sample[1:])
            paths.append(path)
    return paths


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', default=os.path.join(ROOT, 'build/sitl/bin/arducopter'))
    ap.add_argument('--speedup', type=int, default=1)
    ap.add_argument('--param', action='append', default=[], help='NAME=VALUE applied before flight')
    ap.add_argument('--csv-dir', help='directory for the PID/STA attitude and rate CSV files '
                                     '(default: the SITL work directory)')
    ap.add_argument('--clean', action='store_true', help='delete the work directory (and log) at the end')
    args = ap.parse_args()

    work = tempfile.mkdtemp(prefix='nlc_sitl_')
    print('work dir:', work)
    sitl = SITL(args.binary, work, args.speedup)
    try:
        # first boot: configure, then reboot so CC_TYPE takes effect
        sitl.start(wipe=True)
        for name, val in [('CC_TYPE', 3), ('CC_AXIS_MASK', 7), ('RC6_OPTION', 109),
                          ('LOG_BITMASK', 180223), ('LOG_DISARMED', 0)]:
            sitl.set_param(name, val)
        sitl.stop()
        sitl.start()
        for kv in ['CC3_OPT=1'] + args.param:
            k, v = kv.split('=')
            sitl.set_param(k, float(v))

        # wait for the EKF and arm in GUIDED
        sitl.mode('GUIDED')
        t_end = time.time() + 180
        while time.time() < t_end:
            sitl.rc(thr=1000)
            sitl.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            sitl.pump(1.0, thr=1000)
            if sitl.mav.motors_armed():
                break
        else:
            raise RuntimeError('could not arm')
        print('armed')
        sitl.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
        t_end = time.time() + 60
        while sitl.alt() < 9.0:
            sitl.pump(0.2, thr=1500)
            if time.time() > t_end:
                raise RuntimeError('takeoff timeout')
        print('altitude reached, switching to ALT_HOLD')
        sitl.mode('ALT_HOLD')
        sitl.pump(3.0 / args.speedup)

        print('phase A: Testing main controller (P+PID)')
        win_a = fly_profile(sitl, 1000, args.speedup)

        print('phase B: Testing NLC Super-Twisting controller')
        #switch to NLC controller by raising RC6, then fly the same profile
        sitl.pump(1.0 / args.speedup, ch6=2000)
        win_b = fly_profile(sitl, 2000, args.speedup)
        #turn NLC off again so we can land with the main controller
        sitl.pump(1.0 / args.speedup, ch6=1000)

        sitl.mode('LAND')
        t_end = time.time() + 120
        while sitl.mav.motors_armed() and time.time() < t_end:
            sitl.pump(0.5)
        sitl.stop()

        logs = sorted(glob.glob(os.path.join(work, 'logs', '*.BIN')), key=os.path.getmtime)
        res, samples = analyse(logs[-1], {'PID': win_a, 'STA': win_b})
        csv_dir = os.path.abspath(args.csv_dir) if args.csv_dir else work
        csv_paths = write_samples(samples, csv_dir)

        print('\nRMS tracking error (deg/s for rate, deg for attitude)')
        print('%-6s %8s %8s %8s %8s %8s %6s' % ('', 'rate R', 'rate P', 'rate Y', 'att R', 'att P', 'n'))
        for k in ('PID', 'STA'):
            r = res[k]
            print('%-6s %8.3f %8.3f %8.3f %8.3f %8.3f %6d' % (k, r['R'], r['P'], r['Y'], r['aR'], r['aP'], r['n']))
        print('\ncontrol roughness, RMS of per-sample change of mixer input')
        print('%-6s %8s %8s %8s' % ('', 'dU R', 'dU P', 'dU Y'))
        for k in ('PID', 'STA'):
            r = res[k]
            print('%-6s %8.5f %8.5f %8.5f' % (k, r['dUR'], r['dUP'], r['dUY']))
        print('\nimprovement STA vs PID (positive = STA better)')
        for ax in ('R', 'P', 'Y'):
            a, b = res['PID'][ax], res['STA'][ax]
            print('  rate %s: %+6.1f %%' % (ax, 100.0 * (a - b) / a))
        print('\nRPY sample CSVs:')
        for path in csv_paths:
            print(' ', path)
        print('log:', logs[-1])
    finally:
        sitl.stop()
        if args.clean:
            shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    main()
