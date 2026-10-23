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
plotting measured and desired roll, pitch, and yaw over time, plus the STA
output (CNLC) and the per-motor output chain (CCMO: mixer thrust, actuator, PWM).
"""

import argparse
import csv
import datetime
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

# number of motors logged in CCMO (quad)
N_MOTORS = 4

# motor output chain configuration, applied BEFORE phase A so that both
# controllers fly exactly the same plant. CC_OPTIONS=1 enables the CCMO log.
MOTOR_SETUP = [('CC_OPTIONS', 1), ('MOT_THST_EXPO', 0), ('MOT_BAT_VOLT_MAX', 0),
               ('MOT_SLEW_UP_TIME', 0), ('MOT_SLEW_DN_TIME', 0), ('FSTRATE_ENABLE', 0)]

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

#usar modo circulo o guided para que el controlador haga el movimiento del dron.
#Modo circulo 


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
    """fly PROFILE once; return (t0_us, t1_us) and the start of every stick segment.
    Segment start times come from SYSTEM_TIME (~0.1 s resolution); the plotting
    script refines them with the desired-rate signal."""
    t0 = sitl.boot_ms()
    segments = []
    for dur, r, p, y in PROFILE:
        segments.append((sitl.boot_ms() * 1000, dur, r, p, y))
        sitl.pump(dur / speedup, roll=r, pitch=p, yaw=y, ch6=ch6)
    t1 = sitl.boot_ms()
    return (t0 * 1000, t1 * 1000), segments


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float('nan')


def analyse(logfile, windows):
    """windows: dict name -> (t0_us, t1_us)"""
    #Dictionary of lists for each test phase (PID and STA)
    res = {k: {'rate_Re': [], 'rate_Pe': [], 'rate_Ye': [], 'att_Re': [], 'att_Pe': [], 'sR': [], 'sP': [], 'sY': [], 'oR': [], 'oP': [], 'oY': []} for k in windows}
    samples = {k: {'attitude': [], 'rate': [], 'cnlc': [], 'ccmo': []} for k in windows}
    ccmo_open = {k: None for k in windows}   # CCMO row being assembled per phase
    log = DFReader.DFReader_binary(logfile, zero_time_base=False)
    while True:
        m = log.recv_match(type=['RATE', 'ATT', 'CNLC', 'CCMO'])
        if m is None:
            break
        t = m.TimeUS
        #print(f'message: {m.get_type()} time: {t}')
        for k, (a, b) in windows.items():
            if a <= t <= b: #confirm the time is within the window for this Test_phase ( Test phase A or Test phase B)
                d = res[k]#dictionary for the current test phase
                if m.get_type() == 'RATE':
                    #Calculate the error 'e' between the desired and actual Roll, Pitch and Yaw values in deg/sec.
                    #and append them to the corresponding lists for later RMS calculation
                    d['rate_Re'].append(m.RDes - m.R)
                    d['rate_Pe'].append(m.PDes - m.P)
                    d['rate_Ye'].append(m.YDes - m.Y)
                    # Controller Roll, Pitch, Yaw output to the motor mixer
                    d['oR'].append(m.ROut)
                    d['oP'].append(m.POut)
                    d['oY'].append(m.YOut)
                    #save time stamp, current R,P,Y and target R,P,Y for CSV output
                    samples[k]['rate'].append((t - a, m.R, m.P, m.Y, m.RDes, m.PDes, m.YDes,
                                               m.ROut, m.POut, m.YOut))
                elif m.get_type() == 'ATT':
                    #save Attitude error between desired and actual Roll and Pitch values in deg.
                    d['att_Re'].append(m.DesRoll - m.Roll)
                    d['att_Pe'].append(m.DesPitch - m.Pitch)
                    samples[k]['attitude'].append(
                        (t - a, m.Roll, m.Pitch, m.Yaw, m.DesRoll, m.DesPitch, m.DesYaw))
                elif m.get_type() == 'CNLC':
                    d[['sR', 'sP', 'sY'][m.I]].append(math.degrees(m.S))
                    if m.I == 0:
                        I0 = [t-a,m.U, None,None]
                        samples[k]['cnlc'].append(I0)
                    elif samples[k]['cnlc']:
                        # axes 1 and 2 complete the row opened by axis 0; if the window started
                        # mid-sample there is no open row yet, so they are skipped
                        samples[k]['cnlc'][-1][m.I+1] = m.U
                elif m.get_type() == 'CCMO':
                    # one CCMO message per motor and loop, all with the same TimeUS.
                    # Assemble them into one wide row per loop:
                    #   [t, CC, Thr_m1..m4, Out_m1..m4, PWM_m1..m4]
                    row = ccmo_open[k]
                    if m.I == 0:
                        row = [t - a, m.CC] + [None] * (3 * N_MOTORS)
                        ccmo_open[k] = row
                        samples[k]['ccmo'].append(row)
                    elif row is None or row[0] != t - a:
                        # window started in the middle of a loop: skip until motor 0
                        continue
                    if m.I < N_MOTORS:
                        row[2 + m.I] = m.Thr
                        row[2 + N_MOTORS + m.I] = m.Out
                        row[2 + 2 * N_MOTORS + m.I] = m.PWM
                else:
                    raise RuntimeError('unexpected message type %s' % m.get_type())
    out = {}
    for k, d in res.items():
        out[k] = {key: rms(v) for key, v in d.items() if not key.startswith('o')}
        # control-effort roughness: RMS of the sample-to-sample change of the
        # mixer input (RATE.ROut/POut/YOut = output of whichever controller is active)
        for ax in 'RPY':
            o = d['o' + ax]
            out[k]['dU' + ax] = rms([o[i+1] - o[i] for i in range(len(o) - 1)])
        out[k]['n'] = len(d['rate_Re'])  # number of samples in this phase
    #Convert from list to tuple to comply with the CSV writer that write tuples, not lists
    for k in samples:
        samples[k]['cnlc'] = [tuple(row) for row in samples[k]['cnlc']]
        # keep only complete CCMO rows (all motors present)
        samples[k]['ccmo'] = [tuple(row) for row in samples[k]['ccmo'] if None not in row]
    return out, samples


def write_samples(samples, output_dir):
    """Write measured and desired RPY samples to one CSV per phase and message."""
    columns = {
        'attitude': ('time_s', 'roll_deg', 'pitch_deg', 'yaw_deg',
                     'des_roll_deg', 'des_pitch_deg', 'des_yaw_deg'),
        'rate': ('time_s', 'roll_deg_s', 'pitch_deg_s', 'yaw_deg_s',
                 'des_roll_deg_s', 'des_pitch_deg_s', 'des_yaw_deg_s',
                 # mixer input of whichever controller is active (RATE.ROut/POut/YOut)
                 'out_roll', 'out_pitch', 'out_yaw'),
        'cnlc': ('time_s', 'U_roll', 'U_pitch', 'U_yaw'),
        'ccmo': (('time_s', 'CC')
                 + tuple('Thr_m%d' % (i + 1) for i in range(N_MOTORS))
                 + tuple('Out_m%d' % (i + 1) for i in range(N_MOTORS))
                 + tuple('PWM_m%d' % (i + 1) for i in range(N_MOTORS))),
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


def write_segments(segments, windows, output_dir):
    """one row per stick segment and phase, times relative to the phase window start"""
    path = os.path.join(output_dir, 'segments.csv')
    with open(path, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(('phase', 'index', 't_start_s', 'dur_nominal_s',
                         'roll_pwm', 'pitch_pwm', 'yaw_pwm'))
        for phase, segs in segments.items():
            a = windows[phase][0]
            for i, (t_us, dur, r, p, y) in enumerate(segs):
                writer.writerow((phase, i, (t_us - a) / 1e6, dur, r, p, y))
    return path


def write_params(logfile, output_dir):
    """last value of every parameter in the log (PARM), for reproducibility"""
    values = {}
    log = DFReader.DFReader_binary(logfile, zero_time_base=False)
    while True:
        m = log.recv_match(type='PARM')
        if m is None:
            break
        values[m.Name] = m.Value
    path = os.path.join(output_dir, 'params.csv')
    with open(path, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(('name', 'value'))
        for name in sorted(values):
            writer.writerow((name, values[name]))
    return path


def clean_workdir(path, preexisting):
    """Remove only what this run created in path; preexisting is None if path itself was created."""
    if preexisting is None:
        shutil.rmtree(path, ignore_errors=True)
        return
    for name in set(os.listdir(path)) - preexisting:
        target = os.path.join(path, name)
        if os.path.isdir(target) and not os.path.islink(target):
            shutil.rmtree(target, ignore_errors=True)
        else:
            os.remove(target)


def main():

    #save the log and CSVs in /tmp/nlc_sitl_<timestamp> if no other directory is given, so that the user can inspect them after the run
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    foldername = 'nlc_sitl_'+timestamp
    work = os.path.join(tempfile.gettempdir(),foldername)

    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', default=os.path.join(ROOT, 'build/sitl/bin/arducopter'))
    ap.add_argument('--speedup', type=int, default=5)
    ap.add_argument('--param', action='append', default=[], help='NAME=VALUE applied before flight')
    ap.add_argument('--log-dir', help='directory for the PID/STA attitude and rate logs and CSV files ', default=work)
    ap.add_argument('--clean', action='store_true', help='at the end, delete the files and directories '
                                                           'this run created in the log directory')
    args = ap.parse_args()

    
    # --log-dir is the parent directory; each run gets its own nlc_sitl_<timestamp> folder
    if args.log_dir != work:
        args.log_dir = os.path.join(args.log_dir, foldername)
    args.log_dir = os.path.abspath(args.log_dir)
    preexisting = set(os.listdir(args.log_dir)) if os.path.isdir(args.log_dir) else None
    os.makedirs(args.log_dir, exist_ok=True)
    
    print('work dir:', args.log_dir)
    sitl = SITL(args.binary, args.log_dir, args.speedup)
    try:
        # first boot: configure, then reboot so CC_TYPE takes effect
        sitl.start(wipe=True)
        for name, val in [('CC_TYPE', 3), ('CC_AXIS_MASK', 7), ('RC6_OPTION', 109),
                          ('LOG_BITMASK', 180223), ('LOG_DISARMED', 0)]:
            sitl.set_param(name, val)
        sitl.stop()
        sitl.start()
        # same motor output chain for both phases; --param can still override it
        for name, val in MOTOR_SETUP:
            sitl.set_param(name, val)
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
        win_a, seg_a = fly_profile(sitl, 1000, args.speedup)

        print('phase B: Testing NLC Super-Twisting controller')
        #switch to NLC controller by raising RC6, then fly the same profile
        sitl.pump(1.0 / args.speedup, ch6=2000)
        win_b, seg_b = fly_profile(sitl, 2000, args.speedup)
        #turn NLC off again so we can land with the main controller
        sitl.pump(1.0 / args.speedup, ch6=1000)

        sitl.mode('LAND')
        t_end = time.time() + 120
        while sitl.mav.motors_armed() and time.time() < t_end:
            sitl.pump(0.5)
        sitl.stop()

        logs = sorted(glob.glob(os.path.join(args.log_dir, 'logs', '*.BIN')), key=os.path.getmtime)
        res, samples = analyse(logs[-1], {'PID': win_a, 'STA': win_b})
        csv_paths = write_samples(samples, args.log_dir)
        csv_paths.append(write_segments({'PID': seg_a, 'STA': seg_b},
                                        {'PID': win_a, 'STA': win_b}, args.log_dir))
        csv_paths.append(write_params(logs[-1], args.log_dir))

        print('\nRMS tracking error (deg/s for rate, deg for attitude)')
        print('%-6s %8s %8s %8s %8s %8s %6s' % ('', 'rate R', 'rate P', 'rate Y', 'att R', 'att P', 'n'))
        for k in ('PID', 'STA'):
            r = res[k]
            print('%-6s %8.3f %8.3f %8.3f %8.3f %8.3f %6d' % (k, r['rate_Re'], r['rate_Pe'], r['rate_Ye'], r['att_Re'], r['att_Pe'], r['n']))
        print('\ncontrol roughness, RMS of per-sample change of mixer input')
        print('%-6s %8s %8s %8s' % ('', 'dU R', 'dU P', 'dU Y'))
        for k in ('PID', 'STA'):
            r = res[k]
            print('%-6s %8.5f %8.5f %8.5f' % (k, r['dUR'], r['dUP'], r['dUY']))
        print('\nimprovement STA vs PID (positive = STA better)')
        for ax in ('R', 'P', 'Y'):
            pid_rmse, sta_rmse = res['PID']['rate_'+ax+'e'], res['STA']['rate_'+ax+'e']
            print('  rate %s: %+6.1f %%' % (ax, 100.0 * (pid_rmse - sta_rmse) / pid_rmse))
        print('\nSample CSVs:')
        for path in csv_paths:
            print(' ', path)
        print('log:', logs[-1])
    finally:
        sitl.stop()
        if args.clean:
            clean_workdir(args.log_dir, preexisting)


if __name__ == '__main__':
    main()
