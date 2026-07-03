#!/usr/bin/env python3
"""
End-to-end catapult launch test for Geran-2, driving the real ArduPlane
SITL binary over MAVLink (not a standalone JSBSim replay) - this is the only
way to exercise ArduPilot's own launch-detection and takeoff-pitch control,
which is where the catapult launch has been failing.

Starts arduplane itself (killing any stray instance first), waits for it,
uploads the given mission, switches to AUTO, arms, then prints a live
altitude/airspeed/climb-rate trace and verdicts success/failure based on
whether the aircraft sustains positive altitude after the catapult fires.

Usage:
    python3 test_catapult_sitl.py [--duration 60] [--extra-param NAME=VALUE ...]

Requires: pymavlink (pip install pymavlink)
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..'))
AUTOTEST = os.path.join(ROOT, 'Tools', 'autotest')
ARDUPLANE = os.path.join(ROOT, 'build', 'sitl', 'bin', 'arduplane')
MISSION = os.path.join(AUTOTEST, 'Generic_Missions', 'CMAC-circuit.txt')
PARM = os.path.join(AUTOTEST, 'aircraft', 'Geran-2', 'Geran-2-catapult.parm')
DEFAULTS = os.path.join(AUTOTEST, 'default_params', 'plane-jsbsim.parm') + ',' + PARM


def kill_stragglers():
    subprocess.run(['pkill', '-9', '-f', 'bin/arduplane'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'JSBSim --suspend'], stderr=subprocess.DEVNULL)
    time.sleep(1)


def start_arduplane(log_path, extra_params):
    defaults = DEFAULTS
    if extra_params:
        # write a throwaway override file so we don't touch Geran-2-catapult.parm
        override_path = log_path + '.override.parm'
        with open(override_path, 'w') as f:
            for kv in extra_params:
                name, value = kv.split('=', 1)
                f.write(f"{name} {value}\n")
        defaults += ',' + override_path
    logf = open(log_path, 'w')
    proc = subprocess.Popen(
        # -w wipes the persisted eeprom.bin on every start - without it,
        # any parameter that was ever non-default in a past run (e.g. an
        # earlier TKOFF_ROTATE_SPD value) stays stuck in storage and
        # silently overrides whatever --defaults now says, since a
        # defaults file only seeds a parameter that's still at its
        # compiled-in default. Confirmed directly: editing
        # Geran-2-catapult.parm had zero effect on behavior until this
        # wipe was added.
        [ARDUPLANE, '-S', '--model', 'jsbsim:Geran-2-catapult',
         '--speedup', '1', '--slave', '0', '--defaults', defaults,
         '--sim-address=127.0.0.1', '-I0', '-w'],
        cwd=AUTOTEST, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc, logf


def check_gear_contact_after_release(log_path):
    """Ground-truth crash check: any GEAR_CONTACT logged after Catapult
    Release means the aircraft touched down again - a real crash, even if
    MAVLink telemetry looked fine (EKF/home-position can glitch after a
    physical impact and make altitude misleadingly look stable)."""
    ansi_re = re.compile(r'\x1b\[[0-9;]*m')
    release_time = None
    with open(log_path) as f:
        for raw_line in f:
            line = ansi_re.sub('', raw_line)
            if release_time is None:
                m = re.search(r'Catapult Release.*executed at time:\s*([\d.]+)', line)
                if m:
                    release_time = float(m.group(1))
                continue
            m = re.search(r'GEAR_CONTACT:\s*([\d.]+) seconds', line)
            if m and float(m.group(1)) > release_time:
                return True
    return False


def wait_for_port(port, timeout=30):
    import socket
    start = time.time()
    while time.time() - start < timeout:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.settimeout(0.5)
            s.connect(('127.0.0.1', port))
            s.close()
            return True
        except OSError:
            time.sleep(0.3)
        finally:
            s.close()
    return False


def upload_mission(m, mission_path):
    from pymavlink import mavutil, mavwp
    wp = mavwp.MAVWPLoader()
    wp.load(mission_path)
    print(f"  [mission] target_system={m.target_system} target_component={m.target_component}")
    print(f"  [mission] sending MISSION_COUNT={wp.count()}")
    sent = 0
    deadline = time.time() + 30
    last_hb = 0
    last_count_send = 0
    while sent < wp.count() and time.time() < deadline:
        now = time.time()
        if now - last_hb > 1.0:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = now
        if now - last_count_send > 2.0:
            m.mav.mission_count_send(m.target_system, m.target_component, wp.count())
            last_count_send = now
        msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=1)
        if msg is None:
            continue
        print(f"  [mission] got request for seq={msg.seq}")
        w = wp.wp(msg.seq)
        w.target_system = m.target_system
        w.target_component = m.target_component
        m.mav.send(w)
        sent += 1
    ack = m.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    print(f"  [mission] sent={sent}/{wp.count()} ack={ack}")
    return ack is not None and ack.type == 0  # MAV_MISSION_ACCEPTED


def set_mode(m, mode_num):
    from pymavlink import mavutil
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_num, 0, 0, 0, 0, 0)
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=10,
                        condition='COMMAND_ACK.command==176')
    return ack is not None and ack.result == 0


def set_mode_auto(m):
    return set_mode(m, 10)  # AUTO


def set_mode_cruise(m):
    return set_mode(m, 7)  # CRUISE


def set_mode_fbwa(m):
    return set_mode(m, 5)  # FBWA


def arm(m):
    from pymavlink import mavutil
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 21196, 0, 0, 0, 0, 0)
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=10,
                        condition='COMMAND_ACK.command==400')
    return ack is not None and ack.result == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0,
                     help='seconds to observe after arming')
    ap.add_argument('--mission', default=MISSION)
    ap.add_argument('--extra-param', action='append', default=[],
                     metavar='NAME=VALUE',
                     help='override a param for this run only (repeatable)')
    ap.add_argument('--keep-log', action='store_true',
                     help='do not delete the arduplane log on exit')
    ap.add_argument('--cruise-mode', action='store_true',
                     help='arm directly into CRUISE instead of AUTO+NAV_TAKEOFF, '
                          'bypassing flight_stage==TAKEOFF ground-launch logic entirely')
    ap.add_argument('--fbwa-mode', action='store_true',
                     help='arm directly into FBWA instead of AUTO+NAV_TAKEOFF')
    ap.add_argument('--rc-pitch-up', type=int, default=0,
                     metavar='PWM_OFFSET',
                     help='send a sustained RC pitch-up override (elevator '
                          'stick back) of this many us above trim, for use '
                          'with --fbwa-mode to induce an active climb')
    args = ap.parse_args()

    from pymavlink import mavutil

    kill_stragglers()
    log_path = '/tmp/catapult_sitl_test.log'
    proc, logf = start_arduplane(log_path, args.extra_param)

    try:
        if not wait_for_port(5760, timeout=30):
            print("FAIL: arduplane never opened its MAVLink port")
            return 1

        m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
        if m.wait_heartbeat(timeout=30) is None:
            print("FAIL: no heartbeat")
            return 1
        print(f"Connected to system {m.target_system}")

        if args.cruise_mode:
            if not set_mode_cruise(m):
                print("FAIL: mode change to CRUISE was not accepted")
                return 1
            print("Mode CRUISE set")
        elif args.fbwa_mode:
            if not set_mode_fbwa(m):
                print("FAIL: mode change to FBWA was not accepted")
                return 1
            print("Mode FBWA set")
        else:
            if not upload_mission(m, args.mission):
                print("FAIL: mission upload was not accepted")
                return 1
            print(f"Mission uploaded from {args.mission}")

            if not set_mode_auto(m):
                print("FAIL: mode change to AUTO was not accepted")
                return 1
            print("Mode AUTO set")

        from pymavlink import mavutil
        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)
        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)
        # drain a moment so the stream is actually flowing at the higher
        # rate before we arm - otherwise the first post-arm samples can
        # lag behind on the old (slower) default rate
        drain_until = time.time() + 1.5
        while time.time() < drain_until:
            m.recv_match(blocking=True, timeout=0.2)

        # Wait for GPS 3D fix AND EKF3 to actually become the active AHRS
        # source before ever arming - a real launch procedure always
        # waits for this; arming/firing within a couple seconds (as all
        # earlier tests did) launches while ArduPilot is still relying on
        # a less-mature attitude estimate, which lines up with the wild
        # post-launch divergence seen when skipping this wait.
        print("Waiting for GPS 3D fix and EKF3 active...")
        gps_ok = False
        ekf3_active = False
        wait_deadline = time.time() + 15
        while time.time() < wait_deadline and not (gps_ok and ekf3_active):
            msg = m.recv_match(type=['GPS_RAW_INT', 'STATUSTEXT'], blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == 'GPS_RAW_INT' and msg.fix_type >= 3:
                gps_ok = True
            if msg.get_type() == 'STATUSTEXT':
                if 'EKF3 active' in msg.text:
                    ekf3_active = True
                print(f"  [AP] {msg.text}")
        print(f"GPS 3D fix: {gps_ok}, EKF3 active: {ekf3_active}")

        # let EKF/GPS settle before arming, mirroring real usage
        prearm_deadline = time.time() + 30
        armed_ok = False
        while time.time() < prearm_deadline:
            if arm(m):
                armed_ok = True
                break
            time.sleep(1)
        if not armed_ok:
            print("FAIL: arm was not accepted (prearm checks likely still failing)")
            return 1
        print("Armed")

        max_alt = -1e9
        min_alt_after_launch = 1e9
        launched = False
        crashed = False
        start = time.time()
        last_print = 0
        print_interval = float(os.environ.get('PRINT_INTERVAL', '0.2'))
        last_vfr = None
        last_att = None
        last_servo = None
        last_rc_send = 0
        while time.time() - start < args.duration:
            if args.rc_pitch_up:
                now = time.time()
                if now - last_rc_send > 0.1:
                    pitch_pwm = 1500 + args.rc_pitch_up
                    m.mav.rc_channels_override_send(
                        m.target_system, m.target_component,
                        1500, pitch_pwm, 1500, 1500, 0, 0, 0, 0)
                    last_rc_send = now
            msg = m.recv_match(type=['VFR_HUD', 'STATUSTEXT', 'ATTITUDE', 'SERVO_OUTPUT_RAW'],
                                blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                print(f"  [AP] {msg.text}")
                continue
            if msg.get_type() == 'ATTITUDE':
                last_att = msg
                continue
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                last_servo = msg
                continue
            last_vfr = msg
            alt = msg.alt
            max_alt = max(max_alt, alt)
            if alt > 1:
                launched = True
            if launched:
                min_alt_after_launch = min(min_alt_after_launch, alt)
            if time.time() - last_print > print_interval:
                pitch_deg = math.degrees(last_att.pitch) if last_att else float('nan')
                elevator_us = last_servo.servo2_raw if last_servo else -1
                print(f"  t+{time.time()-start:5.1f}s  alt={alt:6.1f}m  "
                      f"airspeed={msg.airspeed:5.1f}m/s  groundspeed={msg.groundspeed:5.1f}m/s  "
                      f"climb={msg.climb:+5.2f}m/s  pitch={pitch_deg:+5.1f}deg  throttle={msg.throttle:3d}%  "
                      f"elevator={elevator_us}us")
                last_print = time.time()
            if launched and alt < -2:
                crashed = True
                print(f"  CRASH DETECTED: altitude went to {alt:.1f}m")
                break

        # ground truth: check the sim log for GEAR_CONTACT after Catapult
        # Release - MAVLink altitude can glitch (EKF re-homing) after a
        # real crash and falsely look like a stable reading.
        ground_truth_crash = check_gear_contact_after_release(log_path)

        print()
        print(f"Peak altitude reached: {max_alt:.1f}m")
        if not launched:
            print("VERDICT: FAIL - never left the ground")
            return 1
        if crashed or ground_truth_crash:
            print("VERDICT: FAIL - launched but crashed"
                  + (" (confirmed by GEAR_CONTACT in sim log)" if ground_truth_crash else ""))
            return 1
        print("VERDICT: PASS - launched and stayed airborne for the full observation window")
        return 0
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        subprocess.run(['pkill', '-9', '-f', 'JSBSim --suspend'], stderr=subprocess.DEVNULL)
        logf.close()
        if not args.keep_log:
            pass  # leave log for inspection by default; nothing to clean up


if __name__ == '__main__':
    sys.exit(main())
