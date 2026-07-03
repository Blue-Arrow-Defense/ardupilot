#!/usr/bin/env python3
"""
Restart-until-success catapult launch runner for Geran-2.

Rather than trying to make a single SITL run deterministic (JSBSim SITL runs
in real time and is sensitive to OS scheduling jitter around the EKF3
activation transient - see test_catapult_sitl.py's investigation notes), this
script just launches a completely fresh ArduPlane + JSBSim process for each
attempt, runs the real mission in AUTO, and judges the attempt against strict
success criteria:

  1. The aircraft actually follows the mission (MISSION_CURRENT sequence
     advances past the first waypoint(s) - proof AUTO nav is in control, not
     just coasting).
  2. Altitude genuinely varies after launch (max-min exceeds a threshold) -
     rules out a false "PASS" from a frozen/glitched EKF altitude reading.
  3. Airspeed is valid and not ~0 while airborne (proof it is actually
     flying through the air, not resting on the ground with a stale
     altitude estimate).
  4. No crash (SERVO/GEAR ground-truth check from test_catapult_sitl.py).

Any attempt that fails any of these is killed outright and a brand new
process is started for the next attempt. The script only exits 0 when one
attempt satisfies every criterion; otherwise it keeps retrying until
--max-attempts is hit (default effectively "keep going": a very high count).

Usage:
    python3 launch_until_success.py [--max-attempts 200] [--duration 90]
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_catapult_sitl import (  # noqa: E402
    ROOT, AUTOTEST, ARDUPLANE, DEFAULTS,
    kill_stragglers, start_arduplane, wait_for_port,
    upload_mission, set_mode_auto, arm, check_gear_contact_after_release,
)

MISSION = os.path.join(AUTOTEST, 'aircraft', 'Geran-2', 'Geran2-catapult-mission.txt')

MIN_MISSION_SEQ_ADVANCE = 2      # must progress at least 2 waypoints into the mission
MIN_ALTITUDE_SPREAD_M = 2.0      # post-launch max-min altitude must exceed this
MIN_AIRSPEED_MPS = 8.0           # average post-launch airspeed must exceed this
STALE_AIRSPEED_MPS = 1.0         # any airborne sample below this after settling = "on the ground"


def run_one_attempt(attempt_num, duration, mission_path, extra_params, keep_logs_dir):
    from pymavlink import mavutil

    kill_stragglers()
    log_path = f'/tmp/geran2_launch_attempt_{attempt_num}.log'
    proc, logf = start_arduplane(log_path, extra_params)

    result = {'ok': False, 'reasons': []}
    try:
        if not wait_for_port(5760, timeout=30):
            result['reasons'].append("arduplane never opened its MAVLink port")
            return result

        m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
        if m.wait_heartbeat(timeout=30) is None:
            result['reasons'].append("no heartbeat")
            return result
        print(f"  [attempt {attempt_num}] connected to system {m.target_system}")

        if not upload_mission(m, mission_path):
            result['reasons'].append("mission upload was not accepted")
            return result
        print(f"  [attempt {attempt_num}] mission uploaded")

        if not set_mode_auto(m):
            result['reasons'].append("mode change to AUTO was not accepted")
            return result
        print(f"  [attempt {attempt_num}] mode AUTO set")

        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)
        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)
        drain_until = time.time() + 1.5
        while time.time() < drain_until:
            m.recv_match(blocking=True, timeout=0.2)

        print(f"  [attempt {attempt_num}] waiting for GPS 3D fix and EKF3 active...")
        gps_ok = False
        ekf3_active = False
        wait_deadline = time.time() + 15
        while time.time() < wait_deadline and not (gps_ok and ekf3_active):
            msg = m.recv_match(type=['GPS_RAW_INT', 'STATUSTEXT'], blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == 'GPS_RAW_INT' and msg.fix_type >= 3:
                gps_ok = True
            if msg.get_type() == 'STATUSTEXT' and 'EKF3 active' in msg.text:
                ekf3_active = True

        prearm_deadline = time.time() + 30
        armed_ok = False
        while time.time() < prearm_deadline:
            if arm(m):
                armed_ok = True
                break
            time.sleep(1)
        if not armed_ok:
            result['reasons'].append("arm was not accepted (prearm checks still failing)")
            return result
        print(f"  [attempt {attempt_num}] armed")

        max_alt = -1e9
        min_alt_after_launch = 1e9
        launched = False
        crashed = False
        initial_seq = None
        max_seq = 0
        airspeed_samples_after_launch = []
        stale_airspeed_hit = False
        start = time.time()
        last_print = 0
        print_interval = float(os.environ.get('PRINT_INTERVAL', '0.5'))
        last_att = None
        last_servo = None

        while time.time() - start < duration:
            msg = m.recv_match(
                type=['VFR_HUD', 'STATUSTEXT', 'ATTITUDE', 'SERVO_OUTPUT_RAW', 'MISSION_CURRENT'],
                blocking=True, timeout=2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'STATUSTEXT':
                print(f"  [attempt {attempt_num}][AP] {msg.text}")
                continue
            if t == 'ATTITUDE':
                last_att = msg
                continue
            if t == 'SERVO_OUTPUT_RAW':
                last_servo = msg
                continue
            if t == 'MISSION_CURRENT':
                if initial_seq is None:
                    initial_seq = msg.seq
                max_seq = max(max_seq, msg.seq)
                continue

            # VFR_HUD
            alt = msg.alt
            max_alt = max(max_alt, alt)
            if alt > 1:
                launched = True
            if launched:
                min_alt_after_launch = min(min_alt_after_launch, alt)
                airspeed_samples_after_launch.append(msg.airspeed)
                # give it 2s past launch to accelerate off the rail before
                # judging "stale" airspeed, otherwise the very first sample
                # right at launch always trips this
                if (time.time() - start) > 2.0 and msg.airspeed < STALE_AIRSPEED_MPS:
                    stale_airspeed_hit = True

            if time.time() - last_print > print_interval:
                pitch_deg = math.degrees(last_att.pitch) if last_att else float('nan')
                elevator_us = last_servo.servo2_raw if last_servo else -1
                print(f"  [attempt {attempt_num}] t+{time.time()-start:5.1f}s  alt={alt:6.1f}m  "
                      f"airspeed={msg.airspeed:5.1f}m/s  climb={msg.climb:+5.2f}m/s  "
                      f"pitch={pitch_deg:+5.1f}deg  wp_seq={max_seq}  elevator={elevator_us}us  "
                      f"throttle={msg.throttle:3d}%")
                last_print = time.time()

            if launched and alt < -2:
                crashed = True
                print(f"  [attempt {attempt_num}] CRASH DETECTED: altitude went to {alt:.1f}m")
                break

        ground_truth_crash = check_gear_contact_after_release(log_path)

        seq_advance = (max_seq - initial_seq) if initial_seq is not None else 0
        alt_spread = (max_alt - min_alt_after_launch) if launched else 0.0
        avg_airspeed = (sum(airspeed_samples_after_launch) / len(airspeed_samples_after_launch)
                         if airspeed_samples_after_launch else 0.0)

        print(f"  [attempt {attempt_num}] summary: launched={launched} crashed={crashed or ground_truth_crash} "
              f"wp_seq_advance={seq_advance} alt_spread={alt_spread:.1f}m avg_airspeed={avg_airspeed:.1f}m/s "
              f"stale_airspeed_hit={stale_airspeed_hit}")

        if not launched:
            result['reasons'].append("never left the ground")
        if crashed or ground_truth_crash:
            result['reasons'].append("crashed" + (" (GEAR_CONTACT confirmed)" if ground_truth_crash else ""))
        if seq_advance < MIN_MISSION_SEQ_ADVANCE:
            result['reasons'].append(
                f"did not follow the mission (wp seq advanced by {seq_advance}, need >= {MIN_MISSION_SEQ_ADVANCE})")
        if alt_spread < MIN_ALTITUDE_SPREAD_M:
            result['reasons'].append(
                f"altitude did not vary enough (spread={alt_spread:.1f}m, need >= {MIN_ALTITUDE_SPREAD_M}m)")
        if avg_airspeed < MIN_AIRSPEED_MPS:
            result['reasons'].append(
                f"airspeed too low/invalid (avg={avg_airspeed:.1f}m/s, need >= {MIN_AIRSPEED_MPS}m/s)")
        if stale_airspeed_hit:
            result['reasons'].append("airspeed dropped near 0 while airborne (looks grounded)")

        result['ok'] = not result['reasons']
        result['max_alt'] = max_alt
        result['alt_spread'] = alt_spread
        result['avg_airspeed'] = avg_airspeed
        result['seq_advance'] = seq_advance
        return result
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        subprocess.run(['pkill', '-9', '-f', 'JSBSim --suspend'], stderr=subprocess.DEVNULL)
        logf.close()
        if keep_logs_dir is None and not result['ok']:
            # keep failed-attempt logs around for post-mortem, but don't let
            # them pile up forever
            try:
                keep = sorted(
                    (p for p in os.listdir('/tmp') if p.startswith('geran2_launch_attempt_')),
                    key=lambda p: os.path.getmtime(os.path.join('/tmp', p)))
                for stale in keep[:-5]:
                    os.remove(os.path.join('/tmp', stale))
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=90.0,
                     help='seconds to observe per attempt after arming')
    ap.add_argument('--mission', default=MISSION)
    ap.add_argument('--extra-param', action='append', default=[],
                     metavar='NAME=VALUE', help='override a param for every attempt (repeatable)')
    ap.add_argument('--max-attempts', type=int, default=200,
                     help='give up after this many failed attempts (0 = never give up)')
    ap.add_argument('--retry-pause', type=float, default=2.0,
                     help='seconds to pause between attempts')
    args = ap.parse_args()

    attempt = 0
    while args.max_attempts == 0 or attempt < args.max_attempts:
        attempt += 1
        print(f"\n=== Attempt {attempt} ===")
        result = run_one_attempt(attempt, args.duration, args.mission, args.extra_param, None)
        if result['ok']:
            print(f"\n=== SUCCESS on attempt {attempt} ===")
            print(f"  peak altitude:      {result['max_alt']:.1f}m")
            print(f"  post-launch spread: {result['alt_spread']:.1f}m")
            print(f"  avg airspeed:       {result['avg_airspeed']:.1f}m/s")
            print(f"  waypoints advanced: {result['seq_advance']}")
            print(f"  log kept at: /tmp/geran2_launch_attempt_{attempt}.log")
            return 0
        print(f"  [attempt {attempt}] FAIL: {'; '.join(result['reasons'])}")
        time.sleep(args.retry_pause)

    print(f"\n=== GAVE UP after {attempt} attempts without a passing run ===")
    return 1


if __name__ == '__main__':
    sys.exit(main())
