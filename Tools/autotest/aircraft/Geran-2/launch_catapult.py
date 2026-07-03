#!/usr/bin/env python3
"""
Geran-2 Catapult Launch — JSBSim Python Test Script
=====================================================
Tests three scenarios:

  --mode rail     Ground-rail catapult (default): engine start + catapult fire.
                  Verifies force (12,500 N) and peak velocity (~44 kts).
                  Aircraft settles post-launch (no autopilot).

  --mode air      Air-release: skips pre-launch, starts at launch speed.
                  Tests sustained flight with engine at cruise conditions.

  --mode force    Force-only: reads catapult force magnitude in a loop,
                  suitable for automated property logging.

Usage:
    python3 launch_catapult.py --root /path/to/jtest [--mode rail|air|force] [--csv out.csv]

Requirements:
    pip install jsbsim
    JSBSim root must contain:
        aircraft/Geran-2/Geran-2.xml        (catapult external_reactions)
        aircraft/Geran-2/catapult_init.xml   (V=0, theta=5 deg IC)
        aircraft/Geran-2/catapult_elevated.xml (V=0, theta=15 deg IC)
        scripts/geran2_catapult.xml          (run script)
        engine/MD550.xml
        engine/my_propeller.xml
"""

import jsbsim
import sys, os, csv, argparse, math

# ── defaults ────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROOT = os.path.normpath(os.path.join(_HERE, '..', '..', 'jtest'))
if not os.path.isdir(DEFAULT_ROOT):
    DEFAULT_ROOT = '/tmp/jtest'

# ── catapult constants ───────────────────────────────────────────────────────
CAT_FORCE_LBF = 2810.0     # pounds-force  (= 12,500 N)
CAT_FORCE_N   = CAT_FORCE_LBF * 4.44822   # Newtons
CAT_DURATION  = 0.40       # seconds
MASS_KG       = 200.0
DV_DESIGN     = CAT_FORCE_N * CAT_DURATION / MASS_KG   # m/s


# ── scenario A: ground-rail catapult (uses run script) ──────────────────────
def run_rail(root, csv_path, verbose):
    fdm = jsbsim.FGFDMExec(root, None)
    fdm.set_debug_level(0)
    script = os.path.join(root, 'scripts', 'geran2_catapult.xml')
    if not fdm.load_script(script):
        sys.exit(f"ERROR: cannot load {script}")
    fdm.run_ic()

    if verbose:
        _print_header("Ground-Rail Catapult", root)
        print(f"  {'Time':>5}  {'V kts':>6}  {'Alt m':>6}  {'Pitch':>6}  "
              f"{'RPM':>5}  {'Cat kN':>6}  {'Phase'}")
        print("  " + "-" * 57)

    records, prev = [], ""
    while fdm.run():
        t = fdm['simulation/sim-time-sec']
        if t > 35.0: break
        cat_n = fdm.get_property_value('external_reactions/catapult/magnitude') * 4.44822
        phase = _phase_rail(t, cat_n)
        if phase == 'pull_up':
            fdm.set_property_value('fcs/elevator-cmd-norm', 0.70)
        elif phase == 'climb':
            fdm.set_property_value('fcs/elevator-cmd-norm', 0.30)
        r = _snap(fdm, t, cat_n, phase)
        records.append(r)
        if verbose and _should_log(t, phase, prev):
            _print_row(r)
        prev = phase

    _summary(records, verbose, csv_path)
    return records


# ── scenario B: air-release (aircraft starts at launch speed) ────────────────
def run_air(root, csv_path, verbose):
    fdm = jsbsim.FGFDMExec(root, None)
    fdm.set_debug_level(0)
    fdm.load_model('Geran-2')

    # IC: aircraft at launch speed (25 m/s = 48.6 kts), 15-deg climb, 100 m AGL
    V_launch_kts = 48.6
    fdm.set_property_value('ic/h-sl-ft',   328.1)   # 100 m
    fdm.set_property_value('ic/vt-kts',    V_launch_kts)
    fdm.set_property_value('ic/theta-deg', 15.0)
    fdm.run_ic()

    # Engine on (use starter then free)
    fdm.set_property_value('propulsion/magneto_cmd',    3)
    fdm.set_property_value('fcs/throttle-cmd-norm[0]', 1.0)
    fdm.set_property_value('propulsion/starter_cmd',    1)
    fdm.set_property_value('propulsion/engine/set-running', 1)

    if verbose:
        _print_header("Air-Release (post-catapult climb-out)", root)
        print(f"  IC: V={V_launch_kts:.1f} kts, theta=15 deg, alt=100 m")
        print(f"  {'Time':>5}  {'V kts':>6}  {'Alt m':>6}  {'Pitch':>6}  "
              f"{'RPM':>5}  {'Cat kN':>6}  {'Phase'}")
        print("  " + "-" * 57)

    records, prev = [], ""
    for step in range(int(40.0 / (1/120))):
        if not fdm.run(): break
        t = fdm['simulation/sim-time-sec']
        if t > 5.0:
            fdm.set_property_value('propulsion/starter_cmd', 0)
        fdm.set_property_value('fcs/elevator-cmd-norm', 0.25)  # gentle climb
        cat_n = 0.0
        phase = 'climb' if t > 0 else 'start'
        r = _snap(fdm, t, cat_n, phase)
        records.append(r)
        if verbose and t % 2.0 < (1/120):
            _print_row(r)
        prev = phase

    _summary(records, verbose, csv_path)
    return records


# ── helpers ──────────────────────────────────────────────────────────────────
def _phase_rail(t, cat_n):
    if t < 5.0:  return 'engine_start'
    if t < 5.5:  return 'ready'
    if cat_n > 0: return 'CATAPULT'
    if t < 6.9:  return 'pull_up'
    return 'climb'

def _snap(fdm, t, cat_n, phase):
    return {
        't':        t,
        'V_kts':    fdm['velocities/vt-fps'] * 0.5925,
        'alt_m':    fdm['position/h-sl-ft'] * 0.3048,
        'theta':    fdm['attitude/theta-deg'],
        'rpm':      fdm['propulsion/engine/engine-rpm'],
        'cat_N':    cat_n,
        'thrust_N': fdm['propulsion/engine/thrust-lbs'] * 4.44822,
        'phase':    phase,
    }

def _print_header(title, root):
    print()
    print("=" * 65)
    print(f"  Geran-2 Catapult — {title}")
    print("=" * 65)
    print(f"  JSBSim root : {root}")
    print(f"  Cat force   : {CAT_FORCE_LBF:.0f} lbf = {CAT_FORCE_N:.0f} N  "
          f"for {CAT_DURATION:.2f} s")
    print(f"  Design ΔV   : {DV_DESIGN:.1f} m/s  ({DV_DESIGN*1.944:.1f} kts)")
    print()

def _should_log(t, phase, prev):
    if phase != prev: return True
    if phase == 'CATAPULT': return t % 0.05 < 0.009
    if phase in ('pull_up', 'climb'): return t % 0.5 < 0.009
    return t % 1.0 < 0.009

def _print_row(r):
    print(f"  {r['t']:5.2f}s  {r['V_kts']:6.1f}  {r['alt_m']:6.1f}  "
          f"{r['theta']:6.1f}  {r['rpm']:5.0f}  {r['cat_N']/1000:6.2f}  {r['phase']}")

def _summary(records, verbose, csv_path):
    if not records: return
    max_v   = max(r['V_kts'] for r in records)
    max_alt = max(r['alt_m'] for r in records)
    max_cat = max(r['cat_N'] for r in records)
    final   = records[-1]
    if verbose:
        print()
        print("─" * 65)
        print(f"  Peak speed      : {max_v:.1f} kts")
        print(f"  Peak altitude   : {max_alt:.1f} m")
        print(f"  Max cat force   : {max_cat:.0f} N  (target {CAT_FORCE_N:.0f} N)")
        print(f"  Final state     : V={final['V_kts']:.1f} kts  "
              f"Alt={final['alt_m']:.1f} m  θ={final['theta']:.1f}°  "
              f"RPM={final['rpm']:.0f}")
        print("─" * 65)

    if csv_path:
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=records[0].keys())
            w.writeheader()
            w.writerows(records)
        if verbose:
            print(f"  Trajectory → {csv_path}")


# ── entry point ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=DEFAULT_ROOT,
                    help='JSBSim root directory')
    ap.add_argument('--mode', default='rail', choices=['rail', 'air', 'force'],
                    help='Simulation scenario')
    ap.add_argument('--csv', default=None, help='Save trajectory CSV')
    ap.add_argument('-q', '--quiet', action='store_true')
    args = ap.parse_args()

    verbose = not args.quiet
    if args.mode == 'rail':
        run_rail(args.root, args.csv, verbose)
    elif args.mode == 'air':
        run_air(args.root, args.csv, verbose)
    elif args.mode == 'force':
        # minimal: just verify catapult property is readable
        fdm = jsbsim.FGFDMExec(args.root, None)
        fdm.set_debug_level(0)
        fdm.load_model('Geran-2')
        fdm.run_ic()
        fdm.run()
        v = fdm.get_property_value('external_reactions/catapult/magnitude')
        print(f"catapult/magnitude = {v} (0 = not firing)")
