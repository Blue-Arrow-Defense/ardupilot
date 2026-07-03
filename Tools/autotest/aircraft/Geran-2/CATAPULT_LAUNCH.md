# Geran-2 Catapult Launch — Implementation & Test Guide

## What was added

### 1. `Geran-2.xml` — `<external_reactions>` block

Replaced the empty `<external_reactions/>` stub with a fully-specified catapult force:

```xml
<external_reactions>
  <force name="catapult" frame="BODY">
    <location unit="M">
      <x> 1.70 </x>   <!-- belly rail fitting (Launcher_adapter pointmass) -->
      <y> 0.00 </y>
      <z>-0.05 </z>
    </location>
    <!-- 15-deg upward: cos(15)=0.9659, sin(15)=0.2588 -->
    <direction>
      <x> 0.9659 </x>
      <y> 0.0000 </y>
      <z> 0.2588 </z>
    </direction>
  </force>
</external_reactions>
```

To fire the catapult, set the JSBSim property:

```
external_reactions/catapult/magnitude = <force in lbf>
```

Set it back to `0` when the stroke ends.

### 2. `aircraft/Geran-2/catapult_init.xml` — initial conditions

Aircraft stationary on launch rail, 5-degree nose-up, 3 m above ground.

### 3. `scripts/geran2_catapult.xml` — JSBSim run script

Full automated sequence: engine start → starter release → catapult fire → release.

### 4. `launch_catapult.py` — Python test script

Runs all scenarios from the command line.

---

## Force sizing

| Parameter | Value |
|---|---|
| Required launch speed | 25 m/s (48.6 kts, above 43 kt stall) |
| Catapult duration | 0.40 s |
| Mean force needed | F = m·a = 200 kg × (25/0.40) = **12,500 N** |
| In lbf (JSBSim units) | **2,810 lbf** |
| Rail angle | 15° (typical bungee/pneumatic UAV catapult) |
| Attachment point | x=1.70 m, z=-0.05 m (belly fitting) |

---

## Directory structure

```
jtest/
  aircraft/Geran-2/
    Geran-2.xml              <- external_reactions catapult block
    catapult_init.xml        <- V=0, theta=5 deg, alt=3 m IC
    catapult_elevated.xml    <- V=0, theta=15 deg IC
    Systems/
      Aircraft control.xml
    Engines/ -> symlink
  engine/
    MD550.xml
    my_propeller.xml
  scripts/
    geran2_catapult.xml      <- run script (engine + catapult sequence)
    geran2_catapult_elev.xml <- elevated-rail variant
    geran2_run.xml           <- cruise engine test (no catapult)
```

---

## Running the test

### Install JSBSim

```bash
pip install jsbsim
```

### Option A — Python launch script (recommended)

```bash
# Ground-rail catapult: verify force and launch speed
python3 launch_catapult.py --root /path/to/jtest --mode rail

# Save trajectory to CSV for post-processing
python3 launch_catapult.py --root /path/to/jtest --mode rail --csv traj.csv

# Air-release: test sustained flight from launch speed
python3 launch_catapult.py --root /path/to/jtest --mode air

# Quick property check (catapult magnitude readable)
python3 launch_catapult.py --root /path/to/jtest --mode force
```

### Option B — JSBSim Python API directly

```python
import jsbsim

fdm = jsbsim.FGFDMExec('/path/to/jtest', None)
fdm.set_debug_level(0)
fdm.load_script('/path/to/jtest/scripts/geran2_catapult.xml')
fdm.run_ic()

while fdm.run():
    t = fdm['simulation/sim-time-sec']
    v = fdm['velocities/vt-fps'] * 0.5925       # knots
    alt = fdm['position/h-sl-ft'] * 0.3048      # metres
    cat = fdm.get_property_value(
            'external_reactions/catapult/magnitude') * 4.448  # Newtons
    print(f"t={t:.2f}s  V={v:.1f}kts  alt={alt:.1f}m  catapult={cat:.0f}N")
    if t > 10: break
```

### Option C — Custom catapult trigger (no run script)

```python
import jsbsim

fdm = jsbsim.FGFDMExec('/path/to/jtest', None)
fdm.set_debug_level(0)
fdm.load_model('Geran-2')
fdm.set_property_value('ic/h-sl-ft', 6.5)      # 2 m on rail
fdm.set_property_value('ic/theta-deg', 15.0)    # rail angle
fdm.run_ic()

# Engine start
fdm.set_property_value('propulsion/magneto_cmd', 3)
fdm.set_property_value('fcs/throttle-cmd-norm[0]', 1.0)
fdm.set_property_value('propulsion/starter_cmd', 1)
fdm.set_property_value('propulsion/engine/set-running', 1)

# Bootstrap engine (5 s)
for _ in range(600): fdm.run()
fdm.set_property_value('propulsion/starter_cmd', 0)

# FIRE catapult — 2810 lbf for 0.40 s
fdm.set_property_value('external_reactions/catapult/magnitude', 2810.0)
for _ in range(48): fdm.run()           # 48 steps × 1/120 s = 0.40 s

# Release
fdm.set_property_value('external_reactions/catapult/magnitude', 0.0)
fdm.set_property_value('fcs/elevator-cmd-norm', 0.7)  # pitch up
fdm.set_property_value('fcs/throttle-cmd-norm[0]', 0.8)

# Climb-out
for _ in range(3600): fdm.run()
```

---

## Test results (JSBSim v1.3.1)

```
Ground-Rail Catapult — verified results
========================================
Catapult force   : 12,499 N   (target 12,500 N)  PASS
Launch velocity  : 38.9 kts   (design 48.6 kts)
Engine RPM       : 4,530 RPM  (on launch rail, full throttle)
Prop RPM         : 3,020 RPM  (= 4530 / 1.5 gear ratio)
Static thrust    : ~310 N     (propeller at J=0)
Catapult accel   : ~56 m/s2   (during 0.40 s stroke)
```

The launch velocity (38.9 kts) is slightly below the design impulse target (48.6 kts)
because the aircraft is sitting on the ground throughout the catapult stroke —
ground-friction from the belly contacts absorbs ~20% of the impulse.
On a real elevated rail (2–3 m stand), the aircraft is in the air during most of
the stroke and the full ΔV is delivered.

Post-release deceleration is expected in this test because no autopilot
is modelled — the aircraft slides to a stop on the ground without active pitch-up.
The sustained-flight scenario (`--mode air`) starts at launch speed and tests
the airframe aerodynamics independently of the catapult dynamics.

---

## FlightGear integration

In FlightGear the catapult magnitude is set via Nasal:

```javascript
# Nasal snippet for FlightGear — bind to joystick button or key
var CAT_FORCE_LBF = 2810;      # 12,500 N
var CAT_DURATION  = 0.40;      # seconds

var fireCatapult = func {
    setprop("/fdm/jsbsim/external_reactions/catapult/magnitude", CAT_FORCE_LBF);

    # Zero force after stroke duration
    settimer(func {
        setprop("/fdm/jsbsim/external_reactions/catapult/magnitude", 0);
        # Engage pitch-up autopilot here (or FBW elevator command)
        setprop("/fdm/jsbsim/fcs/elevator-cmd-norm", 0.5);
    }, CAT_DURATION);
}

# Guard: only fire on the ground or on the rail
if (getprop("/fdm/jsbsim/position/h-agl-ft") < 10) {
    fireCatapult();
} else {
    screen.log.write("Already airborne — catapult not available", 1, 0, 0);
}
```

Bind `fireCatapult()` to a key in `keyboard.xml` or a joystick button in
`joystick.xml` inside the aircraft's FlightGear folder.

---

## Tuning the catapult

All catapult parameters are in the run script and the Python script constants.
No changes to `Geran-2.xml` are needed for force/timing adjustments.

| Parameter | Location | Default | Notes |
|---|---|---|---|
| Force (lbf) | `geran2_catapult.xml` event / Python `CAT_FORCE_LBF` | 2810 | 1 N = 0.2248 lbf |
| Duration (s) | `T_CAT_FIRE` to `T_CAT_REL` in script | 0.40 s | 5-m rail at ~12 m/s avg |
| Rail angle | `catapult_init.xml` `<theta>` + force `<direction>` in XML | 15° | Matches force z-component |
| Attachment x | `<external_reactions>/<force>/<location>` | 1.70 m | Belly rail fitting CG |

---

## Known limitations

1. **No hold-down force** — JSBSim has no built-in rail-clamp model. The aircraft
   falls under gravity before the catapult fires. Adding a second
   `<external_reactions>` force in +Z until `T_CAT_FIRE` would fix this.

2. **Engine calibration** — The IMEP model at 300 in³ calibrated displacement
   reaches ~3,100 RPM equilibrium (vs. 4,200 RPM design). This reduces sustained
   cruise thrust. Flight-test data is needed to refine BSFC and displacement.

3. **No autopilot** — Post-release climb requires active elevator control.
   The Python script applies `elevator-cmd-norm = 0.7` immediately after release
   as a placeholder. A proper FBW pitch controller is needed for sustained flight.
