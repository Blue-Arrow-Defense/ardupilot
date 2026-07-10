# ArduCopter vs ArduPlane — Technical Comparison

Scope: differences relevant to GCS/companion-computer integration and autonomous
mission flight — flight modes, MAVLink telemetry, the arm/launch/mission-start
command sequence, and the underlying architectural differences that explain
why the two vehicles behave differently. All paths are relative to the repo
root (`ardupilot_BlueArrow/`).

---

## 1. Flight Modes

### 1.1 ArduCopter modes (`ArduCopter/mode.h`, `Mode::Number` enum)

| ID | Mode | Purpose |
|---|---|---|
|0|STABILIZE|Manual angle, manual throttle|
|1|ACRO|Manual body-frame rate, manual throttle|
|2|ALT_HOLD|Manual angle, automatic throttle (altitude hold)|
|3|AUTO|Fully automatic waypoint/mission execution|
|4|GUIDED|Fly to GCS-commanded position/velocity in real time|
|5|LOITER|Automatic 3D position hold, auto throttle|
|6|RTL|Automatic return to launch|
|7|CIRCLE|Automatic circle around a point, auto throttle|
|9|LAND|Automatic landing with horizontal position control|
|11|DRIFT|Semi-autonomous coordinated position/yaw/throttle flight|
|13|SPORT|Manual earth-frame rate control|
|14|FLIP|Automated single-axis flip|
|15|AUTOTUNE|Automatic roll/pitch gain tuning|
|16|POSHOLD|Position hold with pilot override|
|17|BRAKE|Full stop via GPS/inertial nav, no pilot input|
|18|THROW|Hand-launch / throw-to-start|
|19|AVOID_ADSB|Automatic avoidance of ADS-B threats|
|20|GUIDED_NOGPS|Guided mode accepting only attitude/altitude, no position|
|21|SMART_RTL|Return home by retracing the recorded flight path|
|22|FLOWHOLD|Position hold via optical flow (no rangefinder)|
|23|FOLLOW|Follow another vehicle/beacon|
|24|ZIGZAG|Predefined A/B zigzag pattern (e.g. spraying)|
|25|SYSTEMID|Injects signals for system identification|
|26|AUTOROTATE|Autonomous helicopter autorotation after power loss|
|27|AUTO_RTL|Virtual alias: AUTO running a DO_LAND_START sequence|
|28|TURTLE|Post-crash self-righting via motors|

### 1.2 ArduPlane modes (`ArduPlane/mode.h`, `Mode::Number` enum)

| ID | Mode | Purpose |
|---|---|---|
|0|MANUAL|Direct stick pass-through to control surfaces|
|1|CIRCLE|No-GPS-required fallback: loiter in a circle at current altitude|
|2|STABILIZE|Manual roll/pitch with self-leveling, manual throttle|
|3|TRAINING|Manual control, bank/pitch limited to configured envelope|
|4|ACRO|Rate-based control with attitude locking when sticks centered|
|5|FBWA (Fly-By-Wire A)|Stick maps to bounded roll/pitch angles, manual throttle|
|6|FBWB (Fly-By-Wire B)|Like FBWA, pitch stick → climb/sink rate, auto throttle|
|7|CRUISE|Like FBWB plus automatic heading lock|
|8|AUTOTUNE|Automatic gain tuning (runs FBWA underneath)|
|10|AUTO|Fully automatic waypoint/mission execution|
|11|RTL|Automatic return to launch/rally point|
|12|LOITER|Circle/orbit a fixed point, auto throttle & navigation|
|13|TAKEOFF|Automated takeoff to a configured altitude|
|14|AVOID_ADSB|ADS-B avoidance (delegates to Guided)|
|15|GUIDED|Fly to GCS-commanded position/velocity in real time|
|16|INITIALISING|Transient startup state|
|17–23|QSTABILIZE / QHOVER / QLOITER / QLAND / QRTL / QAUTOTUNE / QACRO|VTOL-only (QuadPlane) hover, hold, land, RTL, tuning, acro (`#if HAL_QUADPLANE_ENABLED`)|
|24|THERMAL|Automatic thermal-soaring loiter (ArduSoar)|
|25|LOITER_ALT_QLAND|Fixed-wing loiter that auto-transitions into QLAND on a quadplane|

### 1.3 Same name, different behavior

- **LOITER**: Copter = full 3D GPS position hold (station-keeping). Plane = circles/orbits a point while still flying forward.
- **CIRCLE**: Copter = fully autopilot-controlled circle, cannot arm in it. Plane = lightweight no-GPS-needed holding pattern, not a mission tool.
- **RTL**: Copter climbs to RTL altitude, flies home, lands automatically; cannot arm in RTL. Plane flies an approach/loiter home (or converts to QRTL on a quadplane) and does not necessarily land; also cannot arm in RTL.
- **AUTO**: Both run the loaded mission; Copter additionally has the AUTO_RTL alias (AUTO + DO_LAND_START) which Plane lacks as a distinct mode ID.
- **GUIDED**: Copter requires GPS unless the separate GUIDED_NOGPS mode is used. Plane has one GUIDED mode (no NOGPS variant); on quadplanes it also manages VTOL guided flight.
- **ACRO**: Copter = manual body-rate, manual throttle, freestyle. Plane = rate control that self-levels to locked attitude when sticks are centered, still managing fixed-wing lift/energy.
- **AUTOTUNE**: Copter tunes rate-loop gains directly. Plane's AUTOTUNE runs the FBWA control loop underneath while tuning.

### 1.4 Modes unique to each

- **Copter only**: ALT_HOLD, DRIFT, SPORT, FLIP, POSHOLD, BRAKE, THROW, GUIDED_NOGPS, SMART_RTL, FLOWHOLD, FOLLOW, ZIGZAG, SYSTEMID, AUTOROTATE, TURTLE.
- **Plane only**: TRAINING, FBWA, FBWB, CRUISE, TAKEOFF, INITIALISING, THERMAL, LOITER_ALT_QLAND, plus the entire Q-mode family (VTOL/quadplane only).

### 1.5 Mode-change validation

- **Copter** (`ArduCopter/mode.cpp`): each mode implements `requires_GPS()`, `has_manual_throttle()`, `allows_arming()`. `set_mode()` blocks the switch if the target needs GPS and position isn't OK, if EKF altitude isn't healthy for an auto-throttle mode, if a heli's rotor hasn't finished runup, or if fence-breach recovery is active. Arming legality is per-mode — LAND, RTL, CIRCLE, AUTOTUNE, BRAKE, FLIP, SYSTEMID, AVOID_ADSB, FOLLOW, AUTOROTATE all disallow arming while selected.
- **Plane** (`ArduPlane/system.cpp`): no per-mode GPS gate — fixed-wing modes tolerate degraded GPS (no-GPS CIRCLE fallback exists precisely for this). Checks instead cover quadplane availability for Q-modes, fence-breach recovery, and GCS mode-block. Arming legality comes from each mode's `pre_arm_checks()` (e.g. RTL always disallows arming); a global `ONLY_ARM_IN_QMODE_OR_AUTO` option exists for quadplanes.

---

## 2. Telemetry / MAVLink Streaming

**The core mechanism is shared code, not vehicle-specific.** It lives in
`libraries/GCS_MAVLink/GCS_Common.cpp` (base class `GCS_MAVLINK`) and
`libraries/GCS_MAVLink/GCS.h`: `send_heartbeat()`, `set_message_interval()` /
`handle_command_set_message_interval()`, legacy `REQUEST_DATA_STREAM` handling,
the `try_send_message()` dispatch loop, the mission protocol, parameter
protocol, and MAVLink FTP are **not overridden by either vehicle**. Per-vehicle
code only supplies hook functions (`frame_type()`, `base_mode()`,
`custom_mode()`, `landed_state()`, `vtol_state()`, plus vehicle-only message
IDs and stream tables).

Practical consequence: `MAV_CMD_SET_MESSAGE_INTERVAL`, mission upload/download,
parameter protocol, and FTP behave **identically** on both vehicles — the same
GCS/companion-computer code talks to either without special-casing.

### 2.1 HEARTBEAT / MAV_TYPE

- **Copter** (`GCS_Copter::frame_type()`): returns the frame's actual MAV_TYPE — `MAV_TYPE_QUADROTOR`, `MAV_TYPE_HELICOPTER`, hexa/octo, etc. — driven by frame class.
- **Plane**: always `MAV_TYPE_FIXED_WING`, even on a quadplane. VTOL state is instead conveyed separately via `EXTENDED_SYS_STATE`.

### 2.2 EXTENDED_SYS_STATE (landed/VTOL state)

Send logic is shared; values differ:
- **Copter**: `landed_state()` → ON_GROUND / LANDING / TAKEOFF / IN_AIR from `land_complete` + flight-mode flags. `vtol_state()` hardcoded `MAV_VTOL_STATE_MC`.
- **Plane**: `landed_state()` is a simple IN_AIR/ON_GROUND from `is_flying()`. `vtol_state()` queries the quadplane transition state when `HAL_QUADPLANE_ENABLED`, else UNDEFINED.

### 2.3 Default stream rates (`SRx_*` params)

Same underlying stream-group mechanism, but:
- **Copter defaults every `SRx_*` group to 0 Hz** — nothing streams until a GCS explicitly requests it.
- **Plane defaults RAW_SENS / EXT_STAT / RC_CHAN / RAW_CTRL / POSITION / EXTRA1/2/3 to 1 Hz**, PARAMS to 10 Hz, ADSB to 5 Hz — Plane telemetry is "on" out of the box.
- Plane adds a `STREAM_RAW_CONTROLLER` group (`SR_RAW_CTRL` → `MSG_SERVO_OUT`); the same param is a documented no-op on Copter.
- EXTRA1/EXTRA3 message-set composition differs slightly (Plane bundles RPM/AOA_SSA/LANDING/ESC/EFI/hygrometer into EXTRA1; Copter bundles RPM/ESC/generator/winch/EFI into EXTRA3) — mostly reorganization, not a functional gap.

### 2.4 Vehicle-specific messages / commands

- Both vehicles now send `MSG_WIND`, `MSG_AOA_SSA`, `MSG_LANDING`, `MSG_TERRAIN_*` (Copter's `send_wind()` skips if there's no airspeed vector; Plane's always sends).
- **Plane-only**: `MSG_HYGROMETER`; command handlers `MAV_CMD_DO_VTOL_TRANSITION`, `MAV_CMD_GUIDED_CHANGE_SPEED/ALTITUDE/HEADING`, `MAV_CMD_DO_ENGINE_CONTROL`, `MAV_CMD_DO_AUTOTUNE_ENABLE`, `MAV_CMD_SET_HAGL`, `MAV_CMD_DO_GO_AROUND`.
- **Copter-only**: `MAV_CMD_DO_WINCH`, `MAV_CMD_SOLO_BTN_*`, `MAV_CMD_DO_PARACHUTE` (shared but Copter-heavy), `MAV_CMD_NAV_VTOL_TAKEOFF/LAND`, `MAV_CMD_DO_MOTOR_TEST`, a `handle_flight_termination` override.

**Bottom line for a GCS/companion computer**: the *request/response protocols*
are identical; what differs is *what's on by default* (Plane streams
out-of-the-box, Copter doesn't) and a handful of vehicle-specific commands and
landed/VTOL-state semantics.

---

## 3. Arming & Starting Autonomous (Mission) Flight

This is the area with the biggest practical difference, especially for a
catapult/hand-launched fixed-wing airframe.

### 3.1 Copter: command-driven takeoff

1. `MAV_CMD_COMPONENT_ARM_DISARM` → `AP_Arming_Copter::arm_checks()`.
2. Set mode to `GUIDED` or `AUTO`.
3. Takeoff is **explicitly commanded**:
   - GUIDED: `MAV_CMD_NAV_TAKEOFF` → `handle_MAV_CMD_NAV_TAKEOFF` (`ArduCopter/GCS_Mavlink.cpp`) → `do_user_takeoff()` → motors spool immediately (requires armed + landed + interlock).
   - AUTO: the mission's first item is a `MAV_CMD_NAV_TAKEOFF` item, executed by `ModeAuto::takeoff_start()`.
4. `MAV_CMD_MISSION_START` (`handle_MAV_CMD_MISSION_START`) sets mode AUTO, sets `auto_armed(true)`, and **explicitly calls `mission.start_or_resume()`** — this is what kicks the first mission item (often the takeoff) into motion.

Copter's motor start is a **direct software command** — there is no external kinetic event to detect; the mission item itself spins the motors.

### 3.2 Plane: no takeoff mode — launch is detected, not commanded

Plane has **no fixed-wing takeoff mode**. `MAV_CMD_NAV_TAKEOFF`'s handler
(`handle_command_MAV_CMD_NAV_TAKEOFF`) is compiled only `#if
HAL_QUADPLANE_ENABLED` and calls `quadplane.do_user_takeoff()` — **VTOL-only**.
A pure fixed-wing build (e.g. Geran-2) does not have this handler at all.

Sequence for a catapult/hand-launch mission:

1. **Arm** — via `MAV_CMD_COMPONENT_ARM_DISARM`, or (common for catapult/hand-launch) **rudder arming** (`AP_Arming::Method::RUDDER`), which additionally requires zero throttle stick at arm time (`ArduPlane/AP_Arming.cpp`).
2. `MAV_CMD_MISSION_START` (`ArduPlane/GCS_Mavlink.cpp`) just calls `set_mode(mode_auto, ...)` — it does **not** call `mission.start_or_resume()`; entering AUTO itself resumes the mission from the current command.
3. The mission's first item is a `MAV_CMD_NAV_TAKEOFF` **mission item** processed by `Plane::start_command()` → `do_takeoff()` (`ArduPlane/commands_logic.cpp`), which only records target pitch/altitude — **it does not spin the motor**.
4. Throttle stays **suppressed** (`Plane::suppress_throttle()`) until `auto_takeoff_check()` (`ArduPlane/takeoff.cpp`) returns true. The aircraft is armed and sitting in AUTO with the prop off — this is the actual catapult trigger:
   - Armed, safety off, GPS 3D fix; if rudder-armed, rudder must be back at neutral.
   - Longitudinal acceleration (`TECS_controller.get_VXdot()`) must exceed **`TKOFF_THR_MINACC`** — this is the literal "catapult kick" / hand-throw detector.
   - A timer (**`TKOFF_THR_DELAY`**, tenths of a second) must elapse after the accel event — protects hand-launchers from the prop.
   - Attitude check (pitch/roll within limits) unless disabled via `FlightOptions::DISABLE_TOFF_ATTITUDE_CHK`.
   - GPS ground speed must exceed **`TKOFF_THR_MINSPD`** (0 disables) within a ~2.5 s window, or the detector resets.
5. Once satisfied, throttle un-suppresses, the motor starts, and `verify_takeoff()` monitors climb-out/level-off before advancing to the next mission item.

**In short: on Copter, a MAVLink command starts the motor. On Plane, MAVLink
commands only arm the state machine — the physical launch (accelerometer +
speed thresholds) is what actually starts the motor.** This matches what was
found building the Geran-2 catapult SITL (see project memory
`geran2-catapult-sitl` / `geran2-validation-harness`): getting the launch
sequence right was about satisfying `TKOFF_THR_MINACC`/`TKOFF_THR_MINSPD`
conditions and giving the EKF a valid attitude reference pre-launch, not about
sending a takeoff command.

### 3.3 Key MAV_CMD differences

| Command | Copter | Plane |
|---|---|---|
| `MAV_CMD_NAV_TAKEOFF` | Always available; real vertical takeoff (Guided `do_user_takeoff` or Auto `takeoff_start`) | Compiled only for QuadPlane VTOL takeoff; fixed-wing takeoff is a plain mission item, not this handler |
| `MAV_CMD_MISSION_START` | Sets AUTO + `auto_armed(true)` + explicit `mission.start_or_resume()` | Only sets mode AUTO; mission resumes because entering the mode runs it |
| `MAV_CMD_DO_MOTOR_TEST` | Available on any frame | Only under `HAL_QUADPLANE_ENABLED` |
| `MAV_CMD_NAV_VTOL_TAKEOFF` | Aliased to `NAV_TAKEOFF` | Separate handler, quadplane-only |
| `MAV_CMD_DO_VTOL_TRANSITION`, `MAV_CMD_DO_ENGINE_CONTROL`, `MAV_CMD_SET_HAGL`, `MAV_CMD_DO_GO_AROUND` | Not present | Plane-only |
| `MAV_CMD_DO_WINCH`, `MAV_CMD_SOLO_BTN_*` | Copter-only | Not present |

### 3.4 Arming-check differences relevant to launch

- **GPS**: Copter only requires GPS if the current mode's `requires_GPS()` is true (e.g. not required in Stabilize). Plane does not override the base `gps_checks()` — a 3D fix is required to arm in essentially any mode when `ARMING_CHECK_GPS` is set.
- **Throttle position**: Plane's rudder-arm path explicitly rejects non-zero throttle at arm time; Copter has no equivalent gate.
- **RC presence**: Plane's `rc_received_if_enabled_check()` requires RC receipt before arming if throttle failsafe is enabled; no direct Copter analog.
- **Mission shape**: Plane's `ModeAuto::_pre_arm_checks()` only requires a takeoff item if QuadPlane is enabled — a pure fixed-wing mission can arm in AUTO with no takeoff item at all. Copter has no equivalent mission-shape check.

### 3.5 Launch-relevant parameters (Plane-only)

`TKOFF_THR_MINACC`, `TKOFF_THR_DELAY`, `TKOFF_THR_MINSPD`, `TKOFF_THR_MIN`,
`g2.takeoff_throttle_accel_count` — none of these have a Copter equivalent,
since Copter takeoff altitude/behavior is just the `z`/`param7` value on the
takeoff command, consumed directly.

---

## 4. Other Architectural Differences

### 4.1 Failsafe behavior

- **Copter** (`ArduCopter/defines.h`, `events.cpp`): a single `FailsafeAction` enum (`NONE, LAND, RTL, SMARTRTL, SMARTRTL_LAND, TERMINATE, AUTO_DO_LAND_START, BRAKE_LAND`) dispatched via `do_failsafe_action()` with a priority table so the most severe wins when multiple trip at once. EKF failsafe forces a hard descent (Copter can't fly open-loop without a position/attitude solution).
- **Plane** (`ArduPlane/defines.h`, `failsafe.cpp`): split into *short* failsafe (`FS_ACTION_SHORT_CIRCLE/FBWA/FBWB/DISABLED` — just circle or drop to a stabilized manual mode, hoping for signal recovery) and *long* failsafe (`FS_ACTION_LONG_CONTINUE/RTL/GLIDE/PARACHUTE/AUTO`). There's also a separate main-loop-lockup failsafe that passes RC straight through to servos — no Copter equivalent.
- **Net**: Copter failsafes converge on **land now** (powered descent); Plane failsafes converge on **keep flying/gliding** (circle, RTL, glide) since a plane can't simply stop and land where it is.

### 4.2 Geofence

Both use `AC_Fence`, but breach actions differ:
- **Copter**: `RTL_AND_LAND`, `ALWAYS_LAND`, `SMART_RTL`, `SMART_RTL_OR_LAND`, `BRAKE`, plus immediate disarm if grounded and a forced LAND if breach distance exceeds a give-up threshold.
- **Plane**: `REPORT_ONLY`, `GUIDED`, `GUIDED_THROTTLE_PASS`, `RTL_AND_LAND` — sends the aircraft to a computed return point (fence return point, rally point, or home) via guided waypoint, or RTL/auto-land if already landing. Plane also suppresses fence checks during final landing and has fence-recovery/stick-mixing suppression logic Copter lacks.

### 4.3 Control architecture

- **Copter**: `AC_AttitudeControl` (+Heli/6DoF variants) + `AC_PosControl` + `AC_WPNav`/`AC_Loiter`/`AC_Circle` — cascaded rate/attitude/position PID loops for direct thrust-vector control.
- **Plane**: `AP_TECS` (Total Energy Control System — trades throttle/pitch to manage height+speed energy) + `AP_L1_Control` (lateral path tracking) as the `nav_controller`. Confirmed by `ArduPlane/wscript` linking `AP_TECS`/`AP_L1_Control`/`APM_Control`, absent from `ArduCopter/wscript`.
- Plane's wscript *also* links `AC_AttitudeControl`/`AP_Motors`/`AC_WPNav` — only needed for QuadPlane VTOL support, i.e. Plane reuses Copter's control stack for its VTOL subset rather than reimplementing it.

### 4.4 Sensors

`AP_Airspeed` is compiled into both, but only **Plane** actually uses it —
feeding TECS/L1 speed control directly. No Copter source file references
`AP_Airspeed`. Correspondingly Plane has an entire `TKOFF_*`/`ARSPD_*`
parameter family with no Copter equivalent, while Copter has vertical/loiter
speed params (`PILOT_SPEED_UP/DN`, `LOIT_SPEED`) with no Plane equivalent.

### 4.5 Vehicle-specific subsystems

- **Plane: QuadPlane** (`ArduPlane/quadplane.cpp`) embeds Copter-like VTOL capability into the fixed-wing airframe — its own `AC_AttitudeControl_TS` (tailsitter variant), `AP_MotorsMulticopter`, and Tailsitter/Tiltrotor classes for hover, transition, and QRTL/QLAND.
- **Copter: Heli support** — `HELI_FRAME` gates `AC_AttitudeControl_Heli` and `AC_Autorotation`, producing a separate `arducopter-heli` build target; Plane has no analogous variant.

### 4.6 Landing/takeoff logic

- Copter's `ModeLand` is a simple position/velocity-controlled descent at a fixed rate.
- Plane uses the dedicated `AP_Landing` library — glide-slope computation, rangefinder-based slope recalculation/auto-abort, and a flare stage (`LAND_FLARE_ALT`/`LAND_FLARE_SEC`). Plane's RTL also branches into QRTL/QLAND on quadplanes. Plane's `ModeTakeoff` (VTOL) tracks flight-stage transitions and failsafe suppression during the takeoff roll — no Copter analog needed given Copter's simpler vertical takeoff.

### 4.7 Other

Plane's raw-RC-passthrough failsafe (main loop lockup) has no Copter
equivalent — a stalled loop on a multirotor can't be safely handled by
passthrough the way it can on a plane with mechanically-linked-feeling control
surfaces. Both vehicles do share `AC_WPNav`, `AC_AttitudeControl`,
`AP_InertialNav`, and `AC_PrecLand`, reflecting QuadPlane's reuse of Copter's
core control stack rather than a separate implementation.

---

## 5. Summary Table

| Aspect | ArduCopter | ArduPlane |
|---|---|---|
| Telemetry/MAVLink core protocol | Shared `GCS_MAVLINK` code | Same shared code |
| Default stream rates | 0 Hz (silent until requested) | 1–10 Hz (streams by default) |
| HEARTBEAT MAV_TYPE | Frame-dependent (quad/hexa/heli/…) | Always `MAV_TYPE_FIXED_WING` |
| "Start autonomous flight" trigger | MAVLink command spins the motor directly | MAVLink only arms a state machine; physical launch (accel + speed threshold) starts the motor |
| Dedicated takeoff mode | Yes (Guided takeoff / mission NAV_TAKEOFF) | No (fixed-wing); VTOL-only via QuadPlane |
| Position/attitude control | Cascaded PID (`AC_AttitudeControl`/`AC_PosControl`) | Energy-based (`AP_TECS`) + path-tracking (`AP_L1_Control`) |
| Airspeed sensor | Unused | Core to speed/height control |
| Failsafe default behavior | Land / RTL now | Circle / glide / RTL — keep flying |
| VTOL support | N/A (native) | Optional QuadPlane subsystem, reuses Copter's control stack |
