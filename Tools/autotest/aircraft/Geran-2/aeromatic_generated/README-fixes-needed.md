# Aeromatic++ for a Geran-2-class flying wing — what it gives you, and what you must fix

**Short answer:** yes, aeromatic++ gets you a complete, self-consistent JSBSim
skeleton (geometry, mass/inertia estimate, full aero tables, a **piston** engine,
propeller, control system, JSON) from ~30 answers — but it has **no flying-wing
model**. It always builds a conventional tail + elevator/aileron/**rudder**
layout and a tractor-prop aero model, and it mis-handles a very low aspect ratio.
So it's a good *starting point* for a Cessna-like layout, but for a tailless
delta like the Geran-2 you overwrite roughly half of it.

This folder contains exactly what aeromatic produced from the inputs in
`aeromatic_answers_used.txt` (metric, Light-GA, Delta wing, aft-fuselage piston):
- `Geran2aero.xml`, `Engines/MD550.xml` (piston), `Engines/my_propeller.xml`,
  `Systems/Conventional Controls.xml`, `Geran2aero.json`

How it was run:
```
./aeromatic++/build/aeromatic < aeromatic_answers_used.txt
# (answers piped on stdin, one per line, blank = accept default)
```

Compare any value below against the working `../Geran-2.xml`.

---

## What aeromatic got right / usable as-is
- **Overall file structure** and all the JSBSim boilerplate.
- **Metric units** throughout (choose measurement system = Metric).
- **Reference points**: AERORP (neutral point) x=1.68 ≈ correct 1.698.
- **A piston engine** (`Engines/MD550.xml`, FGPiston) with maxhp ≈ 49.6 hp from
  the 37 kW input — the requested piston type.
- Reasonable **propeller stub** (1.0 m, 2-blade, fixed-pitch).
- Sensible **damping-derivative signs** (Cmq, Clp, Cnr all negative).

---

## What you MUST change (by category)

### 1. Wing geometry — the biggest error (it cascades into aero + inertia)
| Field | Aeromatic gave | Correct (Geran-2) | Why |
|---|---|---|---|
| Aspect ratio | **5.50** | 1.33 | It clamps AR to a type-typical value and ignores the low AR implied by span²/area (2.5²/4.69). |
| `chord` (MAC) | **0.45 m** | **2.10 m** | Derived from the wrong AR → every chord-scaled pitch term is wrong. |
| t/c | 1.22 % | ~8–10 % | Consequence of the 0.45 m chord. |
| `htailarea` / `htailarm` | **0.75 / 1.82** | **0 / 0** | Phantom horizontal tail — aeromatic adds one even when you enter 0. A flying wing has none. |
| `EYEPOINT` | (0.45, −0.46, 1.14) | (1.4326, 0, 0) | Put at the IMU/CG — SITL reads the accelerometer here. |
| CG x | 1.61 | 1.433 | See mass below. |

### 2. Mass & inertia — replace the estimate with real numbers
| Field | Aeromatic | Correct | Note |
|---|---|---|---|
| `ixx` | 10.25 | 41.7 | ~4× low |
| `iyy` | 35.70 | 153.6 | ~4× low — pitch inertia dominated by nose-warhead ↔ tail-engine separation, which a single lumped mass can't capture |
| `izz` | 35.70 | 205.0 | ~6× low |
| point masses | one auto 40.2 kg "Payload" at CG | **6 discrete masses** (warhead 50@0.35, engine 18@3.25, prop 4@3.40, avionics 5@0.75, battery 3@0.90, launcher 5@1.70) | Needed for correct CG **and** inertia |
| fuel tanks | **two** tanks, 14.8 kg each (29.6 kg) | **one** 63 kg tank @ x=1.90 | aeromatic splits fuel across "wings" by default |

### 3. Control system — swap conventional → elevon (flying wing)
`Systems/Conventional Controls.xml` has **Pitch (elevator)**, **Roll (left+right
aileron)** and **Yaw (rudder)** channels. A tailless wing has **no rudder** and
uses **elevons**:
- Replace with a mixer: symmetric elevon → `fcs/elevator-pos-rad` (pitch),
  differential elevon → `fcs/aileron-pos-rad` (roll). See `../Systems/Aircraft control.xml`.
- **Check the pitch sign** — an aft elevon needs cmd(+1 nose-up) → TE-up
  deflection, which is the *inverted* range aeromatic writes.

### 4. Aerodynamics — delete wrong terms, retune the rest
| Item | Aeromatic | Fix |
|---|---|---|
| Reported **CL-alpha** | **38.79 /rad** (impossible; max ≈ 2π) | Use Helmbold low-AR value **CLα ≈ 1.552 /rad**; the header even flags this class of bug ("CLa=54.38"). |
| `Lift_alpha` table | has a discontinuity (α 0.08→0.94 then 0.09→0.39) | Replace with a clean monotonic table incl. vortex-lift region. |
| **Propwash** terms (`Lift_propwash`, `Pitch_propwash`, `Roll_differential_propwash`) | present (assume tractor prop over wing) | **Delete** — Geran-2 is a **pusher**; no propwash on the wing. |
| **Rudder** terms (`Side_rudder`, `Roll_rudder`, `Yaw_rudder`) | reference `fcs/rudder-pos-rad` | **Delete** — no rudder. |
| `Pitch_alpha` Cmα | −0.50 | −0.195 (set for your static margin) |
| `Pitch_elevator` Cmδe | −1.10 | −0.35 |
| `Pitch_damp` Cmq | −12.0 | −3.5 |
| `Drag_minimum` CD0 | 0.0355 | 0.020 |
| `Drag_induced` k | 0.171 | 0.341 (=1/(π·e·AR) with the **real** AR=1.33) |
| Re-number tables (Roll_beta, Yaw_alpha "stall initiator", etc.) | present | optional — the working model uses simple constant derivatives instead |

### 5. Piston engine — usable, but tune before trusting it
`Engines/MD550.xml`: maxhp 49.6 (good), but `cycles=4` (the real MD-550 is a
**2-stroke**), 1 cylinder, displacement 94.3 in³, maxrpm 2800.
> ⚠️ This is the same FGPiston family that (see `../Engines/MD550.xml` and
> `../CATAPULT_LAUNCH.md`) **never reached self-sustaining RPM** in SITL — it
> needs `idlerpm`/`bsfc`/mixture calibration against real data, or you swap it
> for `<electric_engine>` (what the working model does). Set `cycles=2` and the
> real displacement first if you keep the piston.

Also set the real **engine location** (x≈3.20) and **prop location** (x≈3.50);
aeromatic only placed the thruster at x=1.98 for the "aft fuselage" layout.

### 6. Missing entirely — you must ADD these
| Missing | Needed for | Source |
|---|---|---|
| `<ground_reactions>` | Aeromatic wrote **none** (you answered "no landing gear"), so the aircraft falls through the ground. Add STRUCTURE belly + wingtip contacts. | copy from `../Geran-2.xml` |
| `<external_reactions>` catapult force | block is **empty** — no launch force. Add the catapult force (location/direction/magnitude). | `../Geran-2.xml` + `SIM_JSBSim.cpp` sets magnitude |
| `reset.xml` / `<initialize>` | initial lat/lon/alt/attitude | SITL generates it (see `SIM_JSBSim.cpp`) |
| ArduPilot `.parm` envelope | AIRSPEED_MIN/CRUISE/MAX, ROLL_LIMIT_DEG, TKOFF_*, TECS — a correct FDM still won't fly the mission without these | `../Geran-2-catapult.parm` |

---

## Bottom line
Aeromatic++ is worth running to **seed** the file (units, structure, a piston
engine, a propeller, a first inertia guess, aero table scaffolding). But for a
Geran-2-class tailless pusher delta you then:
1. fix wing chord/AR (and delete the phantom htail),
2. replace inertias + point-mass budget + fuel tank,
3. swap conventional controls → elevon mixer (drop the rudder),
4. delete propwash + rudder aero, retune CLα/Cmα/Cmδe/Cmq/CD0/k,
5. fix/replace the piston (cycles, displacement — or go electric),
6. add ground contacts, the catapult force, and the ArduPilot `.parm` envelope.

The `../JSBSim-model-parameter-template.md` table lists every one of these with
units and the target Geran-2 values.
