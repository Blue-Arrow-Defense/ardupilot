# Delta-38 — how this model was built

Source of every input number: **`uav jsbsim questionnaire v3_13-08-26.docx`**
(«ОПИТУВАЛЬНИК ДЛЯ НАЛАШТУВАННЯ ЛЬОТНОЇ МОДЕЛІ БПЛА», v3, 13-08-2026).
Items are cited below as `[Q x.y]`, matching the questionnaire's numbering.

Nothing in the FDM is a placeholder. Each value is one of:

| tag | meaning |
|---|---|
| `[Q x.y]` | used exactly as the manufacturer answered |
| `[DER]` | derived from `[Q]` values by a stated calculation |
| `[EST]` | engineering estimate where the questionnaire has no answer — every one of these is listed in [DATA-GAPS.md](DATA-GAPS.md) |
| *measured* | read out of a JSBSim run of this model (see [Verification](#verification)) |

---

## 1. What the questionnaire says the aircraft is

| | |
|---|---|
| Configuration | tailless delta, elevons only, no h-tail, wingtip winglets `[Q 2.5–2.7]` |
| Powerplant | 70 cm³ 2-cyl 2-stroke gasoline, 5.22 kW @ 6300 rpm, **pusher**, direct drive to a 20″ 2-blade fixed-pitch prop `[Q 4.5, 5.1–5.6, 6.1–6.4]` |
| Launch / recovery | catapult, no landing gear, belly recovery `[Q 7.2]`, cover page |
| Mass | 38 kg MTOW, 15.5 kg empty, 14.9 kg payload, 5.5 L fuel `[Q 3.2, 3.3, 3.10, 4.13]` |
| Wing | S = 1.75 m², b = 1.68 m, MAC = 1.243 m, +3° incidence `[Q 2.1–2.4]` |
| Stall | VS1 = 88 km/h at MTOW `[Q 3.1]` |
| Controls | "conventional" `[Q 7.1]`, no flaps `[Q 7.3]` — i.e. elevons |

---

## 2. Planform `[DER]`

`AR = b²/S = 1.68²/1.75 = **1.613**` — a genuine slender delta, which is what
drives most of the aerodynamics.

`c_root + c_tip = 2S/b = 2.083 m`. `[Q 2.9]` gives 1.79 m from the nose to the
wing trailing edge, and the aircraft is essentially all wing, so:

* root chord **1.79 m** with the apex at the nose datum,
* tip chord **0.293 m**, taper ratio **0.164**,
* straight unswept trailing edge at x = 1.79 → **LE sweep 60.7°**, c/4 sweep 53.2°,
* MAC = 1.221 m (the manufacturer's 1.243 is 1.8% higher — within hand-measurement
  spread of a delta; `<chord>` keeps **their** value so the reference length
  matches their data sheet, while the derivations below use the geometric one),
* x_LEMAC = 0.569 m, y_MAC = 0.319 m.

Solving instead for the taper that reproduces MAC = 1.243 exactly gives
λ = 0.135, c_root = 1.835 m — i.e. a root chord slightly *longer* than the
nose-to-TE distance. Both readings put the delta apex at the nose; the 1.79 m
version is the one consistent with `[Q 2.9]`.

## 3. Mass and balance

`<emptywt>` must exclude anything declared as a pointmass, so the 2.2 kg engine
`[Q 3.9]` is broken out of the 15.5 kg empty weight `[Q 3.3]` and the structural
CG is back-solved from the measured empty CG of 1.370 m `[Q 3.4]`:

```
x_struct = (15.5·1.370 − 2.2·1.790) / 13.3 = 1.3005 m
```

| item | kg | x (m) | source |
|---|---|---|---|
| structure + avionics | 13.30 | 1.3005 | `[DER]` from `[Q 3.3, 3.4, 3.9]` |
| engine | 2.20 | 1.790 | `[Q 3.9]` |
| payload | 14.90 | 0.600 | `[Q 3.10]` |
| fuel (5.5 L MOGAS, 0.72 kg/L) | 3.96 | 0.976 | `[Q 4.12, 4.13]` |
| **total** | **34.36** | **0.991** | |

Two things worth flagging to the customer:

* **The tank sits on the CG.** Burning all 5.5 L moves the CG by **+1.9 mm**.
  That is a genuinely good design point and it means fuel state has no effect
  on trim.
* **MTOW is 38 kg but a typical mission is 34.36 kg**, so there is 3.6 kg of
  payload growth in hand. Wing loading: 19.6 kg/m² typical, 21.7 at MTOW —
  less than half the Geran-2's 42 kg/m², which is why this airframe cruises
  much slower.

**Inertia** `[DER]` — `[Q 3.5–3.8]` were all answered `0` ("estimate it").
JSBSim's `<ixx>/<iyy>/<izz>` are the **base** inertia of the `<emptywt>`
structure about the structural CG; JSBSim adds every pointmass's and tank's
parallel-axis term itself. Entering the total would double-count. So:

```
structure as a delta plate:  Ixx = m·b²/24 = 1.56   Iyy = m·c_r²/18 = 2.37   Izz = Ixx+Iyy = 3.93
payload/fuel/engine bodies:      +0.26              +0.42                       +0.42
base (what goes in the XML):      1.82               2.79                        4.35
JSBSim adds transfer terms:      +0.03              +4.98                       +4.96
total (check in JSBSim's report):  1.85               7.77                        9.31   kg·m²
```

Radii of gyration `k_x/b = 0.14`, `k_y/L = 0.27` — both normal for a delta.
`Ixz` is kept at 0 per `[Q 3.8]`; the point-mass sum gives −0.14 kg·m².

## 4. Aerodynamics

| coefficient | value | basis |
|---|---|---|
| CL<sub>α</sub> | 1.731 /rad | Helmbold (1942) low-AR swept formula, `2πAR/(2+√(4+AR²(1+tan²Λ_LE)))` |
| CL<sub>0</sub> | 0.091 | = CL<sub>α</sub>·3° — the `[Q 2.4]` wing setting angle, folded into CL0 (JSBSim's `<wing_incidence>` is informational; the aero tables see body α) |
| CL<sub>max</sub> | 0.582 at α = 16.3° | **back-solved from `[Q 3.1]` VS1 = 88 km/h at `[Q 3.2]` 38 kg** |
| CD<sub>0</sub> | 0.032 `[EST]` | Raymer 12.3 build-up: 0.0097 clean (Cf 0.0037 at Re 2.8e6, form 1.25, Swet/S 2.1) + exposed cylinder head/cooling, engine fairing, pusher pylon, winglets, launch rail fitting |
| k | 0.282 | `1/(π·e·AR)` with e = 0.70 |
| C<sub>mα</sub> | −0.156 /rad | CL<sub>α</sub>·(x_cg − x_np)/MAC, static margin **9.0 % MAC** — see the warning below |
| C<sub>m0</sub> | +0.016 | sized so hands-off trim lands at 34 m/s (α_trim = Cm0/−Cmα = 5.5°) |
| C<sub>mδe</sub> | −0.309 /rad `[EST]` | −CL<sub>δe</sub>·arm/MAC with CL<sub>δe</sub> = 0.55 and the elevon ac at x = 1.676 |
| C<sub>mq</sub> | −3.00 /rad | DATCOM 5.2.1.2, chosen at the strong end of the −2.2…−3.5 band *because* the static margin is thin |
| C<sub>lp</sub> | −0.30 /rad | strip integration gives −0.19 (3D slope) to −0.43 (2D slope) |
| C<sub>lδa</sub> | 0.15 /rad `[EST]` | strip integration over the assumed elevon span gives 0.196, reduced for low-AR tip losses |
| C<sub>nβ</sub> | +0.045 /rad | winglets `[Q 2.7]`: 2.5·(0.150/1.75)·(0.579/1.68)·0.8 = +0.059, less ~0.014 for the body |
| C<sub>nr</sub> | −0.032 /rad | = −2·C<sub>nβ</sub>·(arm/b) |

### ⚠ The neutral point is the model's weak spot

A slender delta's aerodynamic centre is **not** at 25 % MAC. Published
estimates for AR = 1.61 / Λ_LE = 60.7° span `x_ac = 0.566…0.667·c_root`:

| x_ac | as % MAC | static margin vs the given CG | C<sub>mα</sub> |
|---|---|---|---|
| 0.566 c_r = 1.013 m (interpolated from the AR-1.33 Geran-2) | 36.4 % | **+1.8 %** | −0.032 |
| **0.615 c_r = 1.101 m — adopted** | 43.6 % | **+9.0 %** | −0.156 |
| 0.667 c_r = 1.193 m (slender-body limit, 2/3 c_r) | 51.1 % | **+16.6 %** | −0.287 |

The CG sits 55 % of the root chord aft, so the whole plausible band is only
±0.09 m wide and it straddles neutral stability. Note also that the **empty**
CG of 1.370 m `[Q 3.4]` is *behind* every one of those neutral points — this
airframe is only stable because of the 14.9 kg nose payload. That is normal for
a one-way airframe, but it means the payload CG in `[Q 3.10]` is a
flight-safety number, not a bookkeeping one. See [DATA-GAPS.md](DATA-GAPS.md)
item 1.

### ⚠ `AERORP` goes on the CG, not on the neutral point

JSBSim applies the LIFT/DRAG/SIDE forces **at `AERORP`** and then adds `r × F`
about the CG *on top of* whatever the `PITCH`/`ROLL`/`YAW` axis functions
produce (`FGAerodynamics::Calculate`: `vMoments += vDXYZcg*vForces`). So putting
`AERORP` at the neutral point **and** carrying a C<sub>mα</sub> term counts the
static pitch stability twice.

That was not a theoretical worry — it grounded the aircraft on every launch.
With `AERORP` at 1.101 m (0.110 m behind the CG), ~245 N of lift during the
climb-out produced an extra **−27 N·m** of nose-down moment against the
**+34 N·m** the aero functions were making. Net pitching moment ≈ 0: the
aircraft would not rotate out of the post-launch mush no matter how much elevon
was commanded, and it flew into the ground ~4 s after release every time.
Moving `AERORP` onto the CG (0.991 m) makes `r × F` zero and leaves
C<sub>mα</sub> = −0.156/rad as the whole story — which is exactly what the
derivative set was computed for. Measured effect on the launch: post-release
sink fell from *ground contact* to **1.7 m**.

The Geran-2 model has the same double count (`AERORP` 1.698 m against a CG of
1.433 m) and is very likely paying for it with the extreme launch speed it
needs to get away.

## 5. Propulsion — what needed calibrating

Two findings, both documented inline in `Engines/uav_engine.xml` and
`Engines/uav_prop.xml`:

1. **Do not set `<bsfc>` on this engine.** With it, FGPiston derives power from
   its internal air/fuel model instead of `<maxhp>`, and this engine produced
   **1.06 hp / 45 N** instead of 7 hp / 160 N — the aircraft could not hold
   altitude. Removing it gives the rated power within 1 %. This is very likely
   the same root cause behind the Geran-2's abandoned piston model (replaced by
   an `<electric_engine>` for "never reaching a self-sustaining RPM"); the
   Delta-38 keeps a real FGPiston, so **fuel burn, endurance and range are
   simulated**.
2. **`<volumetric-efficiency>` 1.85 is a fuel-flow calibration constant**, not a
   physical VE. FGPiston's fuel-flow model assumes a four-stroke induction
   cycle and under-reads this two-stroke by ≈2.4×; the physical 0.78 implied a
   bsfc of 0.40 lb/hp/hr (better than a large diesel). 1.85 reproduces the
   measured `[Q 5.8a]` 2.0–2.5 L/h. Verified not to affect power.

**Propeller.** `[Q 6.7]` leaves the Ct/Cp tables to the engineer. They are
pinned by two constraints at J = 0: absorbing 5.22 kW at 6300 rpm on a 0.508 m
disc needs `Cp = P/(ρn³D⁵) = 0.109`, and momentum theory at FoM 0.55 gives
`T₀ = 160 N`, i.e. `Ct = 0.178`. The curves in between are standard 2-blade
fixed-pitch shapes (McCormick §9.4, Hartman & Biermann NACA TR-640) with peak
η = 0.77 at J = 0.8, crossing to negative Ct/Cp near J = 1.2–1.45 so the prop
brakes in a dive instead of endlessly loading the engine.

Cp = 0.109 implies a **wide-chord** blade — see DATA-GAPS.md item 3.

## 6. Catapult

VS at the typical 34.36 kg is **23.2 m/s**. Target release 34 m/s = 1.46 VS, so
the wing is flying — and flying at *low α* — the instant it leaves the rail:

```
F = m·ΔV/Δt = 34.36 · 34 / 0.40 s = 2921 N = 657 lbf   →  SITL_CATAPULT_LBF=660
a = 8.7 g, stroke = ½at² = 6.8 m
```

`SIM_JSBSim.cpp` now reads the magnitude from `SITL_CATAPULT_LBF`
(`JSBSim::catapult_lbf()`, default 4900 = the Geran-2's value) because it is
`m·ΔV/Δt` for a specific airframe — 4900 lbf on 34 kg would be a 65 g, 250 m/s
launch. `docker/sim/entrypoint.sh` sets it from the airframe registry in
`backend/app/models.py`.

Two deliberate differences from the Geran-2's launch:

* **Force direction is pure +X body.** The rail IC is 5° nose-up, so the
  aircraft leaves at γ = +5° with **α ≈ 0** — and at α = 0 this wing still makes
  CL0 = 0.091 thanks to its +3° setting angle. The Geran-2 tilts its force 15°
  up *in body axes*, which puts the velocity vector 15° above the body axis,
  i.e. **α = −15°** at release, and costs it a 3 m sink.
* **The force acts on the CG line.** A real rail carries the reaction through
  the belly but *also restrains the airframe in pitch for the whole stroke*,
  which JSBSim cannot do once `forces/hold-down` is released. An off-CG force
  integrates freely instead: 0.12 m of offset at 2921 N is 350 N·m against
  Iyy = 7.8, i.e. 45 rad/s² — it tumbles inside the 0.4 s stroke.

## Two traps in the SITL harness (both cost a day of "launches, then sinks")

Neither is a modelling error — both are cases where ArduPilot's JSBSim backend
was written against the Geran-2's `<electric_engine>` and quietly does not
supply what a real `FGPiston` needs.

### 1. Nobody sets the mixture

`SIM_JSBSim.cpp send_servos()` sends aileron, elevator, rudder and throttle,
and nothing else. An electric engine has no mixture, no magneto and no starter,
so this was never a problem. A piston engine with `fcs/mixture-pos-norm` at its
default of 0 gets **no fuel**: manifold pressure sits at 29.9 inHg, throttle at
1.0, and `power-hp` sits at the −0.07 hp static-friction figure while the
propeller windmills the engine down. Measured inside SITL: **4814 rpm at launch
decaying to 0 rpm over 3.8 s.** The aircraft flew the catapult stroke on the
catapult alone and then glided into the ground.

Fixed in the model, not the backend — `Systems/Aircraft control.xml` has a
`Mixture` channel that pins `fcs/mixture-pos-norm` at full rich. That is also
physically right (a small carburetted two-stroke has no mixture control) and it
keeps the model self-sufficient in any harness.

### 2. ArduPilot's throttle ramp lands in the middle of the stroke

`send_servos()` withholds ArduPilot's throttle while ArduPilot's own command is
below a threshold, so the engine keeps the startup event's 100 %. That threshold
was 5 %: enough for an electric engine, not for a piston. Measured: TECS ramps
its throttle from 0 to 100 % over ~1.3 s from arming (and ~0.5 s even with
`THR_SLEWRATE 0` / `TKOFF_THR_SLEW -1`, because TECS is still acquiring its
errors), so the real engine was handed 20 %, then 44 %... in the middle of the
0.4 s catapult stroke. rpm and manifold pressure collapse, and spinning the
propeller back up takes over a second. Release speed came out 33 m/s instead of
35.7, with almost no thrust afterwards.

Two fixes, both kept:

* `Delta-38-catapult.parm` sets `TKOFF_THR_SLEW -1` and `THR_SLEWRATE 0` — a
  catapult launch is committed at full power on the rail, so a throttle slew
  limit has no business being there.
* `SIM_JSBSim.cpp` now holds full throttle until ArduPilot itself asks for
  ≥95 % (was 5 %). On a takeoff that costs nothing — ArduPilot wants `THR_MAX`
  anyway — and it keeps the engine at rated power right through the launch.

### 3. And one that is a real model number: `TECS_SINK_MIN`

`TECS_SINK_MIN` is *"minimum sink rate at `THR_MIN` and `AIRSPEED_CRUISE`"* and
its default is 2.0 m/s. This airframe's measured idle glide is **−5.5 m/s at
34 m/s** (L/D ≈ 5.4 with a windmilling propeller). TECS therefore believed
closing the throttle cost almost no energy and over-corrected every time, which
showed up as a self-sustaining ~10 s limit cycle in cruise: ±12 m of altitude,
32–36 m/s, throttle slamming between 50 and 100 %. Feeding TECS the real
numbers (`TECS_SINK_MIN 5.5`, `TECS_CLMB_MAX 4.5`) is what settles it.

## Verification

Standalone JSBSim (`jsbsim 1.3.1`, the model driven through the Python API at
the 1200 Hz SITL runs at). Reproduce with `scripts/d38_cruise.xml` and
`scripts/d38_catapult.xml`.

> **Note on `forces/hold-down`:** both standalone drivers (the FlightGear
> `JSBSim` binary and the `jsbsim` Python CLI) treat a held-down FDM as "not
> running" and stop the run loop on the first frame. ArduPilot's SITL backend
> drives JSBSim through its console socket (`iterate 1`) and is unaffected, so
> hold-down is used there and works. `scripts/d38_catapult.xml` therefore rests
> the aircraft on its belly contacts instead of holding it down.

**Propulsion** (full throttle unless noted):

| condition | rpm | hp | thrust | fuel |
|---|---|---|---|---|
| static | 6353 | 6.95 | 157 N | 3.7 L/h |
| 34 m/s | 6350 | 6.94 | 104 N | 3.7 L/h |
| 34 m/s, 80 % throttle (= level cruise) | 5388 | 3.78 | 65 N | **2.2 L/h** |

Rated output is 7.00 hp at 6300 rpm `[Q 5.1/5.2]`; cruise burn target is
2.0–2.5 L/h `[Q 5.8a]`. Endurance 5.5 L ÷ 2.3 L/h = **2.4 h ≈ 290 km** at
122 km/h, against the questionnaire's stated 250+ km.

**Performance**

| | |
|---|---|
| best climb | **+4.8 m/s** at 31 m/s, full throttle |
| level cruise | 34–36 m/s at 78–80 % throttle |
| max level speed | ~40 m/s / 145 km/h |
| closed-throttle descent | engine windmills at 3300–3500 rpm and full power returns immediately |

**Catapult launch** — standalone FDM (`SITL_CATAPULT_LBF=660`, 0.4 s stroke,
15 m rail height)

| | |
|---|---|
| release speed | **35.8 m/s** (1.54 VS) |
| release α / θ | 5.6° / 0.5° |
| **minimum height after release** | **13.3 m** — a 1.7 m sink from the 15 m rail |
| roll excursion during the stroke | ~8°, from propeller torque reaction (≈8 N·m against Ixx = 1.85); washes out once the roll loop has authority |

For comparison the Geran-2 sinks ~3.3 m to a 4.6 m clearance from 8 m.

**Full ArduPlane SITL mission** — one `mcc-sim` container, `-f
jsbsim:Delta-38-catapult`, AUTO with `NAV_TAKEOFF` (120 m) + waypoints at 150 m,
4 and 9 km out:

| | |
|---|---|
| release speed | **36.0 m/s** ("Holding course … at 36.0 m/s") |
| **minimum AGL after arm** | **13.7 m** from a 14.8 m rail — a **1.1 m** sink |
| climb-out | steady +4.9 to +5.1 m/s at 32.5 m/s, no speed bleed |
| takeoff complete | 120.3 m at t = 24.7 s |
| cruise | **34.0 m/s and 150.0 m held to ±0.1 m**, throttle steady at 75–77 %, pitch 5.3–5.9°, roll ≈ 0 |
| waypoint 2 | passed at t = 116 s, **1 m** miss distance |
| waypoint 3 | passed at t = 260 s, **3 m** miss distance |

**Dynamic modes** (open loop**Dynamic modes** (open loop, controls fixed at trim after a pulse)

* Short period: an elevator pulse takes α from 6.5° to 8.4° and back inside
  1 s, **no overshoot, no oscillation**.
* Phugoid: slow and lightly damped (θ 9°→3° over 10 s with ±1 m/s of speed) —
  normal, and TECS handles it.
* Roll/spiral: a 20 % aileron pulse gives 22–27° of bank that then washes out;
  β stays under 2° throughout, so the Dutch roll is well damped.
* With no aileron input the aircraft rolls right at ~1.5 °/s from propeller
  torque. The trim needed is δa ≈ 0.9°, so the autopilot absorbs it easily.
