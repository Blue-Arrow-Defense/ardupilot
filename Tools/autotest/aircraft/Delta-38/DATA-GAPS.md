# Delta-38 — what the questionnaire does not answer

Written against `uav jsbsim questionnaire v3_13-08-26.docx`. Section 8 of that
questionnaire already says the right thing: *«якщо розділи 2 (Геометрія) і 3
(Маса та центрування) заповнені якомога точніше — це вже дає інженеру
достатньо, щоб побудувати робочу початкову аеродинамічну модель»*. That is
exactly what happened — the model flies. What follows is the list of values the
model currently has to **estimate**, ordered by how much the estimate can change
the aircraft's behaviour, with the specific question to send back to the
manufacturer.

Every item below corresponds to an `[EST]` tag in `Delta-38.xml`.

## Status: the model flies

Worth saying up front, because it changes how urgent these are. The aircraft
launches off the catapult and flies a full autonomous mission in ArduPlane SITL:
released at 36 m/s, **1.1 m of sink** from a 14.8 m rail, a steady +4.9 m/s
climb-out, then cruise held at **34.0 m/s and 150.0 m ±0.1 m** with the throttle
steady at 76 %, hitting waypoints 4 and 9 km out to **1 m and 3 m**. None of the
gaps below are blocking. What each one buys is *confidence* — the difference
between "behaves correctly" and "behaves like your aircraft".

Priority for the manufacturer, if only some can be answered:

| | what to ask for | why |
|---|---|---|
| **1** | the CG tolerance / static-margin figure, and confirmation of the payload CG (item 1) | the only number that could make the real aircraft behave *qualitatively* differently |
| **2** | one steady-level-flight point: speed + throttle or fuel flow (item 5) | pins CD₀ exactly, which sets cruise speed, range and climb margin |
| **3** | the propeller's make and model (item 3) | replaces a whole inferred table set with measured data |
| **4** | elevon span and chord (item 2) | sets how the autopilot has to be tuned |
| **5** | distance from the datum plane to the lowest point of the fuselage (item 4) | on the numbers as given, the propeller is the lowest part of the aircraft |

---

## 1. Pitch stability — the position of the neutral point ⚠ highest impact

**What we assumed.** Aerodynamic centre at 0.615 × root chord = **1.101 m** from
the nose (43.6 % MAC), giving C<sub>mα</sub> = −0.156 /rad and a static margin
of **9.0 % MAC**.

**Why it matters.** A slender delta's neutral point is nowhere near the 25 %
MAC the questionnaire's hint suggests, and published estimates for this
planform (AR 1.61, LE sweep 60.7°) span 0.566–0.667 c<sub>root</sub>. Against
the supplied CG of 0.991 m that is a static margin of anywhere from **+1.8 % to
+16.6 % MAC** — from "barely stable, needs a fast autopilot" to "comfortably
stable". Worse, the **empty** CG of 1.370 m (item 3.4) is *behind* every one of
those neutral points: the aircraft is stable only because of the 14.9 kg nose
payload. Two consequences:

* the payload CG (item 3.10, x = 0.600 m) is a **flight-safety** number — a
  100 mm error moves the all-up CG by 43 mm, i.e. 3.5 % MAC, over a third of the
  assumed static margin;
* any mission flown **without** the payload, or with a lighter one, is
  longitudinally unstable and would need its own CG plan.

**What we need — any one of these, best first**

1. Measured C<sub>mα</sub> or neutral-point position (wind tunnel, or a
   flight-test pitch-response record).
2. The designer's own static-margin figure, or the CG range they certify
   ("CG must lie between x = … and x = …").
3. Confirmation of the empty CG (1.370 m) and the payload CG (0.600 m) to
   ±20 mm, plus **whether the aircraft is ever flown without the payload**.
4. Trimmed cruise data: airspeed + elevon trim angle at a known weight. Two
   such points at different speeds pin C<sub>mα</sub> directly.

## 2. Elevon geometry — the whole pitch/roll control model

**What we assumed.** Elevons of 22 % local chord spanning 20–80 % of each
semi-span: 0.231 m² total (13 % of wing area), travel ±20° pitch / ±15° roll,
giving CL<sub>δe</sub> = 0.55, C<sub>mδe</sub> = −0.309, C<sub>lδa</sub> = 0.15.

**Why it matters.** The questionnaire flags this itself under CL<sub>δe</sub>:
*«наразі не покрито анкетою — потрібні розміри елевонів»*. Control power is
what sets how hard the autopilot has to be tuned. With the assumed numbers,
full deflection commands an α far past stall, so `Delta-38-catapult.parm`
carries deliberately soft pitch gains. If the real elevons are smaller the
aircraft will feel sluggish and the launch rotation will be slower; if larger,
the current gains will be too hot.

**What we need**

* Elevon span (inboard and outboard station, in mm from the centreline) and
  chord (mm, or % of the local wing chord).
* Mechanical travel limits, in degrees, up and down.
* Whether the two surfaces are mixed mechanically or in the autopilot, and
  whether there is differential (more up-travel than down).

## 3. Propeller — pitch, blade width, or simply the part number

**What we assumed.** Ct/Cp curves for a wide-chord 2-blade prop at an effective
pitch/D ≈ 0.85, peak η = 0.77 at J = 0.8.

**Why it matters.** Item 6.1 gives only the diameter (20″ = 0.508 m) and item
6.7 leaves the tables to the engineer. Those two facts plus items 5.1/5.2
(5.22 kW at 6300 rpm, direct drive) are actually over-constrained: absorbing
5.22 kW on a 0.508 m disc at 6300 rpm requires C<sub>p</sub> = 0.109, roughly
**2.5× the blade loading of a thin 20×10 sport propeller**. A thin hobby prop
would absorb only ~2 kW there and the engine would overspeed past its redline
instead of making rated power. So the real installation must be using a
wide-chord UAV propeller (Mejzlik / Biela / Xoar PJN class) at 13–15″ of pitch —
we have modelled that, but we are inferring the hardware.

**What we need**

* Propeller **make and model** (e.g. "Mejzlik 20×13 CFRP"). That alone replaces
  the whole table set with the manufacturer's own data.
* Failing that: geometric pitch (inches or degrees at 0.75 R) and blade chord
  at 0.75 R.
* Static thrust at full throttle, if it has ever been measured on a bench — a
  single number would validate the whole propulsion chain (we predict **157 N /
  16 kgf**).

## 4. Vertical geometry — the propeller does not clear the ground

**What we assumed.** Belly contacts 100–120 mm below the datum plane and a tail
skid at 270 mm, just below the propeller tip.

**Why it matters.** On the numbers supplied, the propeller disc centre is at
z = −10 mm (item 4.9) and the diameter is 508 mm, so **the propeller tip reaches
264 mm below the datum plane** — which makes the propeller the lowest point of
the aircraft. Item 2.9a explicitly says one purpose of the overall length is
*«щоб гвинт не «зачіпав» землю»*, so this is a check the questionnaire wanted
made, and on the data as given it fails. Either the belly is deeper than 264 mm,
or the aircraft rests nose-down on the propeller during belly recovery. The
model currently keeps the prop clear artificially; the real number changes
ground handling and recovery damage modelling.

**What we need**

* Distance from the datum plane (the z = 0 reference used in section 2.10) down
  to the **lowest point of the fuselage**, and to the lowest point of any skid.
* Propeller tip clearance to the ground with the aircraft resting on its belly.

## 5. Drag level — CD<sub>0</sub>

**What we assumed.** CD<sub>0</sub> = 0.032 (Raymer build-up: 0.0097 of clean
skin friction, the rest for the exposed cylinder head and cooling, the engine
fairing, the pusher pylon, the winglets and a belly launch-rail fitting).

**Why it matters.** It sets cruise speed, range and how much climb margin the
engine has. Everything else in the model is now anchored to measured
questionnaire data, so CD<sub>0</sub> is the main remaining lever on
performance. The model currently predicts max level speed ≈ 145 km/h and
2.2–2.5 L/h at 122 km/h.

**What we need — one flight-test point closes this completely**

* Airspeed and throttle (or fuel flow, or rpm) in **steady level flight** at a
  known weight and altitude. One such triplet pins CD<sub>0</sub> exactly.
* Or: maximum level speed at full throttle, sea level.

## 6. Smaller items (each worth a line, none of them flight-critical)

| item | questionnaire | assumed | what would replace it |
|---|---|---|---|
| Winglet arm | 2.8 left blank | 0.579 m (winglet ac at 25 % tip chord) | distance from the CG to the winglet's aerodynamic centre |
| Moments of inertia | 3.5–3.7 answered `0` | Ixx 1.85 / Iyy 7.80 / Izz 9.31 kg·m² from a point-mass + plate summation | CAD or bifilar-pendulum values; the model's own consistency checks pass, so this is a refinement |
| P-factor | 4.11 marked "engineer decides" | 2 | nothing needed — it only matters at high α |
| Compression ratio | 5.7 answered `0` | 9.0:1 | engine data sheet; only trims fuel consumption |
| Propeller inertia | 6.6 marked "engineer decides" | 0.0196 kg·m² (0.35 kg/blade, slender rod + 30 % hub) | propeller mass, if convenient |
| Fuel type / density | 4.13/4.14 give litres but not grade | MOGAS A-95, 0.72 kg/L | grade (A-95 / AvGas 100LL / synthetic mix), and the oil ratio for the two-stroke |
| Winglet toe / cant | not asked | 0 | only matters for a directional-trim bias |

## 7. Two data-consistency notes worth raising

* **MAC.** Item 2.3 gives 1.243 m. The planform implied by items 2.1, 2.2 and
  2.9 (S = 1.75 m², b = 1.68 m, nose-to-trailing-edge 1.79 m) gives 1.221 m —
  1.8 % apart. Harmless, but it suggests either the root chord is nearer 1.835 m
  than 1.79 m, or the MAC was computed for a slightly different planform. Worth
  a sanity check against the drawing.
* **Mass budget.** Empty 15.5 + payload 14.9 + fuel 3.96 kg = **34.36 kg**
  against the 38 kg MTOW of item 3.2. Is the remaining 3.6 kg payload growth,
  a heavier fuel load than 5.5 L, or is one of the masses out of date? The
  model flies the 34.36 kg configuration and quotes stall speeds for both.

---

## Two things the simulation itself turned up, worth passing back

Neither is a missing answer — both are findings from building and flying the
model that the manufacturer may care about.

**The tank is on the CG.** Burning all 5.5 L moves the centre of gravity by
**1.9 mm**. Whoever placed that tank did it right, and it means fuel state has
no effect on trim at all. Worth knowing if the layout is ever revised.

**Fuel consumption implies a thirsty engine, and that is fine.** Working
backwards from item 5.8a (2.0–2.5 L/h) and the power the airframe actually needs
at cruise (~3.8 hp) gives a brake specific fuel consumption of about
**0.95 lb/hp/hr**. That is high by aviation standards but completely normal for a
carburetted model two-stroke; it is quoted here only so that nobody later
"corrects" it to an aviation figure and doubles the predicted range. The model
reproduces the stated numbers: 5.5 L ÷ 2.3 L/h = **2.4 h ≈ 290 km** at 122 km/h,
against the questionnaire's stated 250+ km.

## What this does *not* need

For completeness, the following were **not** estimated — they came straight from
the questionnaire and are used as given: wing area, span, MAC, incidence,
winglet area, both lengths, the IMU position, VS1, MTOW, empty weight, empty CG,
engine and payload point masses, tank position and capacity, engine rated power,
redline and idle rpm, displacement, stroke count, cylinder count, cruise fuel
flow, propeller diameter, blade count, gear ratio, rotation sense, the absence
of a supercharger, landing gear, flaps and a horizontal tail.
