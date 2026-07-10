# JSBSim Aircraft Parameter Template

---

# File: `<aircraft-name>.xml`

## Geometry (`<metrics>`)

| Parameter | Description | Units | Example |
|---|---|---|---|
| `wingarea` | Reference wing area (full planform) | M2 / FT2 | 4.6875 M2 |
| `wingspan` | Tip-to-tip span | M / FT | 2.50 M |
| `chord` | Mean aerodynamic chord (MAC) | M / FT | 2.10 M |
| `wing_incidence` | Wing setting angle vs fuselage reference | DEG | 0 |
| `htailarea` | Horizontal tail area (0 if none) | M2 | 0 |
| `htailarm` | Horizontal tail aero centre to CG distance | M | 0 |
| `vtailarea` | Vertical tail / winglet area | M2 | 0.43 |
| `vtailarm` | Vertical tail / winglet arm to CG | M | 1.90 |
| `location AERORP` | Aerodynamic reference point (neutral point), x/y/z | M / IN | x = 1.698 |
| `location EYEPOINT` | Reference point for simulated accelerometer output — place at IMU/CG | M / IN | x = 1.4326 |


```xml
<metrics>
  <wingarea  unit="M2"> 4.6875 </wingarea>
  <wingspan  unit="M">  2.50   </wingspan>
  <wing_incidence unit="DEG"> 0.00 </wing_incidence>
  <chord     unit="M">  2.10   </chord>
  <htailarea unit="M2"> 0.00   </htailarea>
  <htailarm  unit="M">  0.00   </htailarm>
  <vtailarea unit="M2"> 0.43   </vtailarea>
  <vtailarm  unit="M">  1.90   </vtailarm>
  <location name="AERORP" unit="M">
    <x> 1.698 </x>
    <y> 0.00  </y>
    <z> 0.00  </z>
  </location>
  <location name="EYEPOINT" unit="M">
    <x> 1.4326 </x>
    <y> 0.00   </y>
    <z> 0.00   </z>
  </location>
</metrics>
```

---

## Mass & Balance (`<mass_balance>`)

| Parameter | Description | Units | Example |
|---|---|---|---|
| `ixx` | Roll moment of inertia | KG*M2 / SLUG*FT2 | 41.7 |
| `iyy` | Pitch moment of inertia | KG*M2 | 153.6 |
| `izz` | Yaw moment of inertia | KG*M2 | 205.0 |
| `ixz` | X-Z product of inertia | KG*M2 | 2.5 |
| `ixy`, `iyz` | Other products of inertia (0 if symmetric) | KG*M2 | 0 |
| `emptywt` | Structural empty weight (no fuel, no payload) | KG / LBS | 52.0 |
| `location CG` | Structural empty-CG location, x/y/z | M / IN | x = 1.20 |
| `pointmass` *(one per item)* | Discrete mass: `name`, `weight`, `location` x/y/z. Repeat for each component | KG, M | see below |

```xml
<mass_balance>
  <ixx unit="KG*M2"> 41.7  </ixx>
  <iyy unit="KG*M2"> 153.6 </iyy>
  <izz unit="KG*M2"> 205.0 </izz>
  <ixz unit="KG*M2"> 2.5   </ixz>
  <emptywt unit="KG"> 52.0 </emptywt>
  <location name="CG" unit="M">
    <x> 1.20 </x>
    <y> 0.00 </y>
    <z> 0.00 </z>
  </location>

  <pointmass name="Payload">
    <weight unit="KG"> 50.0 </weight>
    <location name="POINTMASS" unit="M">
      <x> 0.35 </x>
      <y> 0.00 </y>
      <z> 0.00 </z>
    </location>
  </pointmass>

  <pointmass name="Engine">
    <weight unit="KG"> 18.0 </weight>
    <location name="POINTMASS" unit="M">
      <x> 3.25 </x>
      <y> 0.00 </y>
      <z> 0.00 </z>
    </location>
  </pointmass>
</mass_balance>
```




---

## Propulsion mount (`<propulsion>`)

### Engine mount

| Parameter | Description | Units | Example |
|---|---|---|---|
| `location` | Engine position, x/y/z | M / IN | 3.20, 0, 0 |
| `orient` | Mount pitch/roll/yaw | DEG | 0, 0, 0 |
| `feed` | Tank index this engine draws from | index | 0 |

### Thruster mount (nested inside `<engine>`)

| Parameter | Description | Units | Example |
|---|---|---|---|
| `location` | Propeller plane position | M / IN | 3.50, 0, 0 |
| `sense` | +1 right-hand rotation, −1 left | — | 1 |
| `p_factor` | Asymmetric-thrust yaw factor | — | 3 |

### Fuel tank (`<tank type="FUEL">`)

| Parameter | Description | Units | Example |
|---|---|---|---|
| `type` | FUEL or OXIDIZER | — | FUEL |
| `number` | Tank index (matches engine `feed`) | — | 0 |
| `location` | Tank position (affects CG as fuel burns) | M / IN | 1.90, 0, 0 |
| `capacity` | Maximum contents | LBS | 138.9 |
| `contents` | Initial contents | LBS | 138.9 |
| `density` | Fuel density | LBS/GAL | 5.97 |

```xml
<propulsion>
  <engine file="MyEngine">
    <location unit="M"> <x>3.20</x> <y>0.00</y> <z>0.00</z> </location>
    <orient unit="DEG"> <pitch>0</pitch> <roll>0</roll> <yaw>0</yaw> </orient>
    <feed> 0 </feed>

    <thruster file="MyPropeller">
      <location unit="M"> <x>3.50</x> <y>0.00</y> <z>0.00</z> </location>
      <orient unit="DEG"> <pitch>0</pitch> <roll>0</roll> <yaw>0</yaw> </orient>
      <sense> 1 </sense>
      <p_factor> 3 </p_factor>
    </thruster>
  </engine>

  <tank type="FUEL" number="0">
    <location unit="M"> <x>1.90</x> <y>0.00</y> <z>0.00</z> </location>
    <capacity unit="LBS"> 138.9 </capacity>
    <contents unit="LBS"> 138.9 </contents>
    <density  unit="LBS/GAL"> 5.97 </density>
  </tank>
</propulsion>
```

---

## Aerodynamics (`<aerodynamics>`)

Each entry is a `<function>` = `qbar · S · [c̄ or b] · coefficient · state`. One `<axis>` block per axis (LIFT, DRAG, SIDE, PITCH, ROLL, YAW), containing one `<function>` per coefficient.

### LIFT

| Coefficient | Description | Units | Example |
|---|---|---|---|
| `CL0` | Lift at zero angle of attack | — | 0.05 |
| `CLα` | Lift-curve slope (often a table covering stall) | 1/RAD | 1.552 |
| `CLq` | Lift due to pitch rate | 1/RAD | 1.20 |
| `CLα̇` | Lift due to angle-of-attack rate | 1/RAD | 0.40 |
| `CLδe` | Lift due to elevator/elevon deflection | 1/RAD | 0.35 |
| `CLmax` | Peak lift coefficient and its angle of attack | — / DEG | 1.02 @ 40° |

```xml
<axis name="LIFT">
  <function name="aero/force/Lift_alpha">
    <description>Lift due to alpha, CLa=1.552/rad</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <table>
        <independentVar lookup="row">aero/alpha-rad</independentVar>
        <tableData>
          -0.175  -0.220
           0.000   0.050
           0.200   0.360
           0.698   1.020
        </tableData>
      </table>
    </product>
  </function>

  <function name="aero/force/Lift_elevon">
    <description>Lift due to elevon deflection, CLde=0.35/rad</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>fcs/elevator-pos-rad</property>
      <value> 0.35 </value>
    </product>
  </function>
</axis>
```

### DRAG

| Coefficient | Description | Units | Example |
|---|---|---|---|
| `CD0` | Zero-lift drag | — | 0.020 |
| `k` | Induced drag factor, `CDi = k·CL²`, `k = 1/(π·e·AR)` | — | 0.341 |
| `e` | Oswald efficiency factor | — | 0.70 |
| `CDβ` | Drag due to sideslip (table) | — | table |
| `CDδe` | Drag due to elevator/elevon deflection | 1/RAD | 0.010 |

```xml
<axis name="DRAG">
  <function name="aero/force/Drag_zero_lift">
    <description>CD0=0.020</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <value> 0.020 </value>
    </product>
  </function>

  <function name="aero/force/Drag_induced">
    <description>CDi=0.341*CL^2</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>aero/cl-squared</property>
      <value> 0.341 </value>
    </product>
  </function>
</axis>
```

### SIDE FORCE

| Coefficient | Description | Units | Example |
|---|---|---|---|
| `CYβ` | Side force due to sideslip | 1/RAD | −0.25 |
| `CYr` | Side force due to yaw rate | 1/RAD | 0.20 |

```xml
<axis name="SIDE">
  <function name="aero/force/Side_beta">
    <description>CYb=-0.25/rad</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>aero/beta-rad</property>
      <value> -0.25 </value>
    </product>
  </function>
</axis>
```

### PITCH

| Coefficient | Description | Units | Example |
|---|---|---|---|
| `Cm0` | Pitching moment at zero angle of attack | — | 0.040 |
| `Cmα` | Static pitch stability (negative = stable) | 1/RAD | −0.195 |
| `Cmδe` | Pitch control power (elevator/elevon) | 1/RAD | −0.350 |
| `Cmq` | Pitch damping (must be negative) | 1/RAD | −3.5 |
| `Cmα̇` | Downwash-lag term | 1/RAD | −0.80 |

```xml
<axis name="PITCH">
  <function name="aero/moment/Pitch_alpha">
    <description>Cma=-0.195/rad; NEGATIVE=stable</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>metrics/cbarw-ft</property>
      <property>aero/alpha-rad</property>
      <value> -0.195 </value>
    </product>
  </function>

  <function name="aero/moment/Pitch_damping">
    <description>Cmq=-3.5/rad; MUST be negative</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>metrics/cbarw-ft</property>
      <property>aero/ci2vel</property>
      <property>velocities/q-aero-rad_sec</property>
      <value> -3.5 </value>
    </product>
  </function>
</axis>
```

### ROLL

| Coefficient | Description | Units | Example |
|---|---|---|---|
| `Clβ` | Dihedral effect (negative = stable) | 1/RAD | −0.035 |
| `Clp` | Roll damping (must be negative) | 1/RAD | −0.25 |
| `Clr` | Roll due to yaw rate | 1/RAD | 0.06 |
| `Clδa` | Roll control power (aileron/differential elevon) | 1/RAD | 0.20 |

```xml
<axis name="ROLL">
  <function name="aero/moment/Roll_damping">
    <description>Clp=-0.25/rad; MUST be negative</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>metrics/bw-ft</property>
      <property>aero/bi2vel</property>
      <property>velocities/p-aero-rad_sec</property>
      <value> -0.25 </value>
    </product>
  </function>

  <function name="aero/moment/Roll_elevon_differential">
    <description>Clda=0.20/rad</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>metrics/bw-ft</property>
      <property>fcs/aileron-pos-rad</property>
      <value> 0.20 </value>
    </product>
  </function>
</axis>
```

### YAW

| Coefficient | Description | Units | Example |
|---|---|---|---|
| `Cnβ` | Weathercock stability (must be positive) | 1/RAD | 0.040 |
| `Cnr` | Yaw damping (must be negative) | 1/RAD | −0.060 |
| `Cnp` | Yaw due to roll rate | 1/RAD | −0.015 |
| `Cnδa` | Adverse yaw from aileron/differential elevon | 1/RAD | 0.008 |

```xml
<axis name="YAW">
  <function name="aero/moment/Yaw_beta">
    <description>Cnb=+0.040/rad; POSITIVE=directionally stable</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>metrics/bw-ft</property>
      <property>aero/beta-rad</property>
      <value> 0.040 </value>
    </product>
  </function>

  <function name="aero/moment/Yaw_damping">
    <description>Cnr=-0.060/rad; MUST be negative</description>
    <product>
      <property>aero/qbar-psf</property>
      <property>metrics/Sw-sqft</property>
      <property>metrics/bw-ft</property>
      <property>aero/bi2vel</property>
      <property>velocities/r-aero-rad_sec</property>
      <value> -0.060 </value>
    </product>
  </function>
</axis>
```

---

# File: `Engines/<engine-name>.xml`

## Piston Engine

| Parameter | Description | Units | Example |
|---|---|---|---|
| `displacement` | Swept volume | IN3 | 300 |
| `maxhp` | Rated power | HP | 50 |
| `cycles` | 2 or 4 stroke | — | 2 |
| `idlerpm` | Idle speed | RPM | 700 |
| `maxrpm` | Redline speed | RPM | 7000 |
| `bsfc` | Brake specific fuel consumption | LBS/HP/HR | 0.45 |
| `numcylinders`/`cylinders` | Cylinder count | — | 2 |
| `compression-ratio` | Compression ratio | — | 8.0 |
| `boostspeeds` | Number of supercharger/turbo stages (0 if none) | — | 0 |

```xml
<piston_engine name="MyEngine">
  <displacement unit="IN3"> 300 </displacement>
  <maxhp>       50   </maxhp>
  <cycles>       2   </cycles>
  <idlerpm>    700   </idlerpm>
  <maxrpm>    7000   </maxrpm>
  <bsfc>         0.45 </bsfc>
  <cylinders>    2   </cylinders>
  <compression-ratio> 8.0 </compression-ratio>
</piston_engine>
```

---

# File: `Engines/<propeller-name>.xml`

| Parameter | Description | Units | Example |
|---|---|---|---|
| `ixx` | Propeller rotational inertia | SLUG*FT2 | 0.20 |
| `diameter` | Propeller diameter | IN / FT | 39.37 IN (1.0 M) |
| `numblades` | Blade count | — | 2 |
| `gearratio` | engine RPM ÷ propeller RPM (reduction) | — | 1.50 |
| `minrpm` | Minimum operating RPM | RPM | 800 |
| `maxrpm` | Maximum operating RPM | RPM | 7000 |
| `sense` | Rotation direction (+1/−1) | — | 1 |
| `<table name="C_THRUST">` | Thrust coefficient vs advance ratio J = V/(n·D) | Ct vs J | table |
| `<table name="C_POWER">` | Power coefficient vs advance ratio J | Cp vs J | table |

```xml
<propeller version="1.01" name="MyPropeller">
  <ixx>  0.20  </ixx>
  <diameter unit="IN"> 39.37 </diameter>
  <numblades>  2  </numblades>
  <gearratio>  1.50 </gearratio>
  <minrpm>   800  </minrpm>
  <maxrpm>  7000  </maxrpm>
  <sense>      1  </sense>

  <table name="C_THRUST" type="internal">
    <tableData>
      0.0  0.10
      0.5  0.08
      1.0  0.05
    </tableData>
  </table>

  <table name="C_POWER" type="internal">
    <tableData>
      0.0  0.06
      0.5  0.05
      1.0  0.04
    </tableData>
  </table>
</propeller>
```
