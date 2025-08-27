---
layout: project
title: "High-Pressure Dual Gear Pump System for Catalyst Regeneration"
date: 2024-04-15
description: "Designed and fabricated a portable dual gear pump system for industrial catalyst regeneration using precise hydraulic analysis, inverter control technology, modular aluminum frame design, and comprehensive electrical integration."
permalink: /projects/dual-gear-pump-system/
---

# Design and Fabrication of High-Pressure Dual Gear Pump System for Catalyst Regeneration

## Abstract

This study presents the design and fabrication of a portable dual gear pump system for catalyst regeneration applications in industrial facilities. The objective was to transport high-viscosity catalyst regeneration solution over long distances (discharge 50m) and high head (static head over 50m) while achieving approximately 4 bar injection pressure for effective catalyst cleaning and reactivation. The system consists of two gear pumps, three-phase induction motors, LS ELECTRIC inverters, analog flowmeters and pressure gauges, weatherproof control panels, and 4040/4080 aluminum profile frames. The design process involved precise head analysis using Darcy-Weisbach equations and minor loss calculations, establishing suction conditions to prevent cavitation through NPSH calculations. The final dual pump configuration successfully achieved approximately 20 L/min total flow rate with independent control capability.

<figure>
  <img class="project-image"
       src="{{ '/project/dual-gear-pump-system/complete-system-main.jpg' | relative_url }}"
       alt="Fabricated Dual Gear Pump System"
       loading="lazy">
  <figcaption>Completed dual gear pump system for catalyst regeneration with aluminum frame, dual pumps, and weatherproof control panel

---

## 1. Introduction

### 1.1 Background

Catalyst regeneration processes in industrial facilities are critical operations for recovering catalyst activity and extending catalyst lifetime. The regeneration solution used in these processes contains specific chemical compounds designed to remove contaminants and restore active sites on the catalyst surface. This solution has a density slightly higher than water (approximately 1,050 kg/m³), and it is essential to efficiently transport this solution while maintaining precise injection pressure for optimal catalyst treatment. Conventional single pump systems have shown limitations in flow control and lack backup system reliability for continuous catalyst regeneration operations.

### 1.2 Objectives and Scope

The objectives of this study are as follows:
- Design an optimized pump system for long-distance transport of catalyst regeneration solution
- Enhance system reliability through dual pump configuration for continuous operation
- Implement precise flow and pressure control via inverter control for optimal catalyst treatment
- Design portable frame considering field applicability in various industrial environments

### 1.3 Design Requirements

Key design requirements include:
- Suction: 3/4" hose, 25m length
- Discharge: 1/4" hose × 2 lines, 50m each
- Static head: Over 50m
- Nozzle injection pressure: 4 bar for effective catalyst regeneration
- Total flow rate: Over 20 L/min
- Portability: Caster-mounted with weatherproof control panel


---

## 2. Theoretical Background and Design Fundamentals

### 2.1 Fluid Properties and Assumptions

Fluid properties for design calculations were established as follows:
- Density (ρ): 1,050 kg/m³
- Kinematic viscosity (ν): 1.4-1.6 mm²/s @ ambient temperature
- Dynamic viscosity (μ): Approximately 1.5 mPa·s

### 2.2 Nozzle Flow Calculation Theory

Flow through nozzles was calculated using the orifice equation:

```
Q = Cd × A × √(2Δp/ρ)
```

Where:
- Q: Flow rate (m³/s)
- Cd: Discharge coefficient (0.62 applied)
- A: Orifice area (m²)
- Δp: Pressure difference (Pa)
- ρ: Density (kg/m³)

Assuming an actual orifice diameter of 1.6mm for the handheld 5-bolt catalyst injection nozzles, the calculated flow per nozzle was 2.06 L/min. For a total of 10 nozzles (2 guns × 5 nozzles/gun), a total flow rate of 20.6 L/min is required for effective catalyst regeneration coverage.

### 2.3 Pipe Loss Calculation Theory

#### 2.3.1 Darcy-Weisbach Equation

Friction losses in piping were calculated using the Darcy-Weisbach equation:

```
hf = f × (L/D) × (v²/2g)
```

Where:
- hf: Friction loss head (m)
- f: Pipe friction factor
- L: Pipe length (m)
- D: Pipe diameter (m)
- v: Average velocity (m/s)
- g: Gravitational acceleration (9.81 m/s²)

#### 2.3.2 Reynolds Number and Friction Factor

The pipe friction factor in turbulent flow was calculated using the Blasius formula:

```
f = 0.316 × Re^(-0.25)    (Re < 10⁵)
```

The Reynolds number is calculated as:

```
Re = (v × D) / ν
```

#### 2.3.3 Minor Losses

Minor losses due to valves, fittings, connectors, elbows, etc., were calculated using:

```
hm = KΣ × (v²/2g)
```

The total loss coefficient (K) for various fittings was assumed to be 20 based on reference data.

---

## 3. System Design and Calculations

### 3.1 Head Calculations and Analysis

#### 3.1.1 Discharge Line Loss Calculation (1/4" × 50m, Single Line)

Calculation conditions for single discharge line:
- Flow rate: Qgun ≈ 10.3 L/min = 1.72×10⁻⁴ m³/s
- Internal diameter: D = 6.35mm (1/4" hose)
- Cross-sectional area: A = 3.17×10⁻⁵ m²
- Average velocity: v = Q/A ≈ 5.42 m/s

Reynolds number calculation:
```
Re = (5.42 × 0.00635) / (1.5×10⁻⁶) ≈ 2.29×10⁴
```

Pipe friction factor (Blasius formula):
```
f = 0.316 × (2.29×10⁴)^(-0.25) ≈ 0.025
```

Friction loss head:
```
hf = 0.025 × (50/0.00635) × (5.42²/(2×9.81)) ≈ 300m
```

Minor loss head:
```
hm = 20 × (5.42²/(2×9.81)) ≈ 30m
```

#### 3.1.2 Static Head and Nozzle Head

Static head was set at 50m discharge height, and the head corresponding to 4 bar spray pressure at the nozzle is:

```
hn = Δp/(ρ×g) = (4×10⁵)/(1050×9.81) ≈ 38.8m
```

#### 3.1.3 Total Required Head

Total required head for single discharge line:
```
Htot = Hs + hf + hm + hn = 50 + 300 + 30 + 38.8 = 418.8m
```

Corresponding pump discharge pressure:
```
p = ρ × g × Htot = 1050 × 9.81 × 418.8 ≈ 4.31×10⁶ Pa = 43.1 bar
```

### 3.2 Pump Power Calculation

#### 3.2.1 Hydraulic Power

Hydraulic power for total flow rate of 20.6 L/min:
```
Ph = p × Q = 43.1×10⁵ × (20.6/60000) = 1.48 kW
```

#### 3.2.2 Motor Output Determination

Assuming pump and mechanical efficiency η = 0.70:
```
Pm = Ph/η = 1.48/0.70 ≈ 2.1 kW
```

Considering field operating conditions (viscosity changes, temperature rise, wear, etc.), a safety factor was applied, and 3.7kW (5HP) motors were selected.

### 3.3 Suction Line NPSH Review

#### 3.3.1 Suction Friction Loss

Loss calculation for suction line (3/4" × 25m):
- Total flow rate: Q = 3.43×10⁻⁴ m³/s
- Internal diameter: D ≈ 19mm
- Average velocity: v ≈ 1.21 m/s
- Reynolds number: Re ≈ 1.54×10⁴
- Pipe friction factor: f ≈ 0.028

Friction loss head:
```
hf,suction = 0.028 × (25/0.019) × (1.21²/(2×9.81)) ≈ 2.76m
```

#### 3.3.2 NPSH Available Calculation

```
NPSHA = Patm/(ρg) - Pv/(ρg) - hf,suction - hstatic
NPSHA = 10.3 - 0.3 - 2.76 - 0 = 7.24m
```

This provides sufficient margin compared to typical gear pump NPSHR (0.5-2m).

---

## 4. Major Component Selection and Design

<figure>
  <img class="flowchart"
       src="{{ '/project/dual-gear-pump-system/system-architecture.png' | relative_url }}"
       alt="System Architecture"
       loading="lazy">
  <figcaption>Figure 4.1: Complete system architecture showing flow paths and control integration

### 4.1 Pump Selection

#### 4.1.1 Pump Type Selection Rationale

Key reasons for selecting gear pumps:
1. High pressure capability: Can generate differential pressures over 40 bar required for catalyst regeneration
2. Positive displacement: Flow rate proportional to speed for consistent catalyst delivery
3. Viscosity tolerance: Stable performance with catalyst regeneration solutions
4. Self-priming capability: Easy initial priming for field operations
5. Structural simplicity: Easy maintenance in industrial environments

<figure>
  <img class="project-image"
       src="{{ '/project/dual-gear-pump-system/gear-pump-internal.PNG' | relative_url }}"
       alt="Gear Pump Internal Structure"
       loading="lazy">
  <figcaption>Figure 4.1-1: Internal structure of gear pump showing meshing gears and flow chambers for positive displacement operation

#### 4.1.2 Selected Specifications

Selected gear pump specifications:
- Model: ASEDA SAP-20 Series (20cc/rev)
- Theoretical flow: 29 L/min @ 1450rpm
- Actual flow: 20-25 L/min (85% volumetric efficiency)
- Maximum operating pressure: 50 bar
- Variable operation 800-1500rpm via inverter

### 4.2 Motor Selection

#### 4.2.1 Motor Specifications

- Type: 3-phase induction motor (Haizen Motor)
- Rated output: 3.7kW (5HP) × 2 units
- Poles: 4-pole (synchronous speed 1500rpm)
- Rated voltage: 380V
- Rated current: Approximately 7.3A

Rated current calculation:
```
I = P/(√3 × V × η × cosφ)
I = 3700/(1.732 × 380 × 0.9 × 0.85) ≈ 7.3A
```

#### 4.2.2 Shaft Coupling

Flexible couplings were used between pump and motor shafts to prevent bearing damage due to shaft misalignment.

### 4.3 Inverter Control System

#### 4.3.1 Inverter Selection: LS ELECTRIC General Purpose

Inverter application purposes:
- Precise speed control for catalyst solution flow regulation
- Soft start to prevent water hammer in catalyst injection lines
- Overcurrent, undervoltage, and overheating protection for reliable operation
- Immediate response to changing catalyst regeneration requirements

<figure>
  <img class="project-image"
       src="{{ '/project/dual-gear-pump-system/control-panel-interior.jpg' | relative_url }}"
       alt="Control Panel Interior"
       loading="lazy">
  <figcaption>Figure 4.2: Control panel interior showing dual inverters, MCCBs, ELBs, and wiring arrangement

#### 4.3.2 Key Parameter Settings

| Parameter | Setting Value | Purpose |
|-----------|---------------|---------|
| Motor rated voltage | 380V | Motor specification matching |
| Motor rated current | 7.3A | Protection setting reference |
| Base frequency | 60Hz | Motor rated point |
| Operating frequency range | 30-60Hz | Flow variable range |
| Acceleration time | 5-10 seconds | Water hammer prevention |
| Deceleration time | 5-10 seconds | Minimize piping shock |
| Maximum frequency | 70Hz | For short-term boost |
| Current limit | 110-120% | Overload protection |
| Carrier frequency | 4-8kHz | Heat/noise balance |

### 4.4 Instrumentation System

#### 4.4.1 Pressure Measurement

- Type: Analog Bourdon gauge
- Measurement range: 0-60 bar
- Accuracy: ±2% F.S.
- Installation location: Discharge manifold

#### 4.4.2 Flow Measurement

- Type: Mechanical flowmeter
- Measurement range: 5-50 L/min
- Accuracy: ±5% F.S.
- Installation location: Upstream of discharge branch

---

## 5. Electrical System Design

### 5.1 Power System Configuration

#### 5.1.1 Main Circuit Configuration

Power supply system:
```
AC 3φ 380V → MCCB → ELB → Inverter(2 units) → Motor(3.7kW×2)
```

Main protection devices:
- Molded Case Circuit Breaker (MCCB): 20A, 3-pole
- Earth Leakage Breaker (ELB): 15A, 30mA sensitivity
- Grounding system: Common PE busbar application

#### 5.1.2 Control Circuit

Control inputs:
- Emergency stop (E-Stop): Common stop for both inverters
- Run/Stop switches: Independent control for each pump
- Speed command: Potentiometer (0-10V) or keypad

### 5.2 Wiring Design

<figure>
  <img class="flowchart"
       src="{{ '/project/dual-gear-pump-system/wiring-diagram.png' | relative_url }}"
       alt="Electrical Wiring Diagram"
       loading="lazy">
  <figcaption>Figure 5.1: Complete electrical wiring diagram showing power distribution and control circuits

#### 5.2.1 Power Wiring

- Main power cable: 5×4mm² (R, S, T, N, PE)
- Motor cable: 4×2.5mm² (U, V, W, PE)
- Cable protection: Cable glands and flexible conduits

#### 5.2.2 Control Wiring

- Control power: AC 220V
- Signal wires: 0.75mm² shielded cable
- Analog signals: Twisted pair shielded wire

### 5.3 Control Panel Design

#### 5.3.1 Structural Design

- Material: Stainless steel (STS304)
- Protection rating: IP65 (waterproof, dustproof)
- Configuration: Upper inverter section, lower wiring section
- Cooling: Natural convection + ventilation fan

#### 5.3.2 Internal Layout

- Top: Two inverters arranged left and right
- Middle: MCCB, ELB, control relays
- Bottom: Terminal blocks, wire management

---

## 6. Mechanical Structure Design

### 6.1 Frame Design

#### 6.1.1 Material Selection

Reasons for using aluminum profiles:
- Weight reduction: Approximately 1/3 weight compared to steel
- Corrosion resistance: Excellent durability in industrial environments
- Machinability: Easy assembly with dedicated brackets
- Expandability: Modification and expansion possible with slot structure

Profiles used:
- Main structure: 4080 profile (high rigidity required areas)
- Sub-structure: 4040 profile (general connection areas)
- Slot: 8mm standard

#### 6.1.2 Structural Design

Load conditions:
- Motor + pump: 35kg × 2 = 70kg
- Control panel: 25kg
- Piping + accessories: 15kg
- Total weight: Approximately 110kg

Structural design performed for 220kg load with safety factor of 2.0.

#### 6.1.3 Vibration Countermeasures

Vibration reduction measures:
1. Motor base vibration mounts: Shore A60 rubber mounts
2. Lower reinforcement beams: Increase frame moment of inertia
3. Flexible hoses: Block piping vibration transmission

### 6.2 Mobility Design

#### 6.2.1 Caster System

- Front: Swivel casters × 2 (for direction change)
- Rear: Fixed casters × 2 (for straight-line stability)
- Wheel diameter: 100mm (floor condition adaptability)
- Load capacity: 100kg/each

#### 6.2.2 Handle Design

- Location: Front upper section
- Material: Stainless steel pipe
- Ergonomic height: 1000mm

---

## 7. Piping System Design

### 7.1 Suction Piping Design

#### 7.1.1 Basic Configuration

- Hose: 3/4", 25m length
- Strainer: 40 mesh, back-washable
- ball valve: Spring type, cracking pressure 0.1 bar

#### 7.1.2 Design Considerations

Suction piping design principles:
1. Minimize sharp direction changes
2. Upward slope to prevent air pocketing
3. Vacuum collapse prevention spring hose application
4. Ensure priming ease

### 7.2 Discharge Manifold Design

#### 7.2.1 Manifold Configuration

Discharge manifold sequence:
```
Pump discharge → Check valve → Pressure gauge tap → Flowmeter → Branch pipe(1/4"×2)
```

#### 7.2.2 Major Components

- Check valve: Swing type, SUS316 material
- Pressure gauge tap: PT 1/4" threaded connection
- Branch valves: Needle valves for flow control
- Quick couplers: Rapid gun attachment/detachment

### 7.3 Safety System

#### 7.3.1 Relief System

- Relief valve: Set pressure 50 bar
- Bypass line: Return to suction tank
- Relief flow: Over 10% of total flow

#### 7.3.2 Piping Material Selection

High-pressure section piping materials:
- Material: SUS316 (excellent corrosion resistance)
- Connection: PT threads + PTFE tape
- Hoses: High-pressure hydraulic hoses

---

## 8. Commissioning and Performance Verification

### 8.1 Initial Commissioning Procedure

#### 8.1.1 Priming Procedure

1. Suction line air purge
2. Manual pump housing filling
3. Foot valve leakage check
4. Initial rotation confirmation

#### 8.1.2 Safety Device Inspection

- Relief valve set pressure confirmation: 50 bar ±2 bar
- Emergency stop operation confirmation
- Earth leakage breaker operation test
- Overcurrent protection operation confirmation

### 8.2 Performance Test Results

#### 8.2.1 Performance Characteristics by Frequency

Test conditions:
- Test fluid: Water (ρ = 1000kg/m³)
- Discharge height: 50m
- Hose length: Discharge 50m × 2

| Frequency(Hz) | Speed(rpm) | Flow(L/min) | Discharge Pressure(bar) | Motor Current(A) |
|---------------|------------|-------------|------------------------|------------------|
| 30 | 900 | 12.3 | 38.2 | 4.2 |
| 40 | 1200 | 16.1 | 39.8 | 5.6 |
| 50 | 1500 | 19.8 | 41.2 | 6.8 |
| 60 | 1800 | 23.2 | 42.1 | 8.1 |

#### 8.2.2 Catalyst Injection Performance

- Injection pressure: 4.1 bar (102.5% of 4 bar target)
- Injection pattern: Uniform fan pattern for effective catalyst coverage
- Injection distance: 8-10m horizontal reach
- Injection angle: Approximately 30° fan for optimal catalyst distribution

### 8.3 Reliability Testing

#### 8.3.1 Continuous Operation Test

Test conditions:
- Test duration: 72 hours continuous
- Operating condition: 50Hz rated operation
- Monitoring: Temperature, vibration, leakage status

Test results:
- Motor temperature: Maximum 65℃ (allowable 80℃)
- Pump temperature: Maximum 58℃
- Leakage: None observed
- Noise: 68dB(A) @ 1m distance

#### 8.3.2 Safety Device Operation Confirmation

- Relief valve: Normal operation at 50.2 bar
- Earth leakage: Trip within 0.1 seconds at 30mA leakage
- Emergency stop: Immediate system shutdown confirmed

---

## 9. Results and Discussion

### 9.1 Achievement of Design Goals

#### 9.1.1 Performance Target Achievement

| Item | Target Value | Measured Value | Achievement Rate |
|------|--------------|----------------|------------------|
| Catalyst injection pressure | 4.0 bar | 4.1 bar | 102.5% |
| Total flow rate | 20 L/min | 19.8 L/min | 99.0% |
| Static head | 50m | 50m | 100% |
| Discharge distance | 50m | 50m | 100% |

All major performance indicators achieved their target values for effective catalyst regeneration.

#### 9.1.2 Additional Achievements

1. Enhanced system reliability through dual pump configuration for continuous catalyst treatment
2. Precise flow control capability via inverter control for optimal catalyst utilization
3. Field adaptability secured through portable design for various catalyst regeneration sites
4. Outdoor use capability with weatherproof control panel for industrial environments

### 9.2 Design Verification

#### 9.2.1 Theoretical vs. Measured Performance

Comparison of theoretical calculations and measured values:
- Required head calculation: 418.8m (43.1 bar)
- Measured discharge pressure: 41.2 bar @ 50Hz
- Error: -4.4% (compared to theoretical value)

Error analysis:
1. Actual volumetric efficiency higher than assumed (85%)
2. Conservative application of minor loss coefficients
3. Difference in actual fluid viscosity values

#### 9.2.2 NPSH Margin Confirmation

The calculated NPSHA = 7.24m was confirmed to operate stably without cavitation in actual operation.

### 9.3 Improvements

#### 9.3.1 Design Phase Improvements

1. Need for precise fluid property measurements
2. Build database of actual minor loss coefficients
3. Incorporate viscosity changes considering temperature effects

#### 9.3.2 Fabrication and Assembly Improvements

1. Strengthen piping supports to reduce vibration
2. Supplement heat countermeasures inside control panel
3. Improve accessibility for enhanced maintainability

---

## 10. Conclusion

This study successfully designed and fabricated a high-pressure spray cleaning system for catalyst regeneration facilities. The major achievements are as follows:

### 10.1 Technical Achievements

1. **Precise Hydraulic Analysis**: Accurate pump capacity calculation through Darcy-Weisbach equations and NPSH calculations.

2. **Optimized Pump Selection**: Implemented a system capable of high-pressure, positive displacement transport utilizing gear pump characteristics.

3. **Dual System Configuration**: Secured system reliability and operational flexibility through independent control of two pumps.

4. **Precision Control System**: Enabled flow and pressure adjustment according to field conditions through inverter control.

### 10.2 Practical Value

1. **Field Adaptability**: Portable frame and weatherproof control panel enable response to various field conditions.

2. **Maintainability**: Modular design and aluminum profile frame facilitate component replacement and maintenance.

3. **Economic Efficiency**: Reduced operating costs through improved energy efficiency and extended maintenance intervals compared to existing systems.

### 10.3 Technical Contribution

This study made the following technical contributions:

1. **Systematic Design Methodology**: Established systematic design process from theoretical calculations to verification.

2. **Dual Pump Control Technology**: Presented practical implementation methods for independently controllable redundant systems.

3. **Field-Optimized Design**: Developed practical design solutions satisfying actual industrial field requirements.

### 10.4 Future Improvement Directions

1. **Automation Expansion**: Establish automatic operation system through pressure and flow feedback control

2. **Enhanced Monitoring**: Add real-time performance monitoring and fault diagnosis functions

3. **Energy Optimization**: Develop algorithms for automatic optimal operating point setting according to operating conditions

This project represents a successful engineering case that comprehensively applied both theoretical analysis and practical design capabilities, providing practical value as reference for future development of similar high-pressure fluid transport systems.

---

## References

[1] Munson, B.R., Young, D.F., Okiishi, T.H., "Fundamentals of Fluid Mechanics", 5th Edition, Wiley, 2006.

[2] Karassik, I.J., Messina, J.P., Cooper, P., Heald, C.C., "Pump Handbook", 3rd Edition, McGraw-Hill, 2001.

[3] Stepanoff, A.J., "Centrifugal and Axial Flow Pumps", 2nd Edition, John Wiley & Sons, 1957.

[4] ASME, "Performance Test Code on Centrifugal Pumps", PTC 8.2, 1990.

[5] Hydraulic Institute, "Centrifugal Pump Design Guidelines", 2010.

[6] LS Electric, "Inverter User Manual", SV-iG5A Series, 2023.

[7] Korean Society of Mechanical Engineers, "Fluid Machinery Handbook", 2015.

---

## Appendix

### Appendix A. Major Calculation Formulas

#### A.1 Orifice Flow Formula
```
Q = Cd × A × √(2Δp/ρ)
Where: Q = Flow rate (m³/s), Cd = Discharge coefficient, A = Orifice area (m²)
       Δp = Pressure difference (Pa), ρ = Density (kg/m³)
```

#### A.2 Darcy-Weisbach Friction Loss Formula
```
hf = f × (L/D) × (v²/2g)
Where: hf = Friction loss head (m), f = Pipe friction factor, L = Pipe length (m)
       D = Pipe diameter (m), v = Average velocity (m/s), g = Gravitational acceleration (m/s²)
```

#### A.3 Reynolds Number
```
Re = (ρ × v × D) / μ = (v × D) / ν
Where: Re = Reynolds number, ρ = Density (kg/m³), v = Velocity (m/s)
       D = Pipe diameter (m), μ = Dynamic viscosity (Pa·s), ν = Kinematic viscosity (m²/s)
```

#### A.4 NPSH Available
```
NPSHA = Patm/(ρg) - Pv/(ρg) - hf,suction - hstatic
Where: NPSHA = Net positive suction head available (m), Patm = Atmospheric pressure (Pa), Pv = Vapor pressure (Pa)
       hf,suction = Suction pipe friction loss (m), hstatic = Suction static head (m)
```

### Appendix B. Major Component Specifications

#### B.1 Gear Pump Detailed Specifications

| Item | Specification | Remarks |
|------|---------------|---------|
| Model | ASEDA SAP-20 | |
| Displacement | 20 cc/rev | |
| Maximum pressure | 50 bar | |
| Maximum speed | 1800 rpm | |
| Suction port | 3/4" BSP | |
| Discharge port | 1/2" BSP | |
| Material | Cast iron + Stainless steel gears | |

#### B.2 Motor Detailed Specifications

| Item | Specification | Remarks |
|------|---------------|---------|
| Output | 3.7 kW (5 HP) | |
| Voltage | 380V | 3-phase |
| Current | 7.3A | Rated |
| Poles | 4-pole | |
| Speed | 1450 rpm | 50Hz basis |
| Insulation class | Class F | |
| Protection rating | IP55 | |

#### B.3 Inverter Detailed Specifications

| Item | Specification | Remarks |
|------|---------------|---------|
| Manufacturer | LS ELECTRIC | |
| Series | SV-iG5A | |
| Capacity | 5.5kW | |
| Input voltage | AC 380V, 3-phase | |
| Output voltage | 0-380V variable | |
| Output frequency | 0.1-400Hz | |
| Protection rating | IP20 | Installed in control panel |

### Appendix C. Commissioning Checklist

#### C.1 Pre-inspection Items

- [ ] Power supply status confirmation (voltage, phase sequence)
- [ ] Motor insulation resistance measurement (>1MΩ)
- [ ] Wiring connection status confirmation
- [ ] Ground connection status confirmation
- [ ] Pump rotation direction confirmation
- [ ] Suction line sealing status confirmation
- [ ] Discharge valve open status confirmation
- [ ] Safety device operation confirmation

#### C.2 Commissioning Procedure

1. **Priming**
   - Suction line water filling
   - Pump casing water filling
   - Air discharge through air vent opening

2. **Initial Start-up**
   - Low-speed start at 30Hz inverter
   - Discharge pressure and flow confirmation
   - Abnormal vibration and noise confirmation

3. **Performance Testing**
   - Performance measurement by frequency
   - Relief valve set pressure confirmation
   - Safety device operation confirmation

4. **Final Inspection**
   - Leakage inspection at all points
   - Temperature rise confirmation
   - Operating data recording

### Appendix D. Maintenance Guidelines

#### D.1 Daily Inspection Items

- Pre-operation visual inspection (leakage, abnormal sounds)
- Pressure gauge and flowmeter reading confirmation
- Motor temperature check (hand touch for overheating)
- Control panel internal temperature confirmation

#### D.2 Periodic Inspection Items

**Weekly Inspection**
- Pump bearing grease condition confirmation
- Coupling alignment status inspection
- Piping support fixture status confirmation

**Monthly Inspection**
- Motor insulation resistance measurement
- Inverter cooling fan cleaning
- Relief valve operation test

**Annual Inspection**
- Pump disassembly inspection
- Motor bearing replacement
- Inverter internal cleaning and inspection
- Piping internal cleaning

### Appendix E. Operation Manual

#### E.1 Start-up Procedure

1. Power on and initial inspection
2. Suction valve fully open
3. Discharge valve 1/4 open (initial load reduction)
4. Start with inverter at 30Hz
5. Increase to target frequency after normal operation confirmation
6. Discharge valve fully open

#### E.2 Shutdown Procedure

1. Gradually reduce inverter frequency
2. Confirm pump complete stop
3. Discharge valve fully closed
4. Power off

#### E.3 Emergency Stop

- Immediate entire system stop when emergency stop button activated
- Restart after cause identification
- No restart without safety confirmation

---

**Keywords**: Gear Pump, Dual System, High Pressure Transport, Inverter Control, Fluid Dynamics Analysis, NPSH Calculation, Modular Design, Industrial Automation, Catalyst Regeneration, Spray Cleaning