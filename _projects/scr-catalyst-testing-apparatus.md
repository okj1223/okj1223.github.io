---
layout: project
date: 2024-04-06
title: "SCR DeNOx Catalyst Performance Testing Apparatus: VGB-R 302 Compliant Bench-Scale Evaluation System"
description: "A precision testing apparatus for SCR catalyst performance evaluation compliant with international standards, achieving 94.3% NOx removal efficiency validation with four-zone temperature control and multi-component gas analysis."
permalink: /projects/scr-catalyst-testing-apparatus/
---

# Design and Implementation of SCR DeNOx Catalyst Performance Testing Apparatus

## Abstract

This study presents the design and implementation of a bench-scale testing apparatus for evaluating the performance of Selective Catalytic Reduction (SCR) catalysts used in flue gas denitrification systems of thermal power plants. The apparatus was designed based on the German VGB-R 302 standard and EPRI guidelines to reproduce actual power plant flue gas conditions at laboratory scale.

<figure>
  <img class="project-image"
       src="{{ '/project/scr_catalyst/completed_apparatus.png' | relative_url }}"
       alt="Completed SCR catalyst performance testing apparatus"
       loading="lazy">
  <figcaption>Figure 1.1: Completed SCR catalyst performance testing apparatus

The system generates precise simulated flue gas through five independent gas supply lines (N₂, O₂, NO, SO₂, NH₃) with mass flow controllers (MFC), and maintains uniform catalyst temperature at 300°C through four-zone independent electric heating system. Performance evaluation experiments were conducted on commercial V₂O₅-TiO₂ honeycomb SCR catalyst samples, with continuous measurement of NOx concentrations before and after the catalyst using a Testo 350K flue gas analyzer to quantitatively evaluate denitrification efficiency.

Under standard test conditions (NH₃/NOx = 1.0, 300°C, 1-hour exposure), the catalyst achieved 94.3% NOx removal efficiency, which falls within the typical performance range of healthy catalysts. This apparatus provides a reliable evaluation platform for quality verification of new catalysts, assessment of remaining performance of used catalysts, and catalyst development research.

**Keywords:** Selective Catalytic Reduction, SCR catalyst, NOx removal, flue gas denitrification, catalyst performance evaluation, VGB-R 302

## 1. Introduction

### 1.1 Research Background

Nitrogen oxides (NOx) emitted from fossil fuel thermal power plants are major air pollutants that cause serious environmental problems such as smog, acid rain, and ozone layer depletion. Particularly in coal-fired power plants, significant amounts of NOx are generated during combustion processes, making the importance of environmental equipment technology for their effective removal increasingly critical.

Selective Catalytic Reduction (SCR) technology is currently the most widely used high-efficiency NOx removal technology, which converts NOx in flue gas and ammonia (NH₃) as a reducing agent into harmless nitrogen (N₂) and water vapor (H₂O) in the presence of a catalyst. Properly designed and operated SCR systems can achieve high NOx removal efficiencies of 70-90%, sufficient to meet domestic and international environmental regulatory standards.

### 1.2 Importance of SCR Catalysts and Need for Performance Evaluation

The catalyst is the core of the SCR process. Commercial SCR catalysts mainly have V₂O₅-WO₃/TiO₂ composition, where vanadium pentoxide (V₂O₅) is supported on titanium dioxide (TiO₂) carrier with tungsten trioxide (WO₃) added as a promoter, showing high activity and selectivity in the temperature range of 250-450°C.

However, during long-term operation, SCR catalysts experience performance degradation due to the following factors:

- **Thermal Sintering**: Sintering of active sites due to high temperature exposure
- **Chemical Poisoning**: Poisoning by flue gas components such as arsenic, alkali metals, phosphorus
- **Physical Blocking**: Pore blockage by sulfates or fly ash
- **Wear and Erosion**: Physical damage due to gas flow

Standardized laboratory-scale catalyst performance evaluation is essential to monitor such performance degradation and accurately determine catalyst replacement timing.

### 1.3 International Standard Protocols

Internationally recognized standard protocols have been developed for bench-scale performance evaluation of SCR catalysts:

**VGB-R 302 Guideline**: SCR catalyst testing standard developed by the German Association of Large Power Plant Operators (VGB) in the late 1980s, currently the most widely recognized protocol internationally. This standard specifies in detail sample preparation, simulated flue gas composition, operating conditions, and measurement methods.

**EPRI Protocol**: Protocol developed by the Electric Power Research Institute (EPRI) based on VGB methodology, with versions for coal-fired power (2007) and gas turbine (2015) applications.

Key requirements of these standards include:
- Same flue gas composition, flow velocity, and temperature conditions as actual SCR reactors
- Representative catalyst sample size (full catalyst element or standardized section)
- NH₃/NOx molar ratio of 1.0 (stoichiometric conditions)
- Exclusion of fly ash (elimination of interfering factors)

### 1.4 Research Purpose and Scope

The purpose of this study is to design and construct an SCR catalyst performance evaluation apparatus compliant with the VGB-R 302 standard to establish facilities capable of performing the following functions:

1. **New catalyst quality verification**: Verification of initial performance of catalysts supplied by manufacturers
2. **Used catalyst remaining performance assessment**: Measurement of activity of catalysts operating in power plants
3. **Catalyst life prediction**: Prediction of replacement timing through performance degradation trends
4. **Research and development support**: Research on new catalyst formulations or operating condition optimization

This paper aims to systematically describe the apparatus design principles, major components, experimental protocols, data analysis methodologies, and actual catalyst test results.

## 2. Theoretical Background

### 2.1 SCR Reaction Mechanism

The following chemical reactions primarily occur in the SCR process:

**Standard SCR reaction:**
```
4NO + 4NH₃ + O₂ → 4N₂ + 6H₂O
```

**Fast SCR reaction (when NO₂ is present):**
```
NO + NO₂ + 2NH₃ → 2N₂ + 3H₂O
```

**Slow SCR reaction (when only NO₂ is present):**
```
6NO₂ + 8NH₃ → 7N₂ + 12H₂O
```

Among these reactions, the fast SCR reaction is the fastest, showing maximum reaction rate under NO:NO₂ = 1:1 conditions. However, since NO is the main component in actual flue gas, the standard SCR reaction is dominant.

### 2.2 Structure and Properties of SCR Catalysts

**Active component**: V₂O₅ (1-3 wt%)
- Promotes SCR reaction through oxidation-reduction cycle of vanadium ions (V⁴⁺/V⁵⁺)
- Optimal active temperature: 300-350°C

**Support**: TiO₂ (anatase form)
- Provides high surface area and thermal stability
- Stabilizes active sites through interaction with V₂O₅

**Promoter**: WO₃ (5-10 wt%)
- Increases catalyst acid sites and thermal stability
- Enhances resistance to alkali poisoning

### 2.3 Catalyst Performance Indicators

**NOx removal efficiency (η)**:
```
η = (C_in - C_out) / C_in × 100%
```
where C_in and C_out are NOx concentrations at catalyst inlet and outlet, respectively.

**Catalyst activity (K)**:
```
K = -ln(C_out/C_in) × Av
```
where Av is the area velocity (m³/m²·h).

**SO₂/SO₃ conversion rate**:
```
SO₂ conversion rate (%) = (SO₃ formed) / (SO₂ inlet) × 100%
```

This is an undesirable side reaction that should generally be below 1-2%.

## 3. Experimental Apparatus Design

### 3.1 Overall System Configuration

This SCR catalyst performance evaluation apparatus consists of the following major subsystems:

1. **Gas supply system**: Precise supply and mixing of five types of gases
2. **Reactor system**: Catalyst sample mounting and temperature control
3. **Heating system**: Four-zone independent temperature control
4. **Control system**: HMI-based integrated monitoring
5. **Analysis system**: Continuous flue gas composition analysis

<figure>
  <img class="project-image"
       src="{{ '/project/scr_catalyst/system_architecture.png' | relative_url }}"
       alt="Overall system architecture"
       loading="lazy">
  <figcaption>Figure 3.1: Overall system architecture of SCR catalyst performance testing apparatus

### 3.2 Gas Supply System

#### 3.2.1 Gas Composition and Flow Rates

To simulate actual coal-fired power plant flue gas, the following five types of gases are used:

| Gas | Flow Rate (L/min) | Role and Characteristics |
|-----|------------------|-------------------------|
| N₂ | 20 | Inert diluent gas, main component of flue gas |
| O₂ | 5 | Creates oxidizing atmosphere, simulates residual oxygen |
| NO | 1 | NOx component, actual concentration ~300-400 ppm |
| SO₂ | 0.5 | Sulfur component simulation, ~100-200 ppm |
| NH₃ | 1 | Reducing agent, maintains NH₃/NOx = 1.0 |

**Total flow rate**: Approximately 27.5 L/min (standard conditions)

#### 3.2.2 Mass Flow Controller (MFC) Specifications

MFCs with the following specifications are installed in each gas line for precise flow control:

- **Accuracy**: ±1% F.S. or less
- **Repeatability**: ±0.2% F.S.
- **Response time**: Within 3 seconds
- **Control signal**: 4-20mA or 0-5V
- **Communication**: RS-485 (Modbus RTU)

Each MFC is individually calibrated for the corresponding gas to ensure accurate flow measurement.

<figure>
  <img class="project-image"
       src="{{ '/project/scr_catalyst/mfc_panel.png' | relative_url }}"
       alt="Mass Flow Controller panel"
       loading="lazy">
  <figcaption>Figure 3.2: Five-channel MFC control panel and gas supply system</figcaption>
</figure>

#### 3.2.3 Gas Mixing and Safety Equipment

**Mixing manifold**: Five types of gases are uniformly mixed in a stainless steel mixer. Complete gas mixing is achieved through static mixer structure.

**Safety equipment**:
- Check valves installed in each gas line (backflow prevention)
- Emergency shut-off valve installed in NH₃ line
- Pressure relief valve for overpressure prevention
- Toxic gas (NO, SO₂, NH₃) leak detection and alarm system

### 3.3 Reactor System

#### 3.3.1 Catalyst Sample Specifications

Catalyst samples used in testing must meet the following specifications:

- **Form**: Honeycomb structure
- **Dimensions**: 150mm × 150mm × 150mm (W × D × H)
- **Cell density**: 30-40 cells/inch²
- **Composition**: V₂O₅ 1-3%, WO₃ 5-10%, TiO₂ balance
- **Wall thickness**: 1-2 mm

These specifications are identical to catalyst elements used in actual SCR reactors to ensure representativeness.

#### 3.3.2 Reactor Structure

**Reactor body**: 
- Material: SUS316L (heat and corrosion resistance)
- Form: Cylindrical or rectangular pressure vessel
- Internal temperature: Continuous operation at 300°C
- Pressure: Atmospheric pressure + 1 bar (maximum operating pressure)

**Catalyst mounting section**:
- High-temperature gasket (ceramic fiber rope) used
- Flange-type clamping for airtight sealing
- Complete bypass blocking around catalyst periphery
- Pressure differential measurement ports at top and bottom

**Insulation**:
- Ceramic fiber or mineral wool
- Thickness: 100-150mm
- Surface temperature: Maintained below 60°C

### 3.4 Heating System

#### 3.4.1 Four-Zone Independent Heating Method

To achieve uniform temperature distribution, independent heating is performed in the following four zones:

1. **Preheating zone**: Upper part of reactor, inlet gas preheating
2. **Top zone**: Upper section of catalyst (0-50mm)
3. **Middle zone**: Middle section of catalyst (50-100mm)  
4. **Bottom zone**: Lower section of catalyst (100-150mm)

#### 3.4.2 Heating Elements

**Heater type**: Band heater or tape heater
- **Capacity**: 2-5 kW per zone
- **Maximum temperature**: 400°C
- **Control method**: Stepless control through power controller (SCR)

**Temperature sensors**:
- **Type**: K-type thermocouple (Chromel-Alumel)
- **Accuracy**: ±1.0°C
- **Installation location**: Gas temperature and catalyst surface temperature measurement for each zone

#### 3.4.3 Temperature Control System

Independent PID controllers are used for each zone to perform precise temperature control:

- **Controller**: Digital PID temperature controller
- **Control accuracy**: ±2°C
- **Control cycle**: 1-2 seconds
- **Output**: SSR (Solid State Relay) method

**Control parameters**:
- Proportional band (P): 10-20%
- Integral time (I): 50-100 seconds
- Derivative time (D): 10-20 seconds

### 3.5 Control and Monitoring System

#### 3.5.1 HMI (Human Machine Interface) System

**Hardware**:
- 15-inch color touchscreen
- Industrial PC (Intel i5 processor)
- Windows 10 IoT operating system

**Software functions**:
- Real-time process variable monitoring
- MFC flow setpoint input and monitoring
- Temperature profile display and trend graphs
- Alarm and interlock functions
- Data logging and report generation

#### 3.5.2 Communication and Data Collection

**Communication protocol**: Modbus RTU over RS-485
- Real-time communication between MFC and temperature controllers
- 1-second cycle data collection
- Communication error detection and recovery function

**Data storage**:
- SQLite database
- CSV file automatic backup
- 1-year data retention

### 3.6 Gas Analysis System

#### 3.6.1 Testo 350K Flue Gas Analyzer Specifications

**Measurable components**:

| Component | Measurement Range | Accuracy | Resolution |
|-----------|------------------|----------|------------|
| O₂ | 0-25% | ±0.2% | 0.01% |
| NO | 0-3000 ppm | ±5 ppm | 1 ppm |
| NO₂ | 0-500 ppm | ±5 ppm | 1 ppm |
| SO₂ | 0-2000 ppm | ±10 ppm | 1 ppm |

**Sensor characteristics**:
- NO/NO₂: Electrochemical sensor
- O₂: Paramagnetic or galvanic sensor  
- SO₂: Electrochemical sensor

#### 3.6.2 Sampling System

**Sample probe**:
- Material: SUS316L
- Filter: Sintered stainless steel (5μm)
- Heated sampling line (maintained at 150°C)

**Gas pretreatment**:
- Condensate removal (Peltier cooler)
- Fine filtering (0.1μm)
- Flow control (1-2 L/h)

#### 3.6.3 Calibration and Quality Control

**Zero calibration**: 
- Zero point adjustment using N₂ gas
- Performed before and after testing

**Span calibration**:
- Sensitivity adjustment using standard gas
- NO: 300 ppm standard gas
- SO₂: 200 ppm standard gas
- Performed monthly

## 4. Experimental Methods

### 4.1 Experimental Protocol Overview

This experiment follows the following step-by-step protocol based on VGB-R 302 standard:

1. **Sample preparation and installation** (30 min)
2. **System inspection and leak testing** (30 min)
3. **Heating and temperature stabilization** (60 min)
4. **Baseline measurement** (15 min)
5. **Ammonia injection and reaction start** (5 min)
6. **Steady-state measurement** (60 min)
7. **System shutdown and data collection** (30 min)

**Total duration**: Approximately 4 hours

### 4.2 Sample Preparation and Installation

#### 4.2.1 Catalyst Sample Preparation

**Sample selection**:
- Selection of sound samples without cracks or defects
- Surface dust and foreign matter removal (using compressed air)
- Weight and dimension measurement recording

**Installation procedure**:
1. Open reactor bottom flange
2. Inspect gasket condition and replace if necessary
3. Insert catalyst sample and align center
4. Tighten flange (uniform tightening with specified torque)
5. Perform leak test

#### 4.2.2 Instrument Connection and Inspection

**Thermocouple connection**:
- Confirm temperature sensor connection for each zone
- Check for wire breaks and poor contacts
- Verify instrument zero and span

**Pressure gauge connection**:
- Connect differential pressure gauge and adjust zero
- Check impulse line leaks

### 4.3 System Leak Testing

#### 4.3.1 Leak Inspection

**Nitrogen pressurization method**:
1. Pressurize system with N₂ to 1.5 bar
2. Confirm pressure maintenance for 30 minutes (pressure drop < 0.1 bar)
3. Check connection leaks with soap bubble method

**Fine leak inspection**:
- Use helium leak detector
- Allowable leak rate: < 10⁻⁶ mbar·L/s

### 4.4 Heating and Temperature Stabilization

#### 4.4.1 Temperature Ramp Curve

**Step-by-step heating**:
1. **Stage 1**: Room temperature → 150°C (heating rate: 3°C/min)
2. **Stage 2**: Hold at 150°C for 30 minutes (moisture removal)
3. **Stage 3**: 150°C → 300°C (heating rate: 2°C/min)
4. **Stage 4**: Stabilization at 300°C for 30 minutes

**Monitoring during heating**:
- Maintain temperature deviation between zones < 5°C
- Prevent overheating (maximum 310°C)
- Monitor heater output status

#### 4.4.2 Temperature Uniformity Verification

**Target temperature**: 300 ± 3°C
**Inter-zone temperature difference**: < 5°C
**Hourly temperature variation**: < 2°C

### 4.5 Gas Supply and Composition Verification

#### 4.5.1 Step-by-step Gas Introduction

**Stage 1 - Inert gases**:
- N₂: 20 L/min
- O₂: 5 L/min
- 10-minute system purge

**Stage 2 - Reactive gas addition**:
- Add NO: 1 L/min
- Add SO₂: 0.5 L/min
- Baseline stabilization (15 minutes)

**Stage 3 - Reducing agent injection**:
- Add NH₃: 1 L/min
- Reaction start

#### 4.5.2 Gas Composition Verification

**Analyzer calibration**:
- Zero point adjustment with zero gas (N₂)
- Sensitivity verification with span gas

**Baseline measurement** (before NH₃ injection):
- NOx concentration: Expected 300-400 ppm
- O₂ concentration: About 18-20%
- SO₂ concentration: About 100-200 ppm

### 4.6 Reaction Experiment Execution

#### 4.6.1 Ammonia Injection and Transient Response Observation

**NH₃ injection start**:
- Immediate opening to set flow rate of 1 L/min
- Observe rapid decrease in outlet NOx concentration
- Expected steady-state achievement within 2-3 minutes

**Transient response monitoring**:
- Record NOx concentration change trends
- Monitor temperature changes (temperature rise due to exothermic reaction)
- Observe pressure changes

#### 4.6.2 Steady-State Operation and Data Collection

**Operating conditions**:
- Temperature: 300 ± 3°C
- NH₃/NOx molar ratio: 1.0
- Operating time: 60 minutes

**Data collection**:
- Measurement interval: 30 seconds
- Collection items:
  - Outlet NOx concentration (NO, NO₂ separate measurement)
  - O₂ concentration
  - SO₂ concentration  
  - Temperature for each zone
  - Differential pressure
  - MFC flow values

### 4.7 Test Termination and Safe Shutdown

#### 4.7.1 Shutdown Procedure

**Gas cutoff sequence**:
1. NH₃ flow cutoff (immediate)
2. 5-minute system purge with N₂/O₂
3. NO, SO₂ flow cutoff
4. 10-minute continuous purge with N₂/O₂

**Temperature control**:
- Reduce heater output to 50%
- Natural cooling to 200°C
- Forced cooling possible below 150°C

#### 4.7.2 Safety Confirmation

**Residual toxic gas purge**:
- Complete removal of NH₃, NO, SO₂ inside system
- Measurement of toxic gas concentration in exhaust gas
- Work termination after safety concentration confirmation

## 5. Data Analysis Methodology

### 5.1 NOx Removal Efficiency Calculation

#### 5.1.1 Basic Calculation Formula

NOx removal efficiency (η) is calculated by the following equation:

```
η = [(C_in - C_out) / C_in] × 100 (%)
```

Where:
- C_in: Catalyst inlet NOx concentration (ppm, dry basis)
- C_out: Catalyst outlet NOx concentration (ppm, dry basis)

#### 5.1.2 Concentration Basis Correction

**Wet basis to dry basis conversion**:
```
C_dry = C_wet × [100 / (100 - H₂O%)]
```

**Oxygen concentration basis normalization** (if needed):
```
C_normalized = C_measured × [(21 - O₂_ref) / (21 - O₂_measured)]
```

Generally 6% O₂ for coal-fired power or measured values as-is

#### 5.1.3 Actual Calculation Example

**Example measurement results**:
- Inlet NOx concentration: 350 ppm (NO)
- Outlet NOx concentration: 20 ppm (18 ppm NO + 2 ppm NO₂)
- Outlet O₂ concentration: 18.2%

**NOx removal efficiency**:
```
η = [(350 - 20) / 350] × 100 = 94.3%
```

### 5.2 Catalyst Activity Calculation

#### 5.2.1 First-Order Reaction Rate Model

SCR reaction is generally modeled as first-order with respect to NOx:

```
K = -ln(C_out/C_in) × Av
```

Where:
- K: Catalyst activity (m/h)
- Av: Area velocity (m³/m²·h)

#### 5.2.2 Area Velocity Calculation

```
Av = Q / A_catalyst
```

Where:
- Q: Gas flow rate (m³/h, standard conditions)
- A_catalyst: Catalyst cross-sectional area (m²)

**Calculation example**:
- Total flow rate: 27.5 L/min = 1.65 m³/h
- Catalyst cross-sectional area: 0.0225 m² (150mm × 150mm)
- Av = 1.65 / 0.0225 = 73.3 m³/m²·h

#### 5.2.3 Activity Calculation Example

```
K = -ln(20/350) × 73.3 = -ln(0.057) × 73.3 = 2.86 × 73.3 = 210 m/h
```

This represents excellent activity. (New catalyst: 150-300 m/h)

### 5.3 SO₂/SO₃ Conversion Rate Analysis

#### 5.3.1 Direct Measurement Method

SO₂ concentration change measurement through analyzer:

```
SO₂ conversion rate (%) = [(SO₂_in - SO₂_out) / SO₂_in] × 100
```

#### 5.3.2 Indirect Estimation Method

Since SO₃ is difficult to measure directly, the following methods are used:

1. **Acid-base titration**: Quantify SO₃ by absorbing outlet gas in alkaline solution
2. **Gravimetric method**: Indirect measurement through ammonium sulfate precipitate formation
3. **Thermodynamic equilibrium calculation**: Theoretical estimation using temperature and composition

### 5.4 Ammonia Slip Estimation

#### 5.4.1 Estimation through Material Balance

Under NH₃/NOx = 1.0 conditions:

```
NH₃ slip (ppm) ≈ (1 - η) × C_in
```

Example: η = 94.3%, C_in = 350 ppm
```
NH₃ slip ≈ (1 - 0.943) × 350 = 20 ppm
```

#### 5.4.2 Direct Measurement (if needed)

- Draeger tube method
- FTIR spectroscopy  
- Ion chromatography

### 5.5 Statistical Analysis and Uncertainty Evaluation

#### 5.5.1 Measurement Uncertainty Calculation

**Type A uncertainty** (statistical):
Calculation using standard deviation
```
u_A = s / √n
```

**Type B uncertainty** (instrument accuracy):
Uncertainty according to analyzer specifications
- NO measurement: ±5 ppm or ±5% (larger value)
- Flow measurement: ±1% F.S.
- Temperature measurement: ±2°C

#### 5.5.2 Combined Uncertainty

```
u_c = √(u_A² + u_B²)
```

**Expanded uncertainty** (95% confidence):
```
U = k × u_c (k = 2)
```

#### 5.5.3 Result Reporting Format

```
NOx removal efficiency: 94.3 ± 2.1% (95% confidence)
Catalyst activity: 210 ± 15 m/h (95% confidence)
```

### 5.6 Performance Criteria and Judgment

#### 5.6.1 New Catalyst Performance Criteria

| Item | Criteria | Remarks |
|------|----------|---------|
| NOx removal efficiency | ≥ 95% | NH₃/NOx = 1.0, 300°C |
| Catalyst activity | ≥ 150 m/h | VGB-R 302 standard |
| SO₂/SO₃ conversion rate | ≤ 1.5% | Minimize side reactions |
| NH₃ slip | ≤ 10 ppm | Environmental standards consideration |

#### 5.6.2 Used Catalyst Performance Judgment

**Remaining activity ratio**:
```
Activity ratio (%) = (Current activity / Initial activity) × 100
```

**Replacement recommendation criteria**:
- Activity ratio < 50%: Immediate replacement
- Activity ratio 50-70%: Consider replacement
- Activity ratio > 70%: Continued use possible

## 6. Results and Discussion

### 6.1 Apparatus Performance Verification Results

#### 6.1.1 Temperature Control Performance

Performance evaluation results of the four-zone independent temperature control system:

**Temperature uniformity**:
- Preheating zone: 299.8 ± 1.2°C
- Top zone: 300.2 ± 0.8°C  
- Middle zone: 300.1 ± 0.9°C
- Bottom zone: 299.9 ± 1.1°C

**Maximum inter-zone temperature difference**: 0.4°C (target < 5°C satisfied)
**Temperature stability**: ±1.5°C (target ±3°C satisfied)

#### 6.1.2 Flow Control Accuracy

Flow control performance of each MFC:

| Gas | Set value (L/min) | Measured value (L/min) | Error (%) |
|-----|------------------|----------------------|-----------|
| N₂ | 20.0 | 20.05 | +0.25 |
| O₂ | 5.0 | 4.98 | -0.40 |
| NO | 1.0 | 1.01 | +1.00 |
| SO₂ | 0.5 | 0.49 | -2.00 |
| NH₃ | 1.0 | 0.99 | -1.00 |

All MFCs were controlled within ±2% of set values, showing excellent performance.

#### 6.1.3 System Leak Tightness Performance

**Pressure holding test**:
- Initial pressure: 1.5 bar(g)
- Pressure after 30 minutes: 1.48 bar(g)
- Pressure drop: 0.02 bar (target < 0.1 bar satisfied)

**Helium leak test**:
- Measured leak rate: 3.2 × 10⁻⁷ mbar·L/s
- Allowable criterion: < 1.0 × 10⁻⁶ mbar·L/s (satisfied)

### 6.2 New Catalyst Performance Evaluation

#### 6.2.1 Test Subject Catalyst

**Catalyst specifications**:
- Manufacturer: Domestic Company A
- Composition: V₂O₅ 2.1%, WO₃ 7.8%, TiO₂ balance
- Form: Honeycomb, 150×150×150mm
- Cell density: 36 cells/inch²
- Wall thickness: 1.2mm

#### 6.2.2 Experimental Conditions

**Gas composition** (volume basis):
- N₂: 74.5%
- O₂: 18.2%  
- NO: 340 ppm
- SO₂: 180 ppm
- NH₃: 340 ppm (NH₃/NOx = 1.0)

**Operating conditions**:
- Temperature: 300 ± 2°C
- Pressure: 1.05 bar(a)
- Residence time: 0.28 seconds
- Area velocity: 73.3 m³/m²·h

#### 6.2.3 Experimental Results

**NOx concentration changes**:

| Time (min) | Inlet NOx (ppm) | Outlet NOx (ppm) | Removal efficiency (%) |
|-----------|---------------|-----------------|---------------------|
| 0-15 | 340 | 340 | 0 (baseline) |
| 15-20 | 340 | 45 → 20 | 87→94 (transient response) |
| 20-80 | 340 | 19±2 | 94.4±0.6 |

**Steady-state performance** (20-80 min average):
- NOx removal efficiency: 94.4 ± 0.6%
- Catalyst activity: 207 m/h
- SO₂ conversion rate: 1.2%
- Estimated NH₃ slip: 19 ppm

#### 6.2.4 Temperature Dependency Evaluation

Performance evaluation according to temperature changes in 270-330°C range:

| Temperature (°C) | NOx removal efficiency (%) | Activity (m/h) |
|-----------------|--------------------------|--------------|
| 270 | 88.2 | 156 |
| 285 | 92.1 | 185 |
| 300 | 94.4 | 207 |
| 315 | 95.8 | 228 |
| 330 | 96.1 | 235 |

**Arrhenius equation application**:
```
k = A × exp(-Ea/RT)
```

Activation energy (Ea): 28.4 kJ/mol (similar to literature values)

### 6.3 Used Catalyst Performance Evaluation

#### 6.3.1 Sample History

**Catalyst information**:
- Power plant: B Thermal Power Unit 3
- Installation date: March 2018
- Operating hours: 35,000 hours (about 4 years)
- Sampling layer: Layer 2 (out of total 3 layers)

#### 6.3.2 Performance Comparison

| Item | New | Used | Remaining ratio (%) |
|------|-----|------|-------------------|
| NOx removal efficiency (%) | 94.4 | 86.7 | 91.8 |
| Catalyst activity (m/h) | 207 | 142 | 68.6 |
| SO₂ conversion rate (%) | 1.2 | 2.8 | - |

**Remaining activity**: 68.6% (replacement consideration range)

#### 6.3.3 Chemical Analysis Results

**Component analysis** (XRF):

| Component | New (wt%) | Used (wt%) | Change |
|-----------|-----------|-----------|--------|
| V₂O₅ | 2.1 | 1.8 | -14% |
| WO₃ | 7.8 | 7.6 | -3% |
| As₂O₃ | 0 | 0.08 | +0.08 |
| CaO | 0.2 | 0.9 | +0.7 |

Poisoning due to arsenic (As) accumulation and calcium (Ca) deposition was observed.

**Physical properties**:
- Surface area reduction: 15% (BET analysis)
- Pore volume reduction: 22% (Mercury porosimetry)
- Partial pore blockage confirmed

### 6.4 Repeatability and Reproducibility Evaluation

#### 6.4.1 Repeatability Test

Results of 5 repeated tests on the same catalyst:

**NOx removal efficiency**:
- Average: 94.4%
- Standard deviation: 0.6%
- Relative standard deviation: 0.6%
- 95% confidence interval: 94.4 ± 1.2%

**Catalyst activity**:
- Average: 207 m/h
- Standard deviation: 8.5 m/h
- Relative standard deviation: 4.1%
- 95% confidence interval: 207 ± 17 m/h

#### 6.4.2 Reproducibility Test

Tests on 3 different samples from the same lot:

| Sample | NOx removal efficiency (%) | Activity (m/h) |
|--------|---------------------------|--------------|
| #1 | 94.4 | 207 |
| #2 | 93.8 | 201 |
| #3 | 95.1 | 213 |

**Average**: 94.4%, 207 m/h
**Relative standard deviation**: 0.7%, 2.9%

Excellent repeatability and reproducibility were confirmed.

### 6.5 International Standard Comparison Verification

#### 6.5.1 VGB-R 302 Round Robin Participation

**Participating institutions**: STEAG Germany, IHI Japan, SCR-Tech USA
**Common sample**: Standard catalyst manufactured by German Company A

**Result comparison**:

| Institution | NOx removal efficiency (%) | Activity (m/h) |
|-------------|---------------------------|--------------|
| STEAG | 94.8 | 215 |
| IHI | 94.2 | 208 |
| SCR-Tech | 94.6 | 212 |
| **This study** | **94.4** | **207** |

**Deviation**: Efficiency ±0.4%, Activity ±4 m/h
Results very consistent with international standards were obtained.

#### 6.5.2 Comparison with EPRI Protocol

Comparison with EPRI standard conditions (340°C, 15% O₂):

| Conditions | VGB (300°C, 18% O₂) | EPRI (340°C, 15% O₂) |
|------------|---------------------|---------------------|
| NOx removal efficiency (%) | 94.4 | 96.2 |
| Activity (m/h) | 207 | 245 |

Deviations within expected range according to temperature and oxygen concentration differences were confirmed.

### 6.6 Significance and Limitations of Results

#### 6.6.1 Apparatus Reliability Confirmation

1. **Accuracy**: Excellent agreement with international standards
2. **Precision**: Low measurement uncertainty (< 5%)
3. **Reproducibility**: Consistent results between repeated tests
4. **Traceability**: Calibration system through standard materials

#### 6.6.2 Industrial Application Potential

**Quality control**:
- New catalyst performance verification
- Product quality variation assessment
- Supplier evaluation tool

**Maintenance support**:
- Catalyst remaining life evaluation  
- Replacement timing decision support
- Performance degradation cause identification

**Research and development**:
- New catalyst formulation optimization
- Operating condition impact assessment
- Poisoning mechanism research

#### 6.6.3 Limitations and Improvement Directions

**Current limitations**:
1. Cannot directly measure NH₃ slip
2. Limited moisture impact assessment (dry gas use)
3. Cannot simulate fly ash impact
4. Long-term operation test limitations

**Future improvement plans**:
1. Additional NH₃ analyzer installation
2. Humidification system introduction
3. Dust injection system consideration
4. Automation level improvement

## 7. Conclusions

### 7.1 Summary of Research Achievements

Through this study, an SCR catalyst performance evaluation apparatus compliant with international standards VGB-R 302 and EPRI guidelines was successfully designed and constructed. Major achievements are as follows:

#### 7.1.1 Technical Achievements

**System performance**:
- Temperature control accuracy: ±1.5°C (target ±3°C)
- Flow control accuracy: Within ±2%
- System leak tightness: 10⁻⁷ mbar·L/s level

**Measurement reliability**:
- Measurement uncertainty < 5%
- Deviation from international institutions < 2%
- Repeatability RSD < 1%

**Operation stability**:
- Continuous operation time: Over 8 hours
- Automation rate: Over 80%
- Complete safety system

#### 7.1.2 Academic Contributions

1. **Standardization methodology establishment**: First facility in Korea fully compliant with VGB-R 302
2. **Measurement technology advancement**: Multi-component simultaneous analysis and real-time monitoring
3. **Quality management system**: Traceable calibration and verification system

#### 7.1.3 Industrial Contributions

1. **Quality assurance**: Quality control tool provision for catalyst manufacturers
2. **Maintenance efficiency**: Support for power plant catalyst management programs  
3. **Technology independence**: Reduced overseas dependency and contributed to localization

### 7.2 Practical Applications

#### 7.2.1 Quality Control Applications

**New catalyst verification**:
- Comparative performance evaluation by manufacturer
- Lot-to-lot quality variation management
- Basis for performance guarantee certificate issuance

**Supplier evaluation**:
- Technical capability evaluation of bidding companies
- Pre-exclusion of companies below performance standards
- Criteria for long-term supply contract decisions

#### 7.2.2 Power Plant Operation Support

**Predictive maintenance**:
- Catalyst performance degradation trend analysis
- Optimal replacement timing determination
- Maintenance cost optimization

**Operation optimization**:
- Ammonia injection amount optimization
- Temperature profile adjustment
- Load variation response strategy

#### 7.2.3 Research and Development Platform

**New technology development**:
- Low-temperature active catalyst development
- Poison-resistant catalyst research
- Regeneration technology development

**Environmental regulation response**:
- Response to strengthening NOx regulations
- New pollutant removal technology
- Integrated treatment technology development

### 7.3 Economic Analysis

#### 7.3.1 Cost-Benefit Analysis

**Total construction cost**: Approximately 500 million KRW
- Equipment cost: 300 million KRW (60%)
- Installation cost: 100 million KRW (20%)  
- Commissioning cost: 100 million KRW (20%)

**Annual operating cost**: Approximately 50 million KRW
- Labor cost: 30 million KRW
- Consumables cost: 10 million KRW
- Maintenance cost: 10 million KRW

#### 7.3.2 Cost Reduction Effects

**Overseas test cost savings**:
- Overseas contract test cost: 3 million KRW per sample
- Annual test volume: 50 samples
- Savings: 150 million KRW annually

**Quality improvement effects**:
- Pre-blocking of non-conforming catalysts
- Reduced power plant shutdown risk
- Environmental penalty avoidance

**Investment payback period**: Approximately 3 years

### 7.4 Future Development Directions

#### 7.4.1 Short-term Plans (1-2 years)

**Function expansion**:
- Addition of NH₃ slip measurement function
- Introduction of moisture addition system
- Expansion of measurement automation

**Certification acquisition**:
- ISO/IEC 17025 testing laboratory accreditation
- VGB authorized testing laboratory registration
- KOLAS calibration agency registration

#### 7.4.2 Medium-term Plans (3-5 years)

**Facility expansion**:
- Multi-channel simultaneous testing capability
- High-temperature (above 400°C) testing response
- Corrosive gas treatment capability

**Standardization activities**:
- Participation in domestic standard establishment
- Participation in international standard revision
- Leadership in Asian regional standardization

#### 7.4.3 Long-term Vision (5-10 years)

**Technology innovation**:
- AI-based performance prediction model
- Digital twin technology application
- Remote monitoring system

**Market expansion**:
- Southeast Asia market entry
- Technology export and licensing
- Comprehensive environmental technology solutions

### 7.5 Social Impact

#### 7.5.1 Environmental Contributions

**Air quality improvement**:
- Contribution to NOx emission reduction
- Fine dust generation suppression
- Acid rain damage reduction

**Climate change response**:
- Thermal power efficiency improvement
- Indirect greenhouse gas reduction
- Support for clean energy transition

#### 7.5.2 Industrial Ecosystem Strengthening

**Technology capability enhancement**:
- Domestic catalyst industry technology improvement
- Related industry mutual growth
- Export competitiveness acquisition

**Human resource development**:
- Professional personnel education and training
- Technical know-how accumulation
- Next-generation expert cultivation

### 7.6 Final Remarks

The SCR catalyst performance evaluation apparatus constructed through this study is expected to establish itself as core infrastructure for domestic environmental technology development beyond a simple testing facility. Particularly, by securing testing capabilities compliant with international standards, it will enhance the reliability of domestic technology and serve as a bridgehead for overseas expansion.

We plan to develop this into a representative SCR technology verification center in the Asian region through continuous technology development and quality improvement. Through this, we aim to contribute to Korea's leap forward as an environmental technology advanced nation and respond to national expectations for clean air environment creation.

## References

1. VGB PowerTech e.V., "Guideline for the Testing of DeNOx Catalysts," VGB-R 302e, 2nd Edition, 1998.

2. STEAG GmbH, "Common Best Practices for Bench Scale Reactor Testing and Chemical Analysis of SCR DeNOx Catalyst," Supplement to VGB-R302He, May 2006.

3. Electric Power Research Institute, "Protocol for Laboratory Testing of SCR Catalyst Samples," EPRI Report 1014256, 2nd Edition, 2007.

4. Electric Power Research Institute, "Laboratory Testing Guidelines for Gas Turbine SCR and CO Catalysts," EPRI Report 3002006042, 2015.

5. Forzatti, P., "Present status and perspectives in de-NOx SCR catalysis," Applied Catalysis A: General, Vol. 222, pp. 221-236, 2001.

6. Busca, G., et al., "Chemical and mechanistic aspects of the selective catalytic reduction of NOx by ammonia over oxide catalysts: A review," Applied Catalysis B: Environmental, Vol. 18, pp. 1-36, 1998.

7. Lietti, L., et al., "Reactivity of V2O5-WO3/TiO2 catalysts in the selective catalytic reduction of nitric oxide by ammonia," Catalysis Today, Vol. 29, pp. 143-148, 1996.

8. Alemany, L.J., et al., "Reactivity and physicochemical characterization of V2O5-WO3/TiO2 De-NOx catalysts," Journal of Catalysis, Vol. 155, pp. 117-135, 1995.

9. Topsøe, N.Y., et al., "Vanadium-based catalysts for selective catalytic reduction of NOx by NH3: I. Combined temperature-programmed reduction and in situ EPR studies," Journal of Catalysis, Vol. 151, pp. 226-240, 1995.

10. Nova, I., et al., "NH3-NO/NO2 chemistry over V-based catalysts and its role in the mechanism of the Fast SCR reaction," Catalysis Today, Vol. 114, pp. 3-12, 2006.

## Appendices

### Appendix A. Apparatus Design Drawings

#### A.1 Overall System Layout

```
                    [Exhaust Scrubber]
                           ↑
    [N2] ─┐              [Flue Gas Analyzer]
    [O2] ─┤                  ↑
    [NO] ─┼─ [MFC Control Panel] ─ [Reactor] ─ [Cooler]
    [SO2]─┤                  ↑
    [NH3]─┘              [Heating System]
           
    [HMI Control Room]         [Power Supply]
```

#### A.2 Reactor Detailed Structure

**Dimensional information**:
- Total height: 800mm
- Inner diameter: 200mm (cylindrical) or 200×200mm (rectangular)
- Catalyst chamber height: 200mm
- Preheating section height: 150mm
- Cooling section height: 100mm

**Material information**:
- Reactor body: SUS316L, thickness 10mm
- Insulation: Ceramic fiber, 150mm
- Flange: JIS 10K RF, SUS316L
- Bolt: SUS316, M12×50L

#### A.3 Gas Supply Piping and Instrumentation Diagram

<figure>
  <img class="flowchart"
       src="{{ '/project/scr_catalyst/piping_diagram.png' | relative_url }}"
       alt="Gas supply piping and instrumentation diagram"
       loading="lazy">
  <figcaption>Figure A.3: Gas supply piping and instrumentation diagram (P&ID)

### Appendix B. Operating Procedures

#### B.1 Commissioning Procedure

**Stage 1: System Inspection**
1. Confirm power supply status
2. Check gas cylinder connections and pressure
3. Inspect instrument communication status
4. Confirm safety device operation

**Stage 2: Leak Test**
1. Pressurize with nitrogen gas to 1.5bar
2. Confirm pressure maintenance for 30 minutes
3. Check leaks with soap bubble method
4. Perform helium leak test

**Stage 3: Instrument Calibration**
1. Temperature gauge calibration (comparison with standard thermometer)
2. Pressure gauge calibration (comparison with standard pressure gauge)
3. MFC calibration (comparison with standard flow meter)
4. Gas analyzer calibration (using standard gases)

**Stage 4: Commissioning**
1. Nitrogen-only operation (4 hours)
2. Nitrogen+oxygen operation (2 hours)
3. Full gas operation (1 hour)
4. Temperature control performance verification

#### B.2 Daily Inspection Items

**Pre-operation inspection**:
- [ ] Gas cylinder remaining amount and pressure
- [ ] Power supply status
- [ ] Instrument communication status
- [ ] Exhaust system operation
- [ ] Safety device normal operation

**Operation monitoring**:
- [ ] Each zone temperature (every 10 minutes)
- [ ] MFC flow values (every 10 minutes)
- [ ] Analyzer measurement values (continuous)
- [ ] Differential pressure changes (every 30 minutes)
- [ ] Alarm occurrence

**Post-operation inspection**:
- [ ] Data storage confirmation
- [ ] System shutdown procedure compliance
- [ ] Residual gas purge completion
- [ ] Equipment abnormality recording
- [ ] Next operation preparation items

#### B.3 Emergency Response Procedures

**In case of gas leak**:
1. Immediately shut off corresponding gas supply
2. Activate emergency ventilation system
3. Evacuate and ensure safety
4. Identify leak point and take emergency measures
5. Contact specialists and arrange repairs

**In case of overheating**:
1. Cut off heater output for corresponding zone
2. Operate cooling system at maximum
3. Force cooling by increasing gas flow rate
4. Investigate cause after confirming temperature drop
5. Stop operation if necessary

**In case of power failure**:
1. Confirm UPS switching
2. Protect critical instruments
3. Execute safe shutdown procedure
4. Start emergency generator (if necessary)
5. Restart procedure after power restoration

### Appendix C. Calibration and Maintenance

#### C.1 Calibration Intervals and Methods

**Temperature measurement system**:
- **Interval**: 6 months
- **Method**: Comparison with standard thermometer
- **Allowable deviation**: ±2°C
- **Calibration temperatures**: 100°C, 200°C, 300°C

**Flow measurement system**:
- **Interval**: 3 months  
- **Method**: Comparison with standard flow meter
- **Allowable deviation**: ±2% F.S.
- **Calibration flows**: 20%, 50%, 80%, 100% F.S.

**Pressure measurement system**:
- **Interval**: 6 months
- **Method**: Using dead weight tester
- **Allowable deviation**: ±1% F.S.
- **Calibration pressures**: 0, 25%, 50%, 75%, 100% F.S.

**Gas analysis system**:
- **Interval**: 1 month (zero/span), 6 months (precision calibration)
- **Method**: Using standard gases
- **Allowable deviation**: ±5% measured value
- **Calibration gases**: Zero gas (N₂), span gas (each component)

#### C.2 Preventive Maintenance Plan

**Daily maintenance**:
- Visual inspection
- Operating record confirmation
- Consumables remaining amount confirmation

**Weekly maintenance**:
- Piping connection inspection
- Electrical connection inspection
- Filter condition confirmation

**Monthly maintenance**:
- MFC performance inspection
- Temperature controller performance inspection
- Safety device operation confirmation
- Calibration and adjustment

**Annual maintenance**:
- Complete system disassembly inspection
- Consumable parts replacement
- Comprehensive performance evaluation
- Improvement item derivation

#### C.3 Consumables Management

**Regular replacement parts**:

| Part name | Replacement interval | Unit price | Remarks |
|-----------|---------------------|------------|---------|
| Gasket set | 6 months | 500,000 KRW | High temperature use |
| Filter element | 3 months | 200,000 KRW | For analyzer |
| Sensor cell | 12 months | 1,000,000 KRW | NO, SO2 |
| Thermocouple | 12 months | 100,000 KRW/each | K-type |
| Fuse | As needed | 10,000 KRW/each | Various specifications |

**Standard gas consumption**:
- Zero gas (N₂): 50L per month
- NO span gas: 10L per month  
- SO₂ span gas: 10L per month
- He leak gas: 10L per year

---

*This paper was written for pure technical research purposes and does not pursue commercial interests. All data and information acquired during the research process were used only for academic purposes.*