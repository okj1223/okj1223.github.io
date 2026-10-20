---
layout: project
title: "Dual Gear Pump System for Catalyst Regeneration"
date: 2024-04-15
description: "A portable dual gear-pump system designed for catalyst-regeneration fluid delivery, with hydraulic sizing, independent speed control, instrumentation, and modular field hardware."
card_description: "A portable dual gear-pump system combining hydraulic sizing, independent VFD control, field instrumentation, and a modular serviceable frame."
share-description: "A portable dual gear-pump system for catalyst-regeneration fluid delivery, with hydraulic sizing, independent speed control, and modular field hardware."
subtitle: "Hydraulic design, independent pump control, and serviceable hardware for field fluid delivery."
thumbnail-img: /project/dual-gear-pump-system/complete-system-main.webp
permalink: /projects/dual-gear-pump-system/
filter_categories:
  - mechanical
  - environmental
category_label: "Fluid Systems - Industrial Equipment"
impacts:
  - value: "Dual"
    label: "Pump Trains"
  - value: "VFD"
    label: "Speed Control"
  - value: "Mobile"
    label: "Field Frame"
tech_tags:
  - label: "Gear Pump"
    style: "design"
  - label: "Darcy-Weisbach"
    style: "eng"
  - label: "NPSH Review"
    style: "eng"
  - label: "LS ELECTRIC"
    style: "comm"
  - label: "4040/4080 Profile"
    style: "design"
  - label: "Inverter Control"
    style: "prog"
  - label: "Hydraulic Sizing"
    style: "data"
quick_summary_note: "A concise build record covering requirements, hydraulic architecture, component selection, controls, and field-oriented assembly."
quick_summary:
  - label: "Context"
    value: "Deliver regeneration fluid through demanding hose and elevation conditions"
  - label: "My Role"
    value: "Hydraulic sizing, pump and motor selection, frame and panel design, assembly, and commissioning"
  - label: "System"
    value: "Two independently controlled gear-pump trains with gauges, flow indication, and mobile frames"
  - label: "Evidence"
    value: "Fabricated equipment, control-panel photograph, wiring diagram, and system architecture"
  - label: "Status"
    value: "Completed hardware build; published performance values require traceable retesting"
---

## Context

The catalyst-regeneration workflow required liquid to be moved from a supply tank to hand-operated application equipment through a field hose network. Long runs, elevation changes, narrow delivery lines, fluid properties, and nozzle restrictions all affect the pressure that the pump must produce. At the same time, the equipment needed to be movable, adjustable, and maintainable at an industrial site.

The project objective was to turn those requirements into a compact pumping package with two controllable delivery trains. The core engineering problem was balancing discharge demand with suction conditions, motor load, hose losses, operator control, and protection of the positive-displacement pumps.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/dual-gear-pump-system/complete-system-main.webp' | relative_url }}"
       alt="Fabricated gear-pump system on a mobile aluminum-profile frame"
       loading="lazy">
  <figcaption>One of the fabricated pump modules, showing the motor, gear pump, gauge, flow indicator, piping, enclosure, and caster-mounted frame.</figcaption>
</figure>

## My Role

I worked across hydraulic, mechanical, and electrical integration. My contribution included:

- translating field hose, elevation, and application requirements into a pump-train architecture,
- estimating suction and discharge losses and reviewing cavitation risk,
- selecting the gear-pump, motor, variable-frequency drive, and instrumentation approach,
- laying out the pump modules and control enclosure on modular frames,
- planning independent run and speed control for the two trains,
- and supporting assembly, wiring, commissioning, and maintainability review.

This page focuses on decisions that are visible in the completed hardware. It does not treat design calculations alone as proof of measured field performance.

## System

<figure markdown="0">
  <img class="flowchart"
       src="{{ '/project/dual-gear-pump-system/system-architecture.png' | relative_url }}"
       alt="Gear-pump fluid delivery architecture"
       loading="lazy">
  <figcaption>The fluid path from regeneration tank and suction conditioning through the pump train to the delivery hose and hand tool.</figcaption>
</figure>

Each train follows the same basic path: supply tank, suction strainer and isolation valve, motor-driven gear pump, pressure and flow indication, delivery hose, and application tool. The two pumps can be controlled independently, allowing the operator to match the active equipment to the job and isolate one side for inspection.

The electrical enclosure contains separate drives and protective devices for the pump motors. Speed control provides an operating adjustment without a mechanical transmission change, while a controlled ramp reduces abrupt starts and stops.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/dual-gear-pump-system/control-panel-interior.webp' | relative_url }}"
       alt="Control enclosure with two motor drives and protective breakers"
       loading="lazy">
  <figcaption>Control enclosure during assembly, with independent motor drives, breakers, grounding, and cable-entry provisions.</figcaption>
</figure>

## Hydraulic Sizing: Work Back from the Application Point

The original design brief included a long delivery hose, elevation change, and a required pressure at the application nozzle. These are different contributions to the pump's duty. A gauge at the pump outlet does not report the pressure available at the far end of the hose.

The head budget was organized as:

`H_required = Δz + Δp_nozzle/(ρg) + Σh_friction + Σh_fittings`

For each line, velocity follows `v = Q/A`, straight-pipe loss follows `h_f = f(L/D)v²/(2g)`, and local losses follow `h_m = K v²/(2g)`. Keeping branch flow separate from total system flow matters: a shared suction line and two discharge branches do not carry the same flow.

This is why hose diameter was a central engineering decision. At a fixed flow, a smaller internal diameter raises velocity and can increase friction loss sharply. The correct calculation needs the actual hose bore, fluid viscosity, and fittings, rather than the nominal connection size alone.

Hydraulic power is `P_h = Δp Q`. Motor selection additionally accounts for pump and transmission efficiency and the operating range. The two pump trains were considered independently so that one operating condition would not be hidden behind a combined system total.

## Suction Conditions and Pump Selection

The suction side determines whether the pump receives a continuous liquid supply. Its pressure budget includes tank level, atmospheric or vessel pressure, vapor pressure, and inlet losses. Suction restriction, air ingress, or a long inlet hose can defeat a pump that otherwise has sufficient discharge capacity.

The design used gear pumps because their positive-displacement behavior supports a controllable delivery rate over a useful pressure range. Approximately, flow is displacement per revolution × speed × volumetric efficiency. Slip and fluid properties affect the realized flow, so the frequency setting is a control input, not a flow measurement.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/dual-gear-pump-system/gear-pump-internal.PNG' | relative_url }}" alt="Gear-pump mechanism showing fluid carried around the meshing gears from inlet to outlet" loading="lazy">
  <figcaption>Positive-displacement operation motivated the pump selection and made pressure relief a separate design requirement.</figcaption>
</figure>

The design also considered strainers, priming, flexible connections, isolation, and access to the pump coupling. A suction calculation needs comparison with the selected pump's requirements at the intended duty; a generic “typical NPSH” value is not a substitute for that check.

## Electrical Integration and Independent Control

Each train combined a pump and motor with a variable-frequency drive. Independent run and speed commands let the operator adjust delivery without making the second train follow every change. The shared stop function, distribution protection, and individual drive states belonged to the same control-panel design.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/dual-gear-pump-system/wiring-diagram.png' | relative_url }}" alt="Archived electrical architecture for the gear-pump motors and their drive controls" loading="lazy">
  <figcaption>Power distribution and control were documented together with the two motor-driven pump trains.</figcaption>
</figure>

The enclosure layout separated incoming power, drive connections, control wiring, and field terminations. Cable routes and service access mattered because a portable unit is repeatedly connected, moved, and inspected. Motor direction, coupling alignment, and the correspondence between panel labels and actual equipment were part of integration.

The VFD provides speed control and motor-related protection. It does not remove the need for a correctly designed relief path for a positive-displacement pump. Likewise, a pressure gauge makes an operating condition visible; it is not itself a pressure-limiting device.

## Packaging the System for Field Use

The fabricated package used modular aluminum profiles, motor/pump mounting, hose connections, instruments, a control enclosure, and casters. The layout had to keep the unit movable while leaving room to inspect leaks, reach valves, access the coupling, and read pressure and flow indications.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/dual-gear-pump-system/complete-system-page.jpg' | relative_url }}" alt="Annotated view of the completed mobile pump package and component arrangement" loading="lazy">
  <figcaption>Completed package showing how the hydraulic path, drive, controls, and portable frame were integrated.</figcaption>
</figure>

Vibration management extended beyond the motor base. A rigidly strained pipe connection can load the pump, and a poorly supported hose can interfere with moving the unit. Flexible connections and serviceable routing therefore influenced the frame arrangement.

## Commissioning Logic and Measurement Boundaries

The commissioning discussion covered rotation and priming, line routing, visible leaks, independent controls, instrument response, and the relationship between speed, flow, pressure, and motor load. A useful operating record identifies the fluid, hose arrangement, elevation, instrument location, and which pump train is running.

| Quantity | What it establishes |
| --- | --- |
| Drive frequency | Requested motor operating point |
| Measured delivery flow | Actual liquid output under that load |
| Pump-outlet pressure | Pressure before the delivery network |
| Application-point pressure | Pressure remaining after line and elevation losses |
| Motor current | Electrical loading at the observed condition |

The hardware record and hydraulic model together explain the commissioning questions. Verified flow, pressure, endurance, and protection behavior require the corresponding instrument and test records.


## Key Decisions

### Use positive-displacement gear pumps

The application called for fluid delivery against substantial system resistance. A gear pump provides a direct relationship between shaft speed and theoretical displacement and is compatible with variable-speed operation. It also requires careful overpressure protection because a blocked discharge does not simply stop flow.

### Review the complete hydraulic path

Pump selection cannot be based on nozzle pressure alone. The design considered static lift, friction along the suction and discharge hoses, fitting losses, fluid density and viscosity, and the pressure required at the application point. On the suction side, the review focused on avoiding excessive restriction and preserving margin against cavitation.

### Keep both trains independently controllable

Separate drives make startup, adjustment, fault isolation, and maintenance more practical. The dual arrangement is an operational choice, not a claim of automatic redundancy: switching, safe isolation, and failure response still depend on the final operating procedure.

### Make the package serviceable

Aluminum profiles and bolted joints allow component positions to be changed without cutting and rewelding a frame. Casters support site movement, and visible gauges help operators distinguish hydraulic problems from electrical or motor problems. Flexible connections and alignment checks help prevent piping loads from being transferred into the pump.

### Treat relief and isolation as design requirements

A positive-displacement system needs a correctly sized relief or bypass path, rated hoses and fittings, guards, grounding, and a verified emergency-stop strategy. These are system-level protections and must be validated with the actual fluid and final hose configuration.

## Evidence and Outcome

The project produced fabricated pump modules and a dual-drive control enclosure. The available images show the physical pump, motor, instrumentation, frame, piping, drives, and protective devices. The architecture and wiring assets document how the package was intended to operate and be serviced.

The defensible result is a completed hardware build and a structured hydraulic commissioning approach. Exact outlet pressure, nozzle pressure, flow, NPSH margin, target achievement, and endurance figures are not reported because the portfolio does not include the original calibrated instruments, test sheets, fluid-property records, or timestamped logs needed to reproduce them.

## Limitations

The hydraulic model depends strongly on actual hose internal diameter, roughness, fittings, elevation profile, liquid viscosity, pump efficiency, and nozzle geometry. Vendor curves and calculations should therefore be used to define a test envelope, not to replace measurement.

Before the package is represented as field-qualified, validation should include:

- calibrated pressure and flow readings at clearly identified measurement points,
- pump current and temperature across the intended speed range,
- suction pressure and evidence of cavitation-free operation,
- relief-valve and blocked-discharge testing under a controlled procedure,
- leak and pressure testing of the assembled fluid path,
- chemical-compatibility review for seals, hoses, and wetted metals,
- and repeated run data with documented inspection criteria.

The main lesson was that a fluid system is only as credible as its measurement locations and assumptions. Separating calculated requirements from measured results makes both the design review and later troubleshooting substantially clearer.
