---
layout: project
title: "Automated Catalyst Cleaning Robot"
date: 2025-01-10
featured: true
featured_order: 3
description: "A workshop-built prototype exploring rail-guided motion, fluid delivery, and Arduino control for SCR catalyst cleaning."
share-description: "A multidisciplinary catalyst-cleaning prototype combining mechanical fabrication, rail-guided motion, nozzle hardware, and Arduino control."
subtitle: "A physical maintenance-automation prototype developed from mechanism design through fabrication and assembly."
thumbnail-img: /project/automated-catalyst-cleaning/architecture-diagram.png
video_url: "https://www.youtube.com/embed/7pVAK6bW15U"
card_video_poster: /project/automated-catalyst-cleaning/cleaning_robot_components.webp
card_media_fit: cover
permalink: /projects/automated-catalyst-cleaning/
filter_categories:
  - robotics
  - environmental
  - mechanical
category_label: "Environmental Automation - Engineering"
impacts:
  - value: "Built"
    label: "Physical Prototype"
  - value: "5-Person"
    label: "Engineering Team"
  - value: "Workshop"
    label: "Validation Stage"
tech_tags:
  - label: "Arduino"
    style: "prog"
  - label: "CAD Design"
    style: "design"
  - label: "Fluid Delivery"
    style: "eng"
  - label: "Machining"
    style: "design"
  - label: "Motion Control"
    style: "prog"
quick_summary_note: "This page documents the fabricated prototype and clearly separates workshop evidence from uncompleted field validation."
quick_summary:
  - label: "Context"
    value: "Five-person multidisciplinary engineering prototype"
  - label: "My Role"
    value: "Led development and coordinated mechanical, control, and fabrication work"
  - label: "Focus"
    value: "Repeatable rail motion and directed fluid delivery for catalyst cleaning"
  - label: "Build"
    value: "Stepper hardware, fabricated brackets, nozzle parts, Arduino control, and CAD"
  - label: "Status"
    value: "Workshop prototype; field performance and safety qualification not established"
---

## Project Overview

This project explored how a compact mechanism could move a cleaning head along an SCR catalyst surface and deliver fluid in a repeatable pattern. A five-person engineering team developed a physical prototype spanning mechanism design, machined and fabricated parts, motion hardware, and Arduino-based control.

I led the development effort and coordinated the mechanical, control, and fabrication work represented in the project archive. The goal at this stage was to turn a maintenance concept into hardware that could be assembled, moved, and inspected in the workshop. It was not a production deployment or a validated replacement for an industrial cleaning procedure.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/automated-catalyst-cleaning/architecture-diagram.png' | relative_url }}"
       alt="Concept architecture for the catalyst-cleaning motion and fluid-delivery prototype"
       loading="lazy">
  <figcaption>Prototype concept connecting rail motion, a cleaning head, fluid delivery, and local control.</figcaption>
</figure>

## Problem and Scope

SCR catalyst modules contain narrow passages that can accumulate particulate matter. Cleaning is a combined access, positioning, and process-control problem: the tool must reach the intended area, move predictably, direct fluid without damaging nearby hardware, and be recoverable when something jams or leaks.

The project addressed a narrow part of that problem:

- build a rail-guided mechanism;
- package motors, brackets, bearings, and a cleaning head;
- create nozzle and handle components;
- sequence travel and fluid delivery with a small controller; and
- keep the assembly accessible for workshop iteration.

The prototype did not establish an approved cleaning chemistry, field procedure, catalyst-restoration result, hazardous-location rating, or operator-safety case.

## Mechanical Development

The mechanism used stepper-driven motion and fabricated supports to guide the cleaning head. Early CAD work helped check packaging and part relationships before machining and assembly. The build records show iteration through grinding, drilling and tapping, machined nozzle hardware, brackets, bearings, fasteners, and motor mounts.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/automated-catalyst-cleaning/cleaning_robot_components.webp' | relative_url }}"
       alt="Motors, bearings, brackets, fasteners, and fabricated parts prepared for the cleaning prototype"
       loading="lazy">
  <figcaption>Mechanical and motion components during workshop assembly.</figcaption>
</figure>

The physical build made several design constraints concrete. The structure had to hold alignment across travel, leave room for hoses and wiring, and allow a technician to reach fasteners during adjustment. Motor torque alone was not enough; bearing placement, bracket stiffness, rail alignment, and cable routing all affected whether motion stayed smooth.

<figure markdown="0">
  <video class="project-video" autoplay loop muted playsinline preload="metadata" aria-label="Recorded fabrication work on a metal component">
    <source src="{{ '/project/automated-catalyst-cleaning/grinding.webm' | relative_url }}" type="video/webm">
  </video>
  <figcaption>Workshop fabrication during prototype development.</figcaption>
</figure>

## Fluid-Delivery Hardware

The cleaning head and nozzle path were developed alongside the motion system. A directed outlet was preferable to an uncontrolled spray because position and dwell could be related to the mechanical sequence. The project archive includes machined nozzle and handle parts, showing that the fluid interface progressed beyond a diagram.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/automated-catalyst-cleaning/nozzle_handle.webp' | relative_url }}"
       alt="Machined metal nozzle-handle component"
       loading="lazy">
  <figcaption>Machined component for the prototype cleaning head.</figcaption>
</figure>

No claim is made here about validated pressure, flow uniformity, cleaning efficiency, chemical compatibility, or catalyst damage. Those properties would require a defined fluid, calibrated instrumentation, controlled samples, and before-and-after measurements.

## Control Approach

An Arduino-based controller was selected for local sequencing of the motion hardware and fluid valve. For a workshop prototype, this made inputs and outputs easy to inspect and allowed the team to iterate without a large controls cabinet.

The intended control sequence included:

1. establish a known travel reference;
2. move the cleaning head to a target position;
3. activate fluid delivery for the configured step;
4. continue along the path; and
5. stop on a limit, fault, or operator command.

The archive does not include a preserved firmware repository or test record. The defensible evidence is therefore the controller architecture and physical integration, not a claim about unrecorded software behaviors.

## CAD, Machining, and Assembly

The mechanism had to connect a rail reference, a moving carriage, a drive, and a cleaning head without making assembly or maintenance unnecessarily difficult. The development work included both the assembled geometry and the individual parts needed to realize it.

<figure class="project-evidence-figure" markdown="0">
  <video controls playsinline muted preload="metadata" aria-label="Original CAD assembly and motion record for the cleaning mechanism">
    <source src="{{ '/project/automated-catalyst-cleaning/cad-motion.mp4' | relative_url }}" type="video/mp4">
  </video>
  <figcaption>Recovered CAD assembly record. This shows the design work behind the fabricated mechanism.</figcaption>
</figure>

I carried out grinding work and coordinated the component fabrication and assembly. The retained workshop record also covers machining of the cleaning-head hardware and rail-related operations. These were coupled decisions: a part that fits in CAD still needs a usable datum, tool access, mounting clearance, and an assembly sequence.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/automated-catalyst-cleaning/tap-machine.webp' | relative_url }}" alt="Workshop machine operation used during fabrication of the catalyst-cleaning mechanism" loading="lazy">
    <figcaption>Recovered workshop fabrication photograph.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/automated-catalyst-cleaning/3dworking2.PNG' | relative_url }}" alt="Technical CAD drawing of the cleaning mechanism and its component geometry" loading="lazy">
    <figcaption>Drawing work connected the assembly concept to component geometry.</figcaption>
  </figure>
</div>

The critical mechanical interfaces were rail alignment, carriage resistance, drive coupling, and nozzle positioning. A misaligned rail can increase friction and consume the motor's available torque. A flexible cleaning-head mount can change stand-off distance even when the carriage reaches its commanded position. These problems require mechanical inspection as well as software changes.

## Motion Sequencing and Firmware Structure

The controller organized the mechanism as a sequence of explicit states: establish a reference, travel to the work region, apply the cleaning action, index to the next location, and return or stop. Limit-switch homing established a repeatable origin for an otherwise open-loop stepper axis.

| State or function | What the controller tracks | Engineering reason |
| --- | --- | --- |
| Homing | Switch state, direction, and travel progress | Establish the position reference before indexing |
| Travel | Requested steps and pulse timing | Move the carriage without blocking input handling |
| Cleaning dwell | Fluid command and elapsed time | Associate application with a defined location |
| Indexing | Next position and remaining pattern | Repeat the scan consistently |
| Fault/stop | Inhibit state and recovery requirement | Prevent an automatic return to motion after a fault |

Stepper pulse count is a commanded displacement. It does not measure whether the mechanism missed steps. The design discussion therefore treated homing, end limits, acceleration, and mechanical load as part of positioning, rather than relying on microstep resolution as an accuracy claim.

For a screw-driven axis, commanded linear travel follows `x = N p / S`, where `N` is the number of driver pulses, `p` is screw lead per revolution, and `S` is pulses per revolution. Microstepping changes `S`; backlash, compliance, and missed motion remain separate sources of positioning error.

Separating pulse generation and state transitions also makes the sequence inspectable. An operator or log should be able to identify whether the machine is traveling, waiting, applying fluid, or stopped. A long blocking delay can obscure input changes and make a sequence difficult to interrupt or diagnose.

## Nozzle, Structure, and Drive Calculations

The mechanical design was supported by three related calculations.

**Fluid delivery.** The orifice relation `Q = C_d A √(2Δp/ρ)` connects opening area and pressure difference to an estimated liquid flow. Hose and fitting losses belong upstream of the nozzle pressure drop. Spray coverage then depends on stand-off distance and pattern geometry, so a pressure value alone does not determine the cleaning footprint.

**Structural stiffness.** Head deflection changes where the spray lands. For an ideal cantilever with an end force, deflection is `δ = FL³/(3EI)` and a rectangular section has `I = bh³/12`. The support condition must match the actual part; applying a simply supported beam's stiffness to a cantilever would give a different result. These calculations guided the design discussion, while actual assembly stiffness still depends on joints and fastening.

**Drive capacity.** Screw force, lead, efficiency, acceleration, and guide friction determine the required drive torque. Available stepper torque must be considered at operating speed, with the actual driver and supply. Holding torque or a microstep count is insufficient to establish that the axis can move a contaminated or misaligned mechanism.

Together, these checks linked process requirements to hardware: the cleaning footprint sets travel and spacing; the head and hoses impose load; the frame and drive must maintain that geometry during motion.

## Workshop Verification and Iteration

The useful checks progressed from components to the assembled sequence: free movement and alignment, direction and homing, carriage travel, nozzle mounting, fluid connections, and coordinated motion/application. Looking at the mechanism during these stages helped separate a tight guide, a command problem, and an unsuitable nozzle arrangement.

CAD, fabricated components, nozzle hardware, the grinding record, and workshop operation document the design-to-build process. Dimensional accuracy and cleaning effectiveness require their own measurement protocols.


## Engineering Decisions

**Use accessible prototype hardware.** Stepper motors, fabricated brackets, and an Arduino made it possible to diagnose motion and wiring directly during assembly.

**Develop mechanics and process delivery together.** The nozzle could not be treated as an accessory because hose routing, reaction forces, access, and dwell behavior all affected the carriage design.

**Design for adjustment.** Bolted connections and reachable components supported alignment changes during workshop testing.

**Keep the safety boundary explicit.** A maintenance robot that handles fluid near industrial equipment requires more than limit switches. Production work would need guarding, pressure protection, leak containment, electrical review, emergency-stop design, operating procedures, and a formal risk assessment.

## Evidence and Outcome

The available evidence includes component and assembly photos, machining and fabrication recordings, CAD views, the architecture diagram, and the demonstration video linked at the top of the page. These artifacts support the claim that the team designed and fabricated a physical prototype.

The archive does not contain a controlled measurement protocol, comparison baseline, raw data, or field report. No numerical productivity, cleaning-effectiveness, financial, or safety-impact result is therefore reported.

The outcome was a workshop-scale mechanism that brought together rail-guided motion, fabricated cleaning-head components, and local control. It provided a concrete platform for identifying the next engineering tests.

## Limitations and Next Validation Steps

- No field deployment or production duty-cycle test is claimed.
- Cleaning effectiveness and potential catalyst damage were not quantified in the retained evidence.
- Fluid pressure, flow, leakage, and containment require instrumented testing.
- Motion repeatability, load capacity, stall behavior, and cable life remain unmeasured.
- Materials and electrical hardware were not documented as suitable for a hazardous industrial environment.
- Safety functions and operator procedures require independent engineering review.

The next phase should use representative catalyst samples and a controlled contamination method. Tests should record travel repeatability, delivered flow, coverage, blockage removal, substrate condition, leaks, and fault recovery. Only that evidence could support performance or maintenance-impact claims.

## What I Learned

The project reinforced the value of building across disciplinary boundaries. A nozzle decision changes the carriage load; rail alignment changes motor behavior; and maintenance access changes the frame. Leading the prototype required coordinating those dependencies and keeping the team focused on observable hardware progress. It also showed why workshop completion and field validation must be described as separate milestones.
