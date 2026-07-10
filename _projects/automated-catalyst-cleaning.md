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
  <video class="project-video" controls muted playsinline preload="metadata" aria-label="Recorded fabrication work on a metal component">
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
