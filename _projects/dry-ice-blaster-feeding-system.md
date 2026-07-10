---
layout: project
title: "Dry Ice Blaster Feeding System Design and Implementation"
card_title: "Dry Ice Blaster Feeding System for Catalyst Cleaning"
permalink: /projects/dry-ice-blaster-feeding-system/
date: 2024-02-06
description: "A fabricated dry-ice pellet feeder combining stainless components, motor-driven metering, pneumatic delivery, and field-oriented controls."
card_description: "A portable dry-ice pellet feeder with motor-driven metering, pneumatic delivery, stainless fabrication, and variable-speed control."
share-description: "A fabricated dry-ice pellet feeder combining stainless components, motor-driven metering, pneumatic delivery, and field-oriented controls."
subtitle: "Mechanical design, fabrication, and controls for repeatable dry-ice pellet delivery."
thumbnail-img: /project/dry-ice-blaster-feeding-system/completed_dry_ice_blaster.webp
card_media_fit: cover
filter_categories:
  - mechanical
  - environmental
category_label: "Pneumatic Systems - TIG Fabrication - Field Equipment"
impacts:
  - value: "Metered"
    label: "Pellet Feed"
  - value: "Variable"
    label: "Drive Control"
  - value: "Mobile"
    label: "Field Frame"
tech_tags:
  - label: "TIG Welding"
    style: "eng"
  - label: "Stainless Steel"
    style: "design"
  - label: "Geared Motor"
    style: "design"
  - label: "4040/4080 Profile"
    style: "design"
  - label: "LS iG5A"
    style: "prog"
  - label: "Pneumatic Conveying"
    style: "eng"
quick_summary_note: "A fabrication-led case study focused on pellet metering, pneumatic integration, field assembly, and practical commissioning risks."
quick_summary:
  - label: "Context"
    value: "Deliver dry-ice pellets consistently for industrial surface-cleaning work"
  - label: "My Role"
    value: "Mechanical layout, component selection, fabrication, controls integration, and commissioning"
  - label: "System"
    value: "Stainless hopper and feeder, geared drive, compressed-air line, nozzle, and mobile frame"
  - label: "Evidence"
    value: "Completed field hardware plus system and control diagrams"
  - label: "Status"
    value: "Fabricated prototype; certified performance and economic claims are not made"
---

## Context

Dry-ice blasting uses compressed air to accelerate solid carbon-dioxide pellets toward a surface. The pellets sublimate after impact, which avoids adding a second abrasive medium to the work area. The practical challenge in this project was not simply producing airflow. It was feeding brittle, temperature-sensitive pellets into that airflow consistently enough for field cleaning work.

Pellets can bridge in a hopper, absorb moisture, fragment during transport, or contribute to icing at restrictions. The feeder therefore had to combine mechanical metering, pneumatic delivery, cold-compatible fabrication, accessible controls, and a frame that could be moved and serviced at an industrial site.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/dry-ice-blaster-feeding-system/completed_dry_ice_blaster.webp' | relative_url }}"
       alt="Completed dry-ice pellet feeding system at a field work area"
       loading="lazy">
  <figcaption>The assembled feeder in the field, including its mobile profile frame, stainless hopper, pneumatic lines, and control enclosure.</figcaption>
</figure>

## My Role

I worked from system layout through physical integration. My contribution covered:

- defining the pellet and compressed-air flow path,
- selecting the hopper, rotary feeder, geared drive, valves, gauges, and control components,
- laying out the equipment on a modular aluminum-profile frame,
- supporting stainless fabrication and TIG-welded parts,
- integrating variable-speed feed control and emergency stopping,
- and troubleshooting pellet handling and nozzle behavior during commissioning.

This was primarily a hands-on mechanical and controls project. Detailed pressure-vessel certification, laboratory material testing, and third-party cleaning validation were outside the evidence available for this public portfolio page.

## System

<figure markdown="0">
  <img class="flowchart"
       src="{{ '/project/dry-ice-blaster-feeding-system/overall_architecture.png' | relative_url }}"
       alt="Dry-ice feeder system architecture"
       loading="lazy">
  <figcaption>Public system diagram showing the compressed-air path, pellet feeder, delivery hose, and nozzle.</figcaption>
</figure>

The system combines two paths:

1. a compressor and air-treatment path supplies the conveying flow;
2. a stainless hopper and rotating feeder introduce pellets into that flow at a controlled rate.

The pellet feeder is driven by a geared motor. A drive-control chain provides an adjustable speed reference, while the field controls expose basic run, stop, and emergency-stop functions. Gauges, valves, and separators make the pneumatic side easier to isolate and inspect.

<figure markdown="0">
  <img class="flowchart"
       src="{{ '/project/dry-ice-blaster-feeding-system/control_system.png' | relative_url }}"
       alt="Dry-ice feeder drive-control architecture"
       loading="lazy">
  <figcaption>The implemented control concept: operator speed setting and emergency stop feeding the drive and geared motor.</figcaption>
</figure>

## Key Decisions

### Meter pellets separately from conveying air

Treating air delivery and pellet metering as separate subsystems made the design easier to tune. Airflow could be checked without pellets, and feeder behavior could be inspected before connecting the full hose and nozzle path.

### Use variable-speed feed control

Pellet demand changes with the cleaning task and with the condition of the material in the hopper. A geared drive with adjustable speed provided a practical way to tune delivery without rebuilding the metering mechanism.

### Build for field maintenance

The profile frame provides mounting flexibility, caster mobility, and direct access to valves, hoses, the feeder, and the enclosure. Stainless surfaces were used where the equipment encountered pellets and cold process conditions. TIG welding supported fabrication of clean, compact stainless parts, but this page does not represent those parts as code-stamped or independently certified.

### Design around bridging and moisture

Commissioning focused on two recurring risks: interrupted pellet flow at the hopper and icing or blockage in the pneumatic path. The design response emphasized a direct hopper geometry, accessible feeder, controllable agitation or metering, dry compressed air, and purge and inspection access. These considerations matter more to reliable field use than a theoretical nozzle calculation alone.

## Evidence and Outcome

The project resulted in an assembled, mobile feeding system used in a field work environment. The available photograph shows the physical frame, hopper, feeder area, controls, and connected pneumatic hardware. The architecture and control diagrams match the visible equipment and explain how the subsystems interact.

The defensible outcome is the completed hardware integration and the commissioning knowledge gained around cold pellet handling. Cleaning efficiency, system availability, continuous-runtime results, and payback are not reported because the public record does not include traceable test sheets, baseline methodology, or financial source data.

## Limitations

This portfolio evidence does not establish a certified pressure rating, welding procedure qualification, nondestructive examination result, calibrated feed-rate envelope, or comparative cleaning benchmark. Any use around pressurized gas and dry ice also requires controls beyond the feeder itself, including:

- pressure relief and component ratings for the complete pneumatic path,
- ventilation and carbon-dioxide exposure monitoring,
- guarding around rotating equipment,
- appropriate eye, hand, hearing, and cold-contact protection,
- moisture management and safe depressurization,
- and a documented inspection and maintenance routine.

The next validation step would be a controlled test plan that records pellet mass flow, inlet and nozzle pressure, blockage events, ambient humidity, cleaning acceptance criteria, and component condition over repeated runs. Until that evidence exists, this project is presented as a fabricated and commissioned prototype rather than a quantified production system.

The main engineering lesson was straightforward: successful pneumatic equipment depends on material behavior as much as nominal airflow. Designing for bridging, moisture, access, and recovery made the system more practical than optimizing a single calculated operating point.
