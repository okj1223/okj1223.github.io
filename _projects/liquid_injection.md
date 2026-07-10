---
layout: project
date: 2025-08-06
title: "Robot-Based Liquid Concentration Control Prototype"
card_title: "Robot-Based Liquid Concentration Control"
description: "A robotic pouring prototype that connects a Doosan M0609, load-cell feedback, Arduino acquisition, ROS2 workflow, and MQTT messaging."
card_description: "A robotic pouring prototype combining a Doosan M0609, a custom load-cell fixture, embedded acquisition, and feedback-oriented control."
share-description: "A robotic pouring prototype connecting a Doosan M0609, load-cell feedback, embedded acquisition, ROS2 workflow, and MQTT messaging."
subtitle: "Connecting mass measurement, robot motion, and feedback-oriented pouring in one working prototype."
thumbnail-img: /project/liquid_injection/hardware_architecture.png
video_url: "https://www.youtube.com/embed/yVg4EGmNAvk"
card_video_poster: /project/liquid_injection/completed_frame.jpg
card_media_fit: cover
permalink: /projects/liquid_injection/
filter_categories:
  - control
  - robotics
category_label: "Robot Integration - Measurement - Control"
impacts:
  - value: "Load-cell"
    label: "Mass Feedback"
  - value: "ROS2"
    label: "Robot Workflow"
  - value: "MQTT"
    label: "System Messaging"
tech_tags:
  - label: "ROS2"
    style: "prog ros"
  - label: "MQTT"
    style: "comm"
  - label: "Doosan M0609"
    style: "robot-hw"
  - label: "HX711 ADC"
    style: "sensor"
  - label: "Python"
    style: "prog"
  - label: "Arduino"
    style: "prog"
  - label: "CAD / 3D Printing"
    style: "design"
quick_summary_note: "A recruiter-facing account of the physical measurement stack and robot integration, scoped to end-to-end behavior rather than an accuracy benchmark."
quick_summary:
  - label: "Context"
    value: "Reduce operator-dependent variation in small-volume liquid pouring"
  - label: "My Role"
    value: "Fixture design, sensor wiring, acquisition workflow, robot integration, and experiments"
  - label: "System"
    value: "Doosan M0609, load cell, HX711, Arduino, ROS2 control, and MQTT messaging"
  - label: "Evidence"
    value: "Working hardware, integration diagram, fabrication photos, and a public demo"
  - label: "Status"
    value: "Integrated prototype; no traceable concentration-accuracy benchmark is published"
---

## Context

Pouring to a target concentration is a useful robotics control problem because the output depends on more than the final arm pose. Container geometry, remaining liquid, tilt angle, angular speed, surface tension, and delayed measurement all influence the transferred mass. A motion that works once may overshoot when the starting volume or setup changes.

The project explored a feedback-oriented alternative to a fixed pouring trajectory. A collaborative robot tilted the source container while a separate mass-measurement station observed the receiving vessel. The control workflow could then use measured mass to decide whether to continue, slow, or stop.

## My Role

I worked on the physical measurement and integration path:

- designed the load-cell mounting fixture and object-positioning features,
- fabricated the fixture with 3D printing and fitted the hardware,
- wired and soldered the load cell, HX711 converter, and Arduino connections,
- structured the sensor-acquisition and stabilization workflow,
- connected mass messages to the robot-control side through MQTT,
- and supported robot pouring trials and analysis of angle-dependent flow behavior.

My contribution was system integration rather than a claim of inventing a new fluid model or owning a production control algorithm.

## System

<figure markdown="0">
  <img class="flowchart"
       src="{{ '/project/liquid_injection/hardware_architecture.png' | relative_url }}"
       alt="Architecture linking the load cell and robot control nodes through MQTT"
       loading="lazy">
  <figcaption>The implemented measurement and control path: load cell to HX711 and Arduino, then a messaging layer connecting the weight and robot-control nodes.</figcaption>
</figure>

The measurement side uses a load cell and HX711 analog-to-digital converter. An Arduino reads the converter, and a computer-side node turns those readings into a usable mass signal. The robot side receives the relevant measurement through MQTT and commands the Doosan M0609 within the pouring sequence.

The architecture deliberately separates sensor acquisition from robot motion. That makes it possible to inspect raw readings, tune stabilization behavior, and test messaging without moving the robot. It also keeps the robot-control node from depending on low-level ADC timing.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/liquid_injection/completed_frame.jpg' | relative_url }}"
       alt="Completed custom load-cell fixture"
       loading="lazy">
  <figcaption>The completed load-cell fixture, designed to locate the receiving vessel consistently while avoiding unintended side loading.</figcaption>
</figure>

## Key Decisions

### Measure the result directly

An open-loop trajectory assumes that the same angle history always transfers the same amount. Mass feedback measures the quantity the task actually cares about. The system still needs filtering and stopping margin, but the control decision is tied to the observed transfer rather than motion alone.

### Improve mechanical repeatability before filtering

A high-resolution converter cannot correct a poor force path. The custom fixture was designed to support the load cell correctly, keep the receiving object in a repeatable area, and provide access for calibration and wiring. This reduced avoidable variation before any software processing.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/liquid_injection/hx711_soldering.jpg' | relative_url }}"
       alt="HX711 module being wired for the load-cell measurement path"
       loading="lazy">
  <figcaption>Assembly of the HX711 measurement electronics used between the load cell and Arduino.</figcaption>
</figure>

### Decouple subsystems with messaging

MQTT provided a simple interface between the measurement computer and robot-control computer. The benefit was operational: either side could be tested or restarted independently, and messages could be logged during integration. For a production design, message age, loss, ordering, and safe behavior on disconnection would need explicit handling.

### Approach the target in stages

Pouring becomes harder to stop as flow increases and measurement delay accumulates. The practical control strategy was therefore to use conservative motion near the target and treat a stale or unstable weight signal as a reason not to continue. This is more robust than assuming a single fitted angle-to-flow equation will generalize to every setup.

## Evidence and Outcome

The project produced a working measurement fixture, assembled sensor electronics, a connected acquisition and robot-control architecture, and a public robot demonstration. The media on this page shows the fabricated hardware rather than only a conceptual model.

The defensible result is end-to-end integration: physical load reaches the sensor, the reading is acquired and shared, and the robot uses the workflow during pouring experiments. Exact concentration error, model-validation scores, response time, filter performance, and statistical comparisons are not reported because the public record does not include the raw experimental dataset, calibration records, analysis scripts, or repeat protocol needed to reproduce them.

## Limitations

The prototype is sensitive to load-cell calibration, tare drift, vibration, vessel placement, fluid temperature, container geometry, network delay, and the robot's motion profile. A successful demo does not by itself establish general accuracy or robustness.

Before making a quantitative performance claim, the next validation should include:

- traceable calibration weights and repeated zero checks,
- timestamped raw mass and robot-state logs,
- predefined starting volumes, target values, and stopping criteria,
- repeated trials across more than one container and liquid condition,
- explicit treatment of spills, oscillation, and overshoot,
- and comparison against a simple open-loop baseline.

Real deployment would also require guarded workspaces, collision and joint-limit handling, stale-data detection, an operator stop path, and a defined safe response when sensing or communication fails.

The key lesson was that precision starts with the measurement chain. A well-mounted sensor, observable data path, and conservative robot behavior provide a stronger foundation than a complex control claim that cannot be reproduced.
