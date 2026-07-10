---
layout: project
date: 2025-08-21
title: "AI-Assisted PCB Inspection and Sorting Prototype"
card_title: "AI-Based PCB Inspection Prototype"
description: "A physical PCB inspection prototype combining a custom conveyor, camera-based detection, robot handling, and an experimental voice interface."
card_description: "A system-integration prototype that moves a PCB through visual inspection and prepares the result for robot-assisted sorting and operator interaction."
share-description: "A PCB inspection prototype combining YOLOv11, RealSense sensing, ROS2, MQTT, a custom conveyor, and an experimental voice interface."
subtitle: "Connecting transport, visual inspection, robot handling, and operator interaction in one prototype workflow."
thumbnail-img: /project/pcb_inspection/archi.png
video_url: "https://www.youtube.com/embed/gpQtFfs4qww"
card_video_poster: /project/pcb_inspection/conveyor-belt-prototype-v1-assembled-arduino-rails.jpg
card_media_fit: cover
permalink: /projects/pcb_inspection/
filter_categories:
  - ai
  - robotics
category_label: "AI - Robotics - Manufacturing"
impacts:
  - value: "Built"
    label: "Physical Conveyor"
  - value: "Integrated"
    label: "Vision and Robot"
  - value: "Prototype"
    label: "Validation Stage"
tech_tags:
  - label: "YOLOv11"
    style: "ai"
  - label: "ROS2"
    style: "prog ros"
  - label: "Computer Vision"
    style: "ai"
  - label: "MQTT"
    style: "comm"
  - label: "RealSense"
    style: "sensor"
  - label: "3D Printing"
    style: "design"
quick_summary_note: "The case study focuses on demonstrated hardware, integration artifacts, and clearly stated prototype limits."
quick_summary:
  - label: "Context"
    value: "Team-built system-integration prototype for PCB inspection"
  - label: "Focus"
    value: "Transport, inspect, classify, and route a board for robot handling"
  - label: "Hardware"
    value: "Custom conveyor, Arduino control, cameras, and robot arm"
  - label: "Stack"
    value: "YOLOv11, RealSense, ROS2, MQTT, and web tooling"
  - label: "Status"
    value: "Demonstrated prototype; no production-line qualification claimed"
---

## Project Overview

This project explored an end-to-end PCB inspection workflow rather than a standalone vision model. The prototype combined a custom conveyor, camera-based component inspection, a robot arm, an inspection-specification interface, and an experimental voice-control path.

The intended sequence was simple:

1. move a board into the inspection area;
2. capture an image and identify relevant board features;
3. compare the observation with the expected configuration;
4. publish the inspection result; and
5. make that result available to the robot and operator interface.

The project demonstrates physical and software integration at prototype scale. It is not presented as a production automatic optical inspection system, and no claim is made that it meets manufacturing quality, traceability, or safety standards.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/pcb_inspection/archi.png' | relative_url }}"
       alt="Architecture diagram for the PCB transport, inspection, robot, and operator-interface prototype"
       loading="lazy">
  <figcaption>System view connecting conveyor transport, visual inspection, robot handling, and operator interaction.</figcaption>
</figure>

## Context and Role

The archived write-up describes the system as a team-built prototype but does not include a dependable ownership matrix for each module. I therefore describe my role as a **system-integration contribution** and avoid claiming sole authorship of the conveyor, vision models, robot code, or speech pipeline.

My work represented in this portfolio is the integration perspective: defining the board flow, connecting hardware and software boundaries, and resolving practical mismatches between the conveyor, cameras, robot, and messaging layer. The case study is limited to the hardware and integration evidence retained with the project.

## Physical Transport Prototype

The team built a compact conveyor around a 3D-printed frame, small DC gear motors, a belt, guide rails, and Arduino-based motor control. Fabricating the transport hardware exposed constraints that are easy to miss in a software-only design: rail spacing must fit the board, the belt must track consistently, and the inspection position must be repeatable enough for the camera and robot.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/pcb_inspection/conveyor-belt-prototype-v1-assembled-arduino-rails.jpg' | relative_url }}"
       alt="Assembled PCB conveyor prototype beside a robot arm"
       loading="lazy">
  <figcaption>The assembled conveyor prototype with belt, rails, drive components, and Arduino control hardware.</figcaption>
</figure>

The conveyor was intentionally small and accessible for iteration. This made it possible to adjust the guides, inspect wiring, and test handoff positions without committing to industrial hardware. It also means that its load capacity, duty cycle, guarding, and long-term belt tracking were not validated.

## Inspection and Robot Workflow

The vision path used a webcam with a YOLOv11-based detector for board-level observations. The project also explored an Intel RealSense D435i mounted near the robot end effector for depth-aware positioning and closer inspection. ROS2 handled robot-side components, while MQTT carried lightweight events between modules that did not require a tightly typed ROS interface.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/pcb_inspection/yolo_detection_realtime.jpg' | relative_url }}"
       alt="Recorded object-detection overlay on a PCB image"
       loading="lazy">
  <figcaption>Prototype detection overlay used while integrating the inspection path.</figcaption>
</figure>

The architecture separated three questions:

- **Where is the board?** Conveyor position and camera framing establish the inspection scene.
- **What is visible?** The detector produces candidate component and board observations.
- **What should happen next?** The inspection logic publishes a result for robot handling or operator review.

This separation was useful for debugging. A failed handoff could be traced to transport positioning, image capture, perception, message delivery, or robot state rather than being treated as one undifferentiated error.

## Operator and Voice Experiments

A web-based circuit-design view was explored as a way to express the expected board layout instead of hard-coding one configuration. The project also experimented with speech-to-text, wake-word detection, text-to-speech feedback, and command routing through MQTT.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/pcb_inspection/web_circuit_designer.PNG' | relative_url }}"
       alt="Web interface for arranging an expected PCB component layout"
       loading="lazy">
  <figcaption>Prototype interface for describing an expected circuit layout.</figcaption>
</figure>

The voice path was an operator-interface experiment, not a validated factory control method. Production use would require command confirmation, access control, noise testing, clear machine-state feedback, and a separate safety-rated stop path.

## Engineering Decisions

**Build the handoffs, not only the model.** The value of the prototype came from connecting transport, perception, and handling. A high offline model score would not solve an inconsistent board pose or an ambiguous robot command.

**Use different communication tools for different boundaries.** ROS2 fit the robot environment, while MQTT made simple cross-application events easier to inspect. The tradeoff is additional schema and state-management work at the boundary.

**Keep the hardware adjustable.** Printed parts and exposed guides supported rapid mechanical changes during integration. That flexibility was appropriate for a prototype but not a substitute for production design.

**Leave final judgment reviewable.** Detection output was treated as input to an inspection decision. A manufacturing system would need traceable rules, calibrated imaging, controlled lighting, and a documented disposition process.

## Evidence and Outcome

The project archive contains photos of the physical conveyor and robot setup, CAD and fabrication recordings, inspection overlays, interface captures, architecture diagrams, and the demonstration video linked at the top of this page. This evidence supports a working integration prototype and the presence of the major hardware elements.

It does not support a reproducible quantitative benchmark. The underlying test sets, raw predictions, timing logs, cost model, and production comparison are not included, so no numerical quality, throughput, or business-impact result is reported.

The defensible outcome is an integrated demonstration of board transport, visual inspection, message-driven coordination, and robot-assisted handling.

## Limitations and Next Steps

- Testing was prototype-scale rather than continuous production-line operation.
- No reproducible evaluation set or per-defect confusion matrix is available in the archive.
- Lighting, camera calibration, board variation, and component occlusion need controlled validation.
- Conveyor guarding, electrical design, emergency stops, and robot-cell safety require professional review.
- Voice control requires noisy-environment testing and explicit confirmation for consequential commands.
- Inspection results would need serialization, board identity, timestamps, and audit history for manufacturing traceability.

The next credible milestone would be a fixed inspection protocol with a versioned board set, controlled lighting, repeatable camera poses, and recorded false accepts and false rejects. Only after that work would a numerical quality claim be appropriate.

## What I Learned

This project reinforced that automation performance is determined by interfaces between disciplines. Mechanical repeatability affects perception; perception uncertainty affects robot actions; and operator feedback affects whether failures can be diagnosed. Building the complete path made those dependencies visible and provided a stronger engineering lesson than optimizing any single module in isolation.
