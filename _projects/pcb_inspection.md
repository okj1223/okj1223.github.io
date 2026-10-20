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

I contributed to **system integration** in this team-built prototype: defining the board flow, connecting hardware and software boundaries, and resolving practical mismatches between the conveyor, cameras, robot, and messaging layer. The engineering account covers the full shared system so that the conveyor, inspection logic, robot handoff, and voice interface can be understood together; it does not assign sole authorship of every module.

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

## Conveyor Design and Fabrication

The conveyor was part of the inspection instrument. Its frame, belt tracking, drive friction, and stopping behavior determined whether the camera saw a board in a repeatable position. The mechanical work covered CAD, printed frame and rail components, motor mounting, belt installation, and Arduino-controlled transport.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/pcb_inspection/conveyor_3d_modeling.gif' | relative_url }}" alt="CAD development of the printed conveyor structure" loading="lazy">
    <figcaption>Conveyor CAD and printable structure.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/pcb_inspection/conveyor-drive-assembly.webp' | relative_url }}" alt="Conveyor frame with motors installed before fitting the belt" loading="lazy">
    <figcaption>Conveyor drive assembly before belt installation.</figcaption>
  </figure>
</div>

<figure class="project-evidence-figure" markdown="0">
  <video controls playsinline muted preload="metadata" aria-label="Fabrication video of the conveyor's printed parts">
    <source src="{{ '/project/pcb_inspection/conveyor-printing.mp4' | relative_url }}" type="video/mp4">
  </video>
  <figcaption>Printing the custom conveyor parts before drive and belt assembly.</figcaption>
</figure>

The control problem included starting from rest, transporting steadily, and stopping at the inspection position. Static friction can keep a lightly driven belt stationary even when the same command sustains motion once it is moving. The implementation discussion therefore included a short starting command followed by a lower running command. Belt tension and alignment had to be addressed mechanically before treating every transport inconsistency as a software problem.

An inspection trigger turns continuous transport into a discrete job. A board crossing an image line should create one inspection event, not a new request on every frame. The event needs a board or job identifier that can follow the board through inspection, result publication, and robot handling.

## From Component Detections to a Board Judgment

The vision work had two layers. YOLO-based detection located the board and visible components. A separate template comparison evaluated whether those components matched the intended board configuration. A detector's confidence answers whether a visual feature resembles a class; it does not by itself answer whether the assembly is correct.

The documented normalization used the direction from the board center toward a recognizable connector as an orientation reference. Detected component positions could then be translated and rotated into a common board frame before matching against a template. This reduces false mismatch caused by the entire board being rotated in the camera.

Matching considers component class and normalized position, while preventing one observed component from satisfying several expected components. Missing, extra, or misplaced parts can then be reported explicitly. The documented implementation uses greedy matching, a practical heuristic whose assignment order can affect ambiguous cases.

The inspection decision also used observations across multiple frames. Aggregating several views reduces sensitivity to one blurred or partially occluded image, but it introduces a time window. The board must remain in the correct inspection region during that window, and the resulting decision must remain associated with the same board.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/pcb_inspection/circuit_judgment_overlay.jpg' | relative_url }}" alt="Circuit inspection overlay showing component locations and connection comparison" loading="lazy">
  <figcaption>Inspection overlays expose which observed features and connections contributed to the board judgment.</figcaption>
</figure>

## RealSense Alignment and Robot Integration

The robot-side workflow used a RealSense D435i with a Doosan M0609 arm. Board localization and circuit inspection are related but separate tasks: localization supplies a spatial target, while inspection compares visual structure with a specification.

For a valid aligned depth sample `Z` at image coordinates `(u, v)`, calibrated camera intrinsics give the point:

`X = (u − cx) Z / fx`, `Y = (v − cy) Z / fy`

The camera-frame point then passes through the hand–eye transform into the robot's working frame. Invalid depth, depth/color misalignment, and an incorrect transform can all create a bad robot target even when the 2D bounding box looks correct. The documented sequence approached a known observation pose and refined the board center before the next robot action.

The web circuit designer provided the other half of the contract. A specification contained component identities, positions, and expected connections in a structured representation. The inspection side cached the selected specification and compared observations against that version. Changing the displayed circuit without changing the inspection-side specification would otherwise produce a system that appeared connected while evaluating the wrong board.

| Interface | Information carried | Why it mattered |
| --- | --- | --- |
| Conveyor → inspection | Board arrival and inspection job | Avoid repeated triggers for one board |
| Circuit editor → inspection | Selected circuit specification | Separate expected configuration from observed detections |
| Camera → robot | Valid spatial target in a declared frame | Turn image localization into an executable pose |
| Inspection → workflow | Result and associated board/job | Keep handling decisions attached to the right item |

## Voice Interface: Wake Word to Structured Command

The voice work was a substantial subsystem, with its own audio features, learned detector, speaker gate, transcription, and intent interpretation. Its purpose was to let an operator request an action through a structured interface while leaving the robot workflow responsible for whether that action could execute.

The documented chain was:

**audio buffer → keyword spotting → speaker verification → utterance capture → speech-to-text → intent parsing → MQTT command**

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/pcb_inspection/voice_control_system_architecture.png' | relative_url }}" alt="Voice-control architecture connecting speech processing, language interpretation, messaging, and feedback" loading="lazy">
  <figcaption>The voice interface connected several processing stages; speech recognition alone was not the complete control path.</figcaption>
</figure>

### Keyword spotting and speaker verification

Audio was transformed into a log-Mel spectrogram so a small convolutional network could recognize a wake-word pattern over time and frequency. The development notes included positive examples, non-keyword examples, background noise, and augmentation. These serve different purposes: background examples help suppress accidental activation, while speaker variation helps avoid memorizing one recording.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/pcb_inspection/voice_to_2D_tensor.jpg' | relative_url }}" alt="Waveform and corresponding time-frequency representation for keyword spotting" loading="lazy">
    <figcaption>Waveform to time-frequency input.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/pcb_inspection/Log-Mel_changing.PNG' | relative_url }}" alt="Stages of log-Mel feature extraction from an audio signal" loading="lazy">
    <figcaption>Feature preparation for the keyword detector.</figcaption>
  </figure>
</div>

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/pcb_inspection/2D_CNN.jpg' | relative_url }}" alt="Convolutional keyword-spotting network diagram" loading="lazy">
  <figcaption>Keyword-spotting model design for the voice interface.</figcaption>
</figure>

Keyword confidence and speaker identity were separate decisions. The speaker stage compared an utterance embedding with an enrolled voice profile using cosine similarity. Passing one gate did not imply passing the other. The architecture also considered repeated-trigger suppression, a bounded audio buffer, voice activity detection, and a capture timeout so background audio could not create an indefinitely growing recording.

### Transcription, intent, and execution

After capturing an utterance, speech-to-text produced text for an intent layer. Simple known phrases could use a deterministic rule; more variable language could use a constrained language-model response. The intended output was a recognized action with parameters, not executable robot code.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/pcb_inspection/STT_system_architecture.png' | relative_url }}" alt="Speech-to-text pipeline with audio acquisition, speaker checks, transcription, and command messaging" loading="lazy">
  <figcaption>Speech processing and command publication were designed as separate stages with explicit handoffs.</figcaption>
</figure>

Network-dependent transcription and language processing run on a different timescale from belt and robot control. Queues isolate those delays from immediate device callbacks. A valid spoken command still needs workflow-state validation: a request to inspect should not start a second inspection while the current board is being handled.

MQTT's at-least-once delivery option can redeliver a message. Command identifiers and state-aware handling are therefore important interface requirements; a retry must not be interpreted as a second physical task. These architectural checks are more useful than treating a text transcript as proof that the requested robot action completed.


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
