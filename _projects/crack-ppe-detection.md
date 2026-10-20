---
layout: project
title: "Industrial Safety Robot System Prototype"
card_title: "Industrial Safety Robot System"
permalink: /projects/crack-ppe-detection/
date: 2025-07-20
description: "A K-Digital team prototype combining robot navigation, camera-based hazard detection, MQTT messaging, and operator monitoring."
share-description: "A K-Digital team prototype exploring how mobile robots, computer vision, and an operator dashboard can support industrial safety monitoring."
subtitle: "A training-project prototype for connecting perception, mobile robots, and operator alerts."
thumbnail-img: /project/crack-ppe-detection/system_overview.png
video_url: "https://www.youtube.com/embed/gsk4GZmsjrw"
card_video_poster: /project/crack-ppe-detection/dashboard-prototype-poster.webp
card_media_fit: cover
card_media_position: center center
filter_categories:
  - ai
  - robotics
  - environmental
category_label: "Environmental Safety - AI"
impacts:
  - value: "Team"
    label: "K-Digital Project"
  - value: "Prototype"
    label: "Project Stage"
  - value: "Demo"
    label: "Available Evidence"
tech_tags:
  - label: "ROS2"
    style: "prog ros"
  - label: "YOLOv8"
    style: "ai"
  - label: "OpenCV"
    style: "data"
  - label: "MQTT"
    style: "comm"
  - label: "Python"
    style: "prog"
quick_summary_note: "This page describes a training prototype and separates demonstrated functionality from production safety claims."
quick_summary:
  - label: "Context"
    value: "Team RobotFactory project completed during K-Digital Training"
  - label: "Focus"
    value: "Robot-assisted monitoring of people, PPE, and visible surface cracks"
  - label: "Stack"
    value: "ROS2, YOLOv8, OpenCV, MQTT, and a monitoring dashboard"
  - label: "Evidence"
    value: "Architecture material, interface recordings, and a project demonstration"
  - label: "Status"
    value: "Integrated training prototype; not a certified or field-deployed safety system"
---

## Project Overview

This project was developed by **Team RobotFactory** during a **K-Digital Training** program. The team explored a practical question: how could mobile robots collect visual observations, share events, and give an operator one place to review potential safety issues?

The resulting prototype connected four functions:

- camera-based detection of people, PPE, and visible crack candidates;
- ROS2-based robot navigation and local processing;
- MQTT messages for sharing events and commands; and
- a dashboard for viewing robot status and detection results.

It should be read as a systems-integration exercise, not as a production safety product. The project did not establish that the system could replace a trained inspector, make safety decisions without human review, or operate under industrial certification requirements.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/crack-ppe-detection/system_overview.png' | relative_url }}"
       alt="Architecture diagram linking robot perception and navigation modules to an MQTT server and monitoring interface"
       loading="lazy">
  <figcaption>Prototype architecture: each robot reports data through MQTT to a shared monitoring system.</figcaption>
</figure>

## Context and Role

My contribution was **system integration within Team RobotFactory**: connecting the robot-side and monitoring-side workflow, organizing the information exchanged between modules, and documenting the resulting behavior and engineering decisions. The perception, navigation, messaging, and interface sections describe the shared prototype rather than assigning every module to one person.

## Problem and Scope

Visual inspection tasks are often separated by tool and operator workflow. A camera used for PPE checks may have no connection to a mobile platform, while a robot's navigation state may not be visible to the person reviewing an alert. The project scope was to connect those pieces in a single prototype:

1. a robot observes a scene;
2. a perception module produces a candidate event;
3. the event and robot state are published;
4. a monitoring interface presents the information; and
5. an operator can decide what to review or command next.

The word **candidate** is important. A model output is not proof that a worker is unsafe or that a structure is damaged. The interface was intended to support review, not to turn model confidence into an automatic safety judgment.

## Prototype Architecture

### Perception

The prototype used YOLOv8 and OpenCV-based image processing for its visual pipeline. The project explored human and PPE observations as well as visible crack candidates. These tasks have different failure modes: PPE can be partially occluded, people move through the frame, and surface markings can resemble cracks. Keeping the results as reviewable events made those uncertainties visible to the operator.

### Robot and Communication

ROS2 provided the robot-side structure for navigation, sensor data, and module coordination. MQTT provided a lightweight channel between robot instances and the monitoring application. This division kept local robot behavior separate from the higher-level event view and made it possible to add another prototype node without tightly coupling every component.

MQTT was used for demonstration-scale coordination. The project did not validate production concerns such as broker redundancy, device identity, network segmentation, message retention policy, or recovery after extended outages.

### Operator View

The dashboard brought detections and robot state into one interface. That was useful during integration because the team could see whether a missing result came from perception, transport, or presentation instead of treating the system as one opaque process.

<figure markdown="0">
  <video class="project-video" autoplay loop muted playsinline preload="metadata" aria-label="Recorded dashboard prototype showing safety events and robot information">
    <source src="{{ '/project/crack-ppe-detection/dashboard-prototype.webm' | relative_url }}" type="video/webm">
  </video>
  <figcaption>Recorded monitoring interface from the prototype.</figcaption>
</figure>

## Person Detection, PPE Association, and Tracking

The perception pipeline combined a lightweight YOLO detector with downstream logic for interpreting detections over time. Person and PPE detections are related but different outputs: a helmet visible somewhere in an image does not establish that every visible person is wearing one.

The engineering problem was to associate candidate equipment with the relevant person, retain confidence and location, and avoid generating a new incident every time the same person appeared in another frame. Bounding-box overlap, plausible body regions, and temporal association provide practical structure for that decision.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/crack-ppe-detection/detection_framework.png' | relative_url }}" alt="Detection framework connecting image processing, object detection, tracking, and event reporting" loading="lazy">
  <figcaption>The detection workflow separated visual observations from the events reported to the operator.</figcaption>
</figure>

The project notes explored Kalman-based tracking to smooth position changes and maintain a track through short interruptions. A prediction helps associate a new detection with an existing track, but it does not create a new observation. Track age, missed frames, and uncertainty should remain visible when a person is temporarily occluded.

This is also why per-frame confidence, detection accuracy, tracking stability, and incident quality are separate evaluation questions. A stable track can still have the wrong class, and a correct detection can still generate an incorrectly associated alert.

## Crack Candidates and Image Segmentation

The crack-analysis path started with a candidate image region and then used color/brightness segmentation and morphology to describe the visible feature. The retained screenshot shows the intermediate mask and detected region, making the processing more inspectable than a final “crack” label alone.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/crack-ppe-detection/hsv_segmentation.png' | relative_url }}" alt="Crack-candidate image with segmentation mask and detection overlay" loading="lazy">
  <figcaption>Crack-candidate segmentation from the prototype. Image measurements need calibration before being interpreted as physical dimensions.</figcaption>
</figure>

Morphological operations can remove isolated noise or join fragmented regions, but they can also change the apparent width of a thin feature. Lighting, texture, joints, and shadows are important confounders. The useful output retains the image evidence and candidate geometry for review.

The design additionally explored depth-assisted measurements. Pixel area is not physical surface area, and camera-to-surface range is not automatically crack depth. Volumetric estimates require a calibrated reference surface, adequate spatial resolution, and a validated measurement model. The available images demonstrate the processing concept; calibrated crack-volume accuracy requires a separate measurement study.

## Navigation and Multi-Robot Coordination

Robot navigation and inspection-event handling operated at different levels. Local ROS2 processing handled the robot's observations and navigation stack. A supervisory layer received status, distributed task requests, and associated reported events with their source robot.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/crack-ppe-detection/nav_arc.PNG' | relative_url }}" alt="Navigation architecture linking planning, obstacle handling, control, and robot state" loading="lazy">
  <figcaption>Navigation was a separate subsystem within the wider inspection workflow.</figcaption>
</figure>

The coordination design considered global planning, local obstacle response, and task prioritization. At the integration boundary, the important questions were concrete: which robot owns a task, whether the request was accepted, whether it is still executing, and what happens when the robot disconnects.

Priority-based dispatch is distinct from a mathematically verified distributed-consensus algorithm. The public case study preserves the implemented-system concept and its coordination interfaces, while treating the former consensus equations and large-fleet claims as design exploration without a retained validation package.

## MQTT Events, Commands, and Operator State

MQTT connected robot-side observations to the monitoring interface. The system architecture separated detections and status from operator commands so a dashboard display update could not be confused with an instruction to the robot.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/crack-ppe-detection/detect_arc.PNG' | relative_url }}" alt="MQTT detection communication diagram with robot publishers and shared monitoring" loading="lazy">
    <figcaption>Detection reporting through the shared messaging layer.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/crack-ppe-detection/protocol_comparison.png' | relative_url }}" alt="Archived communication-flow diagram linking MQTT topic publishers and subscribers" loading="lazy">
    <figcaption>Topic and message flow made the system interfaces explicit.</figcaption>
  </figure>
</div>

A useful event record identifies the robot, event type, observation time, confidence, location context, and supporting image. An operator action needs its own identity and completion state. Network receipt, command acceptance, and physical task completion are three different events.

The reliability design must also account for stale status and duplicate delivery. A recently disconnected robot should not remain visually indistinguishable from a healthy robot simply because its last status was retained. Likewise, retransmitting a command should not create a second navigation task.

## Dashboard and Mobile Operator Prototype

The monitoring work brought robot status, detections, and operator interaction into a shared view. The dashboard and application demonstrations are part of the project evidence, alongside the robot-side perception diagrams.

<figure class="project-evidence-figure" markdown="0">
  <video controls playsinline muted preload="metadata" aria-label="Recovered operator application demonstration for the safety-inspection prototype">
    <source src="{{ '/project/crack-ppe-detection/operator-application.mp4' | relative_url }}" type="video/mp4">
  </video>
  <figcaption>Operator-application demonstration showing the prototype interaction and monitoring flow.</figcaption>
</figure>

The operator needs enough context to review a candidate incident: what was detected, where it was observed, whether the robot is still connected, and which image supports the event. This is a human review workflow. The interface demonstration does not establish autonomous industrial safety decisions, predictive maintenance, or validated augmented-reality operation.

## Evaluation Boundaries

An end-to-end evaluation needs to follow an observation through detection, association, publication, display, and any requested robot action. That catches integration failures that a detector-only score misses: the wrong robot identifier, a delayed image, a duplicated event, or a command shown as complete before the robot finishes.

The demonstrated contribution is the connected perception, tracking, navigation, messaging, and interface workflow. Quantitative fleet-scale reliability and detection accuracy require retained datasets and repeatable trials.


## Engineering Decisions

**Modular event flow.** Perception, navigation, messaging, and presentation were separated so that each part could be tested without rebuilding the full application.

**Human review.** The prototype treated detections as prompts for inspection. This is more appropriate for an early safety project than presenting uncertain model output as a final decision.

**Visible integration state.** Dashboard feedback helped diagnose whether data was being generated, transmitted, and rendered. For a multi-module robot system, that observability was as important as adding another algorithm.

**Prototype-first scope.** The project focused on showing an end-to-end path. Features such as fleet-scale orchestration, safety-rated stopping, formal risk assessment, and certified alert delivery remained outside the demonstrated scope.

## Evidence and Outcome

The retained evidence consists of the architecture diagram, recorded dashboard and application views, and the demonstration video linked at the top of this page. Together, they support the claim that the team assembled and presented an integrated prototype.

They do **not** provide a reproducible quantitative benchmark. The archive does not include the underlying dataset split, test protocol, raw logs, or independent verification, so no numerical system-performance or safety-effectiveness result is reported.

The defensible outcome is therefore straightforward: the team demonstrated how a robot perception pipeline, ROS2 navigation, MQTT event exchange, and an operator interface could be connected for a safety-monitoring scenario.

## Limitations and Next Validation Steps

- No production-site deployment or safety certification is claimed.
- The archived material does not include a reproducible dataset, confusion matrix, or raw timing logs.
- Camera performance under occlusion, dust, glare, low light, and changing viewpoints remains unquantified.
- Crack candidates require an inspection method that can distinguish cosmetic surface markings from structural defects.
- MQTT security, failure recovery, and fleet behavior would need dedicated testing before operational use.
- Responsibilities and response procedures for human operators were not validated as part of this prototype.

A credible next phase would define one narrow inspection scenario, collect traceable test data, record false positives and false negatives, and evaluate the full operator response loop. That evidence would be required before making any numerical performance or safety-effectiveness claim.

## What I Learned

The main lesson was that safety robotics is an integration and evidence problem, not only a model-selection problem. A useful prototype must show where an observation came from, how it moved through the system, and where human judgment enters the process. Just as importantly, portfolio documentation must distinguish a demonstrated prototype from a validated operational system.
