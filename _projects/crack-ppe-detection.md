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

The surviving project material identifies this as a team training project, but it does not preserve a reliable task-by-task ownership record. I therefore present it as a **team contribution** and do not claim sole authorship of the perception, navigation, communication, or interface modules.

My portfolio contribution is the integrated system work represented here: understanding how the modules exchange information, bringing the robot-side and monitoring-side workflow together, and documenting the engineering decisions and limitations. The scope is limited to what the retained project artifacts can support.

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
