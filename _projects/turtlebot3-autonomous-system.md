---
layout: project
date: 2025-09-05
featured: true
featured_order: 2
title: "TurtleBot3 Digital Twin Autonomy Prototype"
card_title: "TurtleBot3 Digital Twin Autonomy"
description: "A ROS2 and Gazebo prototype for testing lane following, visual signals, and arm-assisted object handling before tuning on physical robot hardware."
card_description: "A digital-twin workflow connecting mobile-robot perception, control, and a 4-DOF arm across simulation and physical demonstrations."
share-description: "A TurtleBot3 autonomy prototype combining ROS2, Gazebo, OpenCV, ArUco markers, and inverse kinematics for navigation and object handling."
subtitle: "Simulation-first integration of indoor navigation, visual cues, and arm-assisted object handling."
thumbnail-img: /project/turtlebot3-autonomous-system/system_architecture_diagram.png
video_url: "https://www.youtube.com/embed/WX1D9GZJOB4"
card_video_poster: /project/turtlebot3-autonomous-system/turtlebot3_arm_joints.png
card_media_fit: cover
permalink: /projects/turtlebot3-autonomous-system/
filter_categories:
  - robotics
  - ai
category_label: "Robotics - Autonomous Driving - Digital Twin"
impacts:
  - value: "Gazebo"
    label: "Simulation"
  - value: "ROS2"
    label: "Integration"
  - value: "Prototype"
    label: "Validation Stage"
tech_tags:
  - label: "ROS2"
    style: "prog ros"
  - label: "Computer Vision"
    style: "ai"
  - label: "Inverse Kinematics"
    style: "eng"
  - label: "Gazebo"
    style: "design"
  - label: "Python"
    style: "prog"
  - label: "ArUco"
    style: "ai"
quick_summary_note: "This case study documents the demonstrated sim-to-real workflow and the limits of the retained test evidence."
quick_summary:
  - label: "Focus"
    value: "Indoor lane navigation and arm-assisted object handling"
  - label: "Stack"
    value: "ROS2, Gazebo, OpenCV, ArUco markers, and inverse kinematics"
  - label: "Workflow"
    value: "Test in simulation, inspect intermediate outputs, then tune on hardware"
  - label: "Evidence"
    value: "Gazebo scenes, perception captures, arm geometry, and a demo video"
  - label: "Status"
    value: "Integrated robotics prototype; no operational reliability claim"
---

## Project Overview

This project used a digital-twin workflow to assemble and test an indoor mobile-robot autonomy stack. The prototype connected lane following, visual traffic cues, barrier-state detection, ArUco-based object localization, and a 4-DOF arm for object handling.

Gazebo provided a repeatable environment for checking message flow and behavior sequencing before transferring the work to physical robot hardware. The goal was not to prove a general autonomous-driving solution. It was to learn where simulation assumptions break when cameras, wheels, lighting, calibration, and arm geometry become physical constraints.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/turtlebot3-autonomous-system/digital_twin_gazebo_map.png' | relative_url }}"
       alt="Gazebo scenes used to test the mobile robot on a marked indoor course"
       loading="lazy">
  <figcaption>Gazebo environments used to exercise navigation and behavior sequencing before hardware tests.</figcaption>
</figure>

## Context and Role

The archived project material describes an integrated build but does not include a team roster or module-by-module ownership record. I therefore present my role as a **system integration and sim-to-real contribution** without claiming sole ownership of every perception, control, or manipulation component.

My focus in the portfolio is the engineering process represented by the project: connecting the autonomy modules, exposing intermediate results for debugging, and tuning behavior after moving from the simulated track to the physical platform. The discussion is limited to artifacts and decisions preserved in the project archive.

## Scope

The prototype divided the robot task into several observable behaviors:

- follow a marked indoor lane;
- detect color-based traffic cues;
- estimate whether a visual barrier is raised or lowered;
- transition between behaviors according to the current scene; and
- locate a marked object and command the arm through a pick-and-place sequence.

This scope was deliberately structured around a known course and visible cues. It did not attempt open-world navigation, safety-rated obstacle avoidance, or operation around untrained members of the public.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/turtlebot3-autonomous-system/system_architecture_diagram.png' | relative_url }}"
       alt="Architecture diagram for perception, navigation, behavior control, and manipulation"
       loading="lazy">
  <figcaption>System structure used to keep perception outputs, behavior selection, and robot commands inspectable.</figcaption>
</figure>

## Perception and Mobile Control

The lane pipeline used OpenCV processing on a camera region of interest to isolate the course marking and estimate lateral error. A steering controller converted that error into motion commands. Controller values were adjusted through closed-loop observation on the physical platform; no analytical derivation is presented because the retained material does not include traceable tuning data.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/turtlebot3-autonomous-system/lane_detection_rqt.png' | relative_url }}"
       alt="ROS visualization showing the lane-detection result and intermediate image processing"
       loading="lazy">
  <figcaption>Intermediate lane-processing output used to diagnose thresholding and steering behavior.</figcaption>
</figure>

Color and geometry were also used for traffic-light and barrier cues. These methods were appropriate for a controlled course because they were lightweight and easy to inspect. Their limitation is equally clear: fixed color thresholds can drift with illumination, exposure, reflections, and camera position. Temporal checks can reduce one-frame noise, but they do not turn a laboratory cue detector into a general perception system.

## Manipulation Workflow

For object handling, the project used an ArUco marker to estimate a target pose and a kinematic model for the 4-DOF arm. The practical integration tasks included establishing consistent coordinate frames, checking joint limits, and selecting a reachable approach rather than treating an inverse-kinematics solution as sufficient by itself.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/turtlebot3-autonomous-system/turtlebot3_arm_joints.png' | relative_url }}"
       alt="Robot arm image annotated with its joints and links"
       loading="lazy">
  <figcaption>Arm geometry used to reason about coordinate frames and reachable poses.</figcaption>
</figure>

<figure markdown="0">
  <video class="project-video" autoplay loop muted playsinline preload="metadata" aria-label="Recorded ArUco marker detection used during object-localization tests">
    <source src="{{ '/project/turtlebot3-autonomous-system/aruco-marker-detection.webm' | relative_url }}" type="video/webm">
  </video>
  <figcaption>Recorded marker-detection step from the manipulation workflow.</figcaption>
</figure>

The arm sequence remained a prototype task on a prepared setup. Grasp success depends on camera calibration, base pose, target placement, gripper geometry, and mechanical backlash; those factors require a controlled test log before a success rate can be claimed.

## Digital Twin Workflow

Simulation was most useful for repeatability and visibility. The team could reset the course, exercise state transitions, inspect ROS topics, and identify obvious coordinate or sequencing errors without risking hardware. Physical testing then exposed effects that the simulation did not reproduce accurately:

- camera thresholds changed with real lighting;
- wheel response and floor friction altered steering;
- sensor timestamps and processing delay affected behavior transitions;
- arm link dimensions and offsets required physical calibration; and
- small base-position errors propagated into the grasp pose.

This made the digital twin a development tool rather than a claim that simulation and reality were identical.

## Engineering Decisions

**Expose intermediate perception.** Publishing processed images and state made threshold and geometry failures visible during testing.

**Separate behaviors from arbitration.** Lane following, signal handling, and manipulation were treated as distinct modes so that priority and transition errors could be diagnosed.

**Validate reachability before motion.** An arm target needed to respect workspace and joint constraints before a command was sent.

**Tune on the physical platform.** Simulation provided a starting point, while final steering behavior depended on observed hardware response.

## Evidence and Outcome

The retained evidence includes Gazebo scenes, the system architecture, ROS perception captures, traffic-light and barrier examples, arm and coordinate-frame diagrams, an ArUco detection recording, and the demonstration video linked at the top of the page. These artifacts support the claim that the modules were assembled and demonstrated across simulation and physical hardware.

They do not provide a reproducible quantitative benchmark because the archive lacks raw trials, failure definitions, environment controls, and complete logs. No numerical reliability or perception result is therefore reported.

The defensible outcome is an integrated prototype and a documented sim-to-real debugging process.

## Limitations and Next Steps

- The course relied on prepared lanes, markers, and visual cues.
- No versioned test suite or raw trial log is included in the archive.
- Camera calibration and lighting robustness need controlled measurement.
- Manipulation needs repeated trials across target poses, payloads, and calibration states.
- Behavior recovery after missed detections or interrupted motion remains unquantified.
- The system is not safety-rated and should not be treated as an autonomous industrial vehicle.

A next phase should define fixed scenarios and failure criteria, record every navigation and grasp attempt, and preserve sensor data with configuration versions. That would make later performance claims traceable.

## What I Learned

The central lesson was that simulation reduces iteration cost but does not remove the need for physical calibration. A robust robotics workflow must preserve visibility into perception, state transitions, coordinate frames, and hardware response. The project also reinforced that a concise record of what was actually demonstrated is more useful than precise-looking numbers without a test trail.
