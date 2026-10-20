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

My contribution focused on **system integration and sim-to-real development**: connecting perception, behavior selection, mobile control, and manipulation; exposing intermediate results for debugging; and tuning behavior on the physical platform. The case study explains those interfaces and the integrated team build, with component-level design detail where the retained material supports it.

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

The lane pipeline used OpenCV processing on a camera region of interest to isolate the course marking and estimate lateral error. A steering controller converted that error into motion commands. Controller values were adjusted through closed-loop observation on the physical platform. The model below explains the steering relationship; specific tuning values require their original trial records.

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

## Lane Perception: From Pixels to a Steering Error

The lane-following work began with the image pipeline, because an unstable visual target makes controller tuning misleading. The documented processing sequence was **region of interest → HSV mask → reflection suppression → geometric validation → lane fit → temporal filtering**.

HSV made it possible to inspect color and brightness separately. A bright, low-saturation floor reflection can resemble a painted lane in RGB; suppressing that region before fitting reduces the chance of steering toward a light reflection. The mask still needs spatial checks: a large patch of the right color is not necessarily a lane boundary.

The lane fit used image rows as the independent coordinate, expressed as `x(y) = ay² + by + c`. A sliding search followed the boundary through the image, while previous detections supplied continuity when part of a line disappeared. The useful debugging views were the mask, selected pixels, fitted boundaries, and the resulting centerline—not only the final camera image with an attractive overlay.

At a chosen look-ahead row, the centerline gives a lateral pixel error relative to the image center. The steering controller combines this error with its rate of change. The derivative uses the actual sample interval; changing camera frequency otherwise changes the effective controller. Forward speed decreases as the error grows so the robot has time to turn.

| Stage | Engineering purpose | Failure to distinguish |
| --- | --- | --- |
| Color and brightness mask | Identify plausible lane pixels | Lighting drift versus a real lane change |
| Geometry and curve fitting | Turn fragments into a path estimate | A short reflection versus a continuous boundary |
| Temporal filtering | Reduce frame-to-frame steering jitter | Useful smoothing versus excessive delay |
| PD steering and speed scheduling | Convert path error into motion | Poor perception versus poorly tuned control |

Pixel-space curve coefficients are image measurements. Converting them into physical curvature requires camera calibration and a ground-plane mapping; an image curve alone does not establish a turning radius in metres.

## Traffic Signals and Barrier-State Detection

The robot also needed discrete decisions: proceed, wait, or hand control to another behavior. Traffic-light detection combined color masks with candidate shape checks. Red spans the end and beginning of the hue range, so its mask needs both intervals. Candidate area, circularity, and aspect ratio help reject colored objects that are not the intended signal.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/turtlebot3-autonomous-system/traffic_light_detection.png' | relative_url }}" alt="Physical-course traffic-light detection examples with red and green classification overlays" loading="lazy">
  <figcaption>Archived traffic-light debugging views on the physical course. Color classification is one input to behavior selection.</figcaption>
</figure>

Barrier detection used a different geometric cue: the orientation of the barrier and its visible points. The documented approach grouped candidate points, fitted a line, and classified its orientation as raised or lowered. A temporal decision window reduced rapid switching when an LED or line segment disappeared in a frame.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/turtlebot3-autonomous-system/barrier_detection.png' | relative_url }}" alt="Barrier perception pipeline showing the physical barrier, detected points, and line-based state analysis" loading="lazy">
  <figcaption>Barrier-state analysis connected an observed line orientation to a persistent discrete state.</figcaption>
</figure>

This distinction mattered for integration: lane following emits a continuous steering request, while a red light or lowered barrier imposes a discrete constraint on motion. Allowing all of them to publish competing wheel commands would make the combined behavior depend on callback timing.

## Marker Pose, Coordinate Frames, and Arm Kinematics

The manipulation path connected a detected ArUco marker to a reachable arm pose. The camera first estimates a marker pose from known marker geometry and calibrated image observations. That pose must then be transformed into the robot base frame before inverse kinematics can use it:

`T_base_marker = T_base_camera × T_camera_marker`

The desired grasp additionally includes a marker-to-tool offset. This keeps “where the marker is” separate from “where the gripper must be.” Confusing either the transform direction or the tool offset can produce a plausible-looking camera overlay and a consistently wrong arm motion.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/turtlebot3-autonomous-system/solvepnp_3d_visualization.png' | relative_url }}" alt="Camera and marker geometry used to explain pose estimation" loading="lazy">
    <figcaption>Pose estimation connects image points to a marker frame.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/turtlebot3-autonomous-system/camera_coordinate_3d.png' | relative_url }}" alt="Camera coordinate axes and robot geometry in a 3D visualization" loading="lazy">
    <figcaption>The camera-to-base transform connects perception to arm motion.</figcaption>
  </figure>
</div>

For the four-joint arm, the geometric solution separates base yaw from the planar shoulder–elbow problem. After subtracting the tool offset for the requested tool orientation, the wrist target has coordinates `(r, z)`. With planar link lengths `L1` and `L2`:

`c3 = (r² + z² − L1² − L2²) / (2 L1 L2)`

`q3 = ±acos(c3)`

`q2 = atan2(z, r) − atan2(L2 sin(q3), L1 + L2 cos(q3))`

The wrist angle completes the selected tool pitch, and the model's joint offsets and sign conventions convert these geometric angles into actuator commands. Both elbow branches need a joint-limit and continuity check. A target with `|c3| > 1` beyond numerical tolerance is unreachable; clipping a substantially invalid value would conceal the problem.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/turtlebot3-autonomous-system/kinematics_basics.png' | relative_url }}" alt="Arm links, joint angles, and end-effector position used in the kinematics explanation" loading="lazy">
  <figcaption>Link geometry provides the starting model. Encoder offsets, camera alignment, and the actual tool point still have to agree with it.</figcaption>
</figure>

The practical work was therefore broader than deriving an inverse formula: align coordinate frames, account for the physical tool, choose a continuous valid solution, and verify the resulting pose through forward kinematics. Near a singular posture, a small target-position change can require a large joint change. For a task Jacobian that is not square, singular values or a pseudoinverse are appropriate tools; taking an ordinary determinant or inverse of the full rectangular Jacobian is not.

## State Estimation and Behavior Arbitration

The design also explored combining wheel odometry, inertial information, and visual observations. These signals do not measure the same thing. Encoders describe wheel motion, an IMU describes rotational and acceleration-related motion, and a marker supplies a pose relative to a known frame. Raw LiDAR ranges become pose constraints only through a map or geometric observation model.

A useful state estimate must preserve those meanings and timestamps. Wheel slip can make odometry disagree with visual position; temporary occlusion can make a visual update stale. Filtering can balance uncertain measurements, but it cannot repair a wrong frame transform or a measurement model that treats ranges as coordinates.

At the system boundary, a mission state selects which behavior is allowed to command the robot. The architecture separates lane tracking, signal waiting, barrier waiting, marker approach, and arm manipulation from the final command publisher. Manipulation requires a stable mobile base; navigation should not silently resume while the arm is still executing a task.

The digital twin made these transitions repeatable. Hardware trials then exposed the factors that the virtual course simplified: reflections, camera mounting, wheel response, latency, and arm calibration. Keeping intermediate images and discrete state transitions visible made it possible to determine whether a failure began in perception, planning, or execution.


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
