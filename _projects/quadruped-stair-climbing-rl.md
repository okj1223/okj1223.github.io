---
layout: project
date: 2026-10-15
featured: false
title: "Learning to Climb Stairs: Quadruped Reinforcement Learning and Control"
card_title: "Quadruped Stair Climbing with Reinforcement Learning"
description: "A simulation research project combining PPO, terrain-aware foot trajectories, actuator modeling, and contact-level diagnostics to develop a quadruped stair-climbing controller."
card_description: "Built and evaluated a terrain-aware PPO locomotion pipeline, modeled motor delay and joint limits, and traced stair-climbing failures from policy outputs to actual foot support."
share-description: "Quadruped stair-climbing research: PPO, hardware-aware simulation, foothold planning, and measured controller evaluation."
subtitle: "Connecting learned actions, feasible foot trajectories, and actual load transfer in an actuator-aware Isaac simulation."
thumbnail-img: /project/quadruped-stair-climbing-rl/stair-climbing-cover.webp
share-img: /project/quadruped-stair-climbing-rl/stair-climbing-cover.webp
card_image: /project/quadruped-stair-climbing-rl/stair-climbing-cover.webp
card_media_fit: cover
card_media_position: center center
permalink: /projects/quadruped-stair-climbing-rl/
filter_categories:
  - robotics
  - ai
  - control
category_label: "Robot Learning - Legged Locomotion - Simulation"
impacts:
  - value: "4 steps"
    label: "7 cm Risers in Simulation"
  - value: "PPO"
    label: "GPU-Batched Learning"
  - value: "40–50 ms"
    label: "Modeled Actuator Delay"
tech_tags:
  - label: "Reinforcement Learning"
    style: "ai"
  - label: "Isaac Sim / PhysX"
    style: "robot-hw"
  - label: "PyTorch / PPO"
    style: "prog python"
  - label: "Locomotion Control"
    style: "eng"
  - label: "Actuator Modeling"
    style: "robot-hw"
  - label: "Terrain Mapping"
    style: "sensor"
  - label: "Experiment Design"
    style: "data"
quick_summary_note: "Simulation research, September–October 2026. Results below distinguish moving ascent, stopping, controller changes, and new policy training."
quick_summary:
  - label: "Role"
    value: "Simulation, PPO training, locomotion-control integration, and physical failure analysis"
  - label: "Platform"
    value: "12-joint quadruped in Isaac Sim / PhysX, with fitted motors, joint limits, and actuation delay"
  - label: "Demonstrated"
    value: "Four 7 cm steps in simulation; continuous ascent and measured contact dynamics"
  - label: "Status"
    value: "Research prototype with ideal terrain mapping; smooth stopping, higher stairs, and hardware transfer remain open"
---

## The Engineering Problem

A quadruped can lift its feet above a stair and still fail to climb it. The landing point may be reachable while the path to it crosses a joint limit. A scheduled stance phase may begin before the foot carries weight. A policy can earn more reward while becoming rougher or failing to stop at the top.

I developed this project around that gap between **a requested motion and a physically supported motion**. Starting from an existing ASRobotics model and locomotion material, I built and iterated a simulation learning and evaluation workflow: terrain-conditioned control, PPO training, actuator-aware execution, and time-aligned analysis of the resulting behavior. The robot model and upstream learning methods are existing foundations; the case study focuses on my integration, experiments, and debugging work.

The longer-term task is reliable stair ascent under realistic execution constraints. The current learned-controller evidence concerns four **7 cm risers**, each with a **30 cm tread**. A 12 cm extension was also tested and failed to complete. Earlier 20 cm reference-controller demonstrations belong to a different experiment and are not counted as learned-policy success here.

## Simulation Demo

<figure class="project-evidence-figure" markdown="0">
  <video class="project-video" controls playsinline muted preload="metadata" poster="{{ '/project/quadruped-stair-climbing-rl/stair-climbing-cover.webp' | relative_url }}" width="960" height="540" aria-label="Real-time Isaac simulation of a learned quadruped controller ascending four 7 centimetre steps">
    <source src="{{ '/project/quadruped-stair-climbing-rl/stair-ascent-7cm.mp4' | relative_url }}" type="video/mp4">
    <a href="{{ '/project/quadruped-stair-climbing-rl/stair-ascent-7cm.mp4' | relative_url }}">Watch the stair-climbing simulation.</a>
  </video>
  <figcaption>One nominal-start rollout, recorded directly from live Isaac physics at 1× speed: 13.44 seconds of video, with ascent completion at 13.46 seconds in the control log. Orange = front right, cyan = front left, purple = rear right, yellow = rear left. Colors and stair grids are visual aids without collision changes. This is moving ascent, not a stop-at-the-top demonstration or a physical-robot test.</figcaption>
</figure>

## What I Built

- **A hardware-aware simulation contract:** robot geometry, collision behavior, joint bounds, fitted torque behavior, actuator delay, and separate physics and control clocks.
- **A GPU-batched PPO workflow:** versioned training configurations, saved policy and optimizer state, continuation checks, and physical rollout evaluation after training.
- **Terrain-conditioned execution:** learned gait and foot-placement adjustments connected to full-foot landing checks, constrained inverse kinematics, and position control.
- **A diagnostic pipeline:** aligned requested targets, transmitted and delayed commands, measured joint motion, per-foot support, non-foot contact, body motion, and joint torque.
- **Controlled comparisons:** common checkpoints and initial conditions, fixed criteria, matched-time and matched-position analysis, and preservation of both gains and regressions.

## From Observation to Supported Motion

The controller needs to know where the terrain is, how the robot is moving, and what motion is requested. Its runtime inputs combine terrain-map features, joint state and history, IMU-related state, the public velocity command, and controller state. The terrain map and localization are idealized simulation inputs in these experiments; a camera-based deployment pipeline has not been validated.

The later controller uses a 1,651-feature actor input and 28 learned outputs. Those outputs adjust gait timing, foot placement, and body feedback. A trajectory and kinematics layer converts them into targets for the robot's 12 joints. The resulting motion still has to survive delayed actuation, contact, and the robot's actual dynamics.

<figure class="project-evidence-figure" markdown="0">
  <a href="{{ '/project/quadruped-stair-climbing-rl/control-pipeline.svg' | relative_url }}" aria-label="Open the control-loop diagram at full size">
  <img src="{{ '/project/quadruped-stair-climbing-rl/control-pipeline.svg' | relative_url }}" alt="Control loop: available terrain and robot observations feed a PPO actor; learned outputs pass through foot trajectories and constrained inverse kinematics, then delayed joint control and Isaac physics. Measured state returns to the controller. Privileged training and diagnostic information is kept separate from actor inputs." width="1440" height="1000" loading="lazy" decoding="async">
  </a>
  <figcaption>The policy and the execution layer form one closed loop. Simulator contact truth is available for training rewards and diagnostics, while the actor and runtime support logic use their declared public inputs.</figcaption>
</figure>

The simulation runs at **200 Hz physics** and **50 Hz control**: four 5 ms physics steps per 20 ms command. Nominal actuator delay is **40 ms for hip/thigh joints** and **50 ms for calf joints**. I retained these delays and the declared physical limits when comparing controller changes. Removing delay or enlarging joint limits would change the task being measured.

The foot-trajectory and support layers provide geometric and position-control assistance. They are not a claim of a complete whole-body dynamics controller or full MPC. A valid kinematic solution is a candidate command; actual support must be checked in the physics rollout.

## Learning and Evaluation

I tested direct joint-target PPO, demonstration-assisted variants, and a later policy combined with terrain-aware foot trajectories. The development moved beyond adjusting reward weights: when a failure originated in an infeasible path or a broken support transition, I examined the execution layer before spending more learning budget.

The later PPO setup used parallel environments, clipped policy updates, an actor–critic split, and explicit checkpoint and optimizer restoration. One October training continuation used **256 environments × 32 rollout steps × 32 updates = 262,144 additional control transitions**. That is a recorded training budget, not an independent success-rate measurement or the project's total training volume.

Evaluation began from the floor and ran continuously without intermediate state resets. Learning curves, completed training jobs, and successful checkpoint reloads were tracked separately from ascent outcomes. I also checked command-to-joint alignment and physical-step counts so a visually plausible replay could not substitute for a real evaluated rollout.

Later mild domain randomization varied mass and inertia, center-of-mass offset, friction, gains, delay, and selected sensor biases. The compared runs reused the same 20 development conditions. These conditions were used during iteration and therefore do not establish held-out generalization or real-world reliability.

## A Measured Landing Repair

One useful comparison kept the learned policy fixed and changed two parts of the execution layer. First, I removed a competing height-restoration component when a planned stance foot was still airborne and needed to lower toward the surface. Second, I made the support estimate more conservative by accounting for rotor acceleration inferred from existing joint-velocity history.

The second change matters because motor effort can accelerate the leg itself. Treating all of that effort as ground support can tell the controller that a foot is supporting the body while the foot is still in the air. The correction reduced this error in the recorded comparison, but it is not a complete inverse-dynamics model or a replacement for a verified contact sensor.

Both controllers completed the nominal 7 cm staircase. For the common **0.5–12.0 second** interval:

| Recorded quantity | Before | After |
| --- | ---: | ---: |
| Body roll RMS | 5.45° | 4.66° |
| Body roll-rate RMS | 0.349 rad/s | 0.320 rad/s |
| Foot-impact 90th percentile | 839.9 N | 793.7 N |
| Joint-torque RMS | 32.59 N·m | 32.79 N·m |
| Full-rollout ascent completion time | 12.02 s | 13.46 s |

<figure class="project-evidence-figure" markdown="0">
  <a href="{{ '/project/quadruped-stair-climbing-rl/landing-comparison.svg' | relative_url }}" aria-label="Open the measured landing comparison at full size">
  <img src="{{ '/project/quadruped-stair-climbing-rl/landing-comparison.svg' | relative_url }}" alt="Three paired bar charts show nominal-case roll RMS decreasing from 5.45 to 4.66 degrees, foot-impact P90 decreasing from 839.9 to 793.7 newtons, and joint-torque RMS increasing slightly from 32.59 to 32.79 newton metres." width="1440" height="600" loading="lazy" decoding="async">
  </a>
  <figcaption>Selected metrics recomputed from the retained comparison summary. This is one nominal condition with two bundled controller changes, not an isolated causal effect or a multi-seed benchmark. Lower roll and impact coexist with slightly higher torque and slower completion.</figcaption>
</figure>

Front-right loading also arrived sooner after landing: the mean delay to two consecutive 20 ms samples above 30 N changed from **0.272 s to 0.208 s**, across 12 landing events in each rollout. Some upper-stair regions became rougher even as the overall roll metric improved. I retained the useful behavior without presenting this candidate as uniformly smoother locomotion.

## What the Failures Revealed

### A phase transition is not a load transfer

A foot can be labeled “stance” by the gait clock while carrying no load. If the next leg lifts at that moment, the planned support pattern and the physical support pattern diverge. I compared phase transitions with estimated support and actual diagnostic contact forces, rather than treating the gait schedule as proof of touchdown.

### A foot target needs a coordinate frame and a timestamp

A landing goal stored in the body frame moves in the world when the body translates or pitches. In one diagnosed failure, an unchanged virtual goal moved about **5.3 cm forward and 6.7 cm upward** in world coordinates. Refreshing its vertical location against the current map recovered three previously failed conditions, but it also lost ten previous successes.

The matched mild-randomization comparison made that regression visible:

| Controller variant, same policy checkpoint | Moving-ascent completions | Result relative to baseline |
| --- | ---: | --- |
| Retained baseline | 14/20 | Reference |
| Pre-lift body preparation | 11/20 | 0 gained; 3 lost |
| Current-map vertical landing refresh | 7/20 | 3 gained; 10 lost |

The two variants were separate changes to the same baseline, not successive training improvements. Neither was adopted as an overall replacement. A controller can fix a real local failure while disrupting behavior the policy already depends on.

### A reachable endpoint does not guarantee a feasible swing

At 12 cm riser height, a landing point could be accepted while intermediate swing targets violated the joint workspace. I added whole-path checks and timing changes without expanding the physical limits. One change removed seven consecutive front-right target rejections; another produced a short loaded contact on the first tread. Full ascent still failed.

<figure class="project-evidence-figure" markdown="0">
  <video class="project-video" controls playsinline muted preload="metadata" poster="{{ '/project/quadruped-stair-climbing-rl/height-extension-poster.webp' | relative_url }}" width="960" height="540" aria-label="Failed 12 centimetre stair extension after a short PPO adaptation">
    <source src="{{ '/project/quadruped-stair-climbing-rl/stair-extension-12cm-failure.mp4' | relative_url }}" type="video/mp4">
    <a href="{{ '/project/quadruped-stair-climbing-rl/stair-extension-12cm-failure.mp4' | relative_url }}">Watch the failed higher-stair evaluation.</a>
  </video>
  <figcaption>12 cm extension after a short PPO adaptation: the evaluated controller failed at about 2.30 seconds. The recorded adaptation completed eight updates and 32,768 transitions. It was not promoted over the retained 7 cm controller. This result does not establish the robot's physical height limit.</figcaption>
</figure>

### Reward improvement is not task completion

An earlier direct-joint policy visited the top region in all 20 development conditions while achieving **0/20 stable stops** under the stated finish criterion. Contact analysis showed a case where a thigh carried much of the body weight. Position, reward, and appearance alone would have missed that behavior.

This led me to examine reward contributions on states actually visited by the policy, distinguish foot support from non-foot contact, and report ascent, stopping, smoothness, and actuator effort separately. Joint torque is a mechanical measurement here; it does not establish electrical efficiency, temperature, or hardware safety.

## Current Outcome and Next Step

The project produced a working simulation learning pipeline, recorded 7 cm four-step ascent, and a set of reproducible methods for tracing failures through observation, action, delayed execution, contact, and body response. Its strongest engineering result is the ability to test whether a proposed repair changes the intended physical behavior—and to retain partial gains without hiding new failures.

The remaining work is coordinated body–foot motion through support transitions, smoother ascent, reliable stopping, and higher-stair evaluation under the same actuator contract. The current evidence assumes ideal terrain mapping and localization. Physical sensing, inference-to-actuator parity, latency variation, and real-robot validation require separate verification.

## Evidence and Reproducibility

This page was assembled from retained September–October experiment records, with the latest comparison dated October 15, 2026. The demo is an existing recorded rollout; preparing this case study did not start new training or simulation. Source, checkpoints, and original run records were preserved.

A compact [results and media provenance summary]({{ '/project/quadruped-stair-climbing-rl/results-summary.json' | relative_url }}) contains the reported measurements, experiment identifiers, comparison scope, and video hashes. Full source, weights, and raw traces remain in the research archive.
