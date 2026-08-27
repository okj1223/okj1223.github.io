---
layout: project
date: 2026-04-13
featured: true
featured_order: 1
title: "Humanoid Manipulation Data and Learning Workflow Support for Warehouse Task Automation"
card_title: "Humanoid Learning Data Workflow for Warehouse Manipulation"
description: "A public-safe summary of my Robros research-assistant work on teleoperation, multi-modal demonstration data, dataset QA, and learning-workflow support for humanoid warehouse tasks."
card_description: "Research-assistant work at Robros on master-arm and MANUS glove teleoperation, dataset QA, and learning-workflow support for humanoid packing and box-transfer PoCs."
share-description: "Robros research-assistant work on teleoperation, multi-modal demonstration data, dataset QA, and learning-workflow support for humanoid warehouse tasks."
subtitle: "Owning the data-quality layer between teleoperation and humanoid manipulation experiments."
thumbnail-img: /project/robros-humanoid-manipulation-data-pipeline/system-overview.svg
share-img: /assets/img/humanoid-teleoperation-workflow-cover.webp
card_image: /assets/img/humanoid-teleoperation-workflow-cover.webp
card_media_fit: cover
card_media_position: center center
permalink: /projects/robros-humanoid-manipulation-data-pipeline/
filter_categories:
  - robotics
  - ai
category_label: "Humanoid Robotics - Robot Learning - Logistics"
impacts:
  - value: "2"
    label: "Task Families"
  - value: "3-level"
    label: "QA Taxonomy"
  - value: "Multi-modal"
    label: "Demonstrations"
tech_tags:
  - label: "Humanoid"
    style: "robot-hw"
  - label: "Teleoperation"
    style: "eng"
  - label: "MANUS Glove"
    style: "sensor"
  - label: "Imitation Learning"
    style: "ai"
  - label: "RL Workflow"
    style: "ai"
  - label: "Dataset QA"
    style: "data"
  - label: "Robot Vision"
    style: "sensor"
  - label: "ROS2 Workflow"
    style: "prog ros"
quick_summary_note: "An operational case study of the data side of robot learning; internal model details, dataset scale, and performance remain non-public."
quick_summary:
  - label: "Role"
    value: "Research Assistant supporting teleoperation, dataset curation, QA, and learning experiments"
  - label: "Scope"
    value: "Upper-body humanoid packing and box-transfer PoCs for logistics tasks"
  - label: "Inputs"
    value: "Master-arm motion, MANUS glove articulation, robot state, and multi-view vision"
  - label: "Workflow"
    value: "Collect, inspect, label, filter, hand off, and review downstream behavior"
  - label: "Status"
    value: "Internal research PoC; dataset scale, model details, and performance remain non-public"
---

## Context

Many warehouse manipulation tasks are difficult to describe as fixed robot programs. Packing deformable items into a box depends on grasp shape and release timing. Moving boxes from a rolltainer to a conveyor depends on approach pose, contact, object geometry, and consistent placement. Learning from human demonstration provides a practical way to begin these tasks, but only if the demonstrations are synchronized, repeatable, and suitable for training.

At Robros, I supported upper-body humanoid proof-of-concept work in two task families:

- packing soft household items such as socks, shirts, and towels into boxes;
- transferring boxes from a rolltainer to a conveyor.

The work remains an internal research PoC. This page describes my responsibilities and the public-safe workflow, not unreleased robot, customer, or model details.

![Public-safe overview of the humanoid manipulation data workflow]({{ '/project/robros-humanoid-manipulation-data-pipeline/system-overview.svg' | relative_url }}){: .flowchart}

*Conceptual overview of demonstration collection, QA, learning support, and robot evaluation. It is not an export of the internal architecture.*

## My Role

My clearest ownership was the data-quality layer of the learning loop. I directly contributed to:

- running master-arm teleoperation sessions,
- using MANUS glove input where finger articulation was part of the task,
- improving repeatability of collection procedures,
- inspecting recorded episodes,
- initial labeling, relabeling, and bad-sample removal,
- defining and applying QA criteria,
- documenting failure and downgrade cases,
- and supporting imitation-learning-centered experiments and reinforcement-learning-oriented iteration.

I do not claim sole ownership of the humanoid platform, final model architecture, training infrastructure, reinforcement-learning algorithm, reward design, or deployment decisions. My work made the data and operating procedure more useful to the engineers responsible for those layers.

## System and Data Flow

The demonstrations combined several kinds of information:

| Input | Purpose |
| --- | --- |
| Master-arm state | Captures the operator's gross upper-body motion intent |
| MANUS glove finger state | Captures grasp shape, opening, closure, and release timing |
| Robot joint and hand state | Records what the robot actually executed |
| Multi-view camera streams | Provides task-space, object, hand, and placement context |
| Episode metadata and reviewer notes | Preserves task setup, outcome, and QA decisions |

These streams need a shared episode boundary and useful time alignment. A visually successful run can still become poor training data if the camera, hand, and arm states describe different moments.

The operational pipeline was:

1. prepare the task and record its protocol version;
2. initialize the robot, teleoperation interfaces, and camera streams;
3. collect the demonstration;
4. replay the episode and inspect task completion, intervention, motion, and hand behavior;
5. assign a QA label and reviewer notes;
6. remove or separate samples that do not meet the intended training criteria;
7. hand curated data into the learning workflow;
8. use rollout observations to refine later collection and review.

This feedback loop mattered because data collection was not a one-time upstream activity. Downstream behavior exposed ambiguities in the protocol and showed which demonstrations needed tighter consistency.

## Key Decisions

### Judge learnability, not only task completion

A run that reaches the final state may contain hesitation, an unintended recovery, external intervention, or inconsistent grasp timing. Treating every completion as equivalent would teach the policy behavior that the team did not intend to reproduce.

### Preserve a three-level QA outcome

I supported a practical taxonomy:

| Label | Interpretation |
| --- | --- |
| Clean Success | Completed with behavior consistent with the collection protocol |
| Dirty Success | Completed, but with a recovery or deviation that requires case-by-case use |
| Fail | Violated the task or data-quality criteria |

The middle category prevented a forced binary decision. It kept marginal episodes available for analysis without silently mixing them into the highest-confidence training set.

### Treat hand articulation as first-class data

For grasp-sensitive tasks, the arm path is not enough. A delayed opening, asymmetric closure, or inconsistent release can invalidate an otherwise plausible trajectory. Capturing and reviewing glove and robot-hand state made those cases visible.

### Make rejection reasons actionable

Typical downgrade or rejection reasons included external interference, incorrect hand opening, unstable finger posture, inconsistent release timing, deviation from the planned task sequence, or an irregular recovery. Recording the reason helped distinguish a hardware issue from an operator-procedure or dataset issue.

## Evidence and Outcome

My contribution established a more disciplined path from raw teleoperation to learning-ready data. The work connected collection procedure, synchronized multi-modal recording, a consistent QA vocabulary, failure review, filtering, and downstream experiment feedback.

The outcome claimed here is operational rather than a model benchmark: I helped make demonstrations easier to inspect, compare, curate, and reuse across the two manipulation task families. No dataset-size, policy-success-rate, ablation, production-throughput, or deployment-readiness figure is claimed on this page.

## Confidentiality and Limitations

Because this work belongs to an active company research program, the public summary intentionally omits:

- dataset volume and storage structure,
- exact robot and control architecture,
- model code, checkpoints, hyperparameters, and training infrastructure,
- reinforcement-learning rewards and simulator details,
- customer or deployment information,
- and internal evaluation results.

The diagram is a portfolio abstraction rather than an export of the internal architecture. Company source code and raw demonstrations are not published because they may contain proprietary robot behavior, workspaces, or operational details.

The current scope is upper-body research rather than a claim of autonomous warehouse deployment. Remaining technical risks include demonstration variance, sensor synchronization, hand-state mismatch, object diversity, real-hardware safety, and transfer from a controlled setup to deployment-like conditions.

The main lesson was that robot-learning quality is built before training begins. Clear task setup, disciplined teleoperation, synchronized sensing, and explicit review criteria determine whether later model work starts from a useful signal or from avoidable noise.
