---
layout: project
date: 2026-04-13
featured: true
featured_order: 1
title: "End-to-End Humanoid Manipulation Experiment and Data Pipeline for Warehouse Automation"
card_title: "End-to-End Humanoid Experiment & Learning Pipeline"
description: "A public-safe case study of the whole-body humanoid experiments and end-to-end teleoperation, action/no-action data design, validation, labeling, and learning pipeline I designed and operated at Robros."
card_description: "Designed and operated a whole-body humanoid experimentation pipeline spanning task protocols, positive and no-op conditions, teleoperation, synchronized collection, QA, labeling, and learning iteration."
share-description: "End-to-end whole-body humanoid experiment and learning-data pipeline designed and operated at Robros."
subtitle: "Designing when the robot should act or stop, operating the whole body, and owning the pipeline from task protocol to learning-ready data."
thumbnail-img: /assets/img/humanoid-teleoperation-workflow-cover.webp
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
  - value: "3"
    label: "Task Protocols"
  - value: "Whole-body"
    label: "Teleoperation"
  - value: "2-stage"
    label: "Automated + Manual QA"
tech_tags:
  - label: "Humanoid"
    style: "robot-hw"
  - label: "Whole-Body Control"
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
quick_summary_note: "An end-to-end experiment-design and pipeline-ownership case study; customer details, internal tooling, dataset scale, and model performance remain non-public."
quick_summary:
  - label: "Role"
    value: "Experiment and pipeline designer; whole-body teleoperator; action/no-action protocol, collection, QA, labeling, and learning-workflow owner"
  - label: "Scope"
    value: "Whole-body humanoid soft-goods packing, box transfer, and tabletop pick-and-place experiments"
  - label: "Inputs"
    value: "Master-arm motion, MANUS finger articulation, robot state, multi-view vision, and episode metadata"
  - label: "Workflow"
    value: "Design action and no-action conditions, calibrate, control, collect, validate, segment, grade, curate, train, and iterate"
  - label: "Status"
    value: "Internal research PoC; dataset scale, model details, and performance remain non-public"
---

## Context

Warehouse manipulation is difficult to reduce to a fixed sequence of robot commands. Packing deformable items depends on grasp shape and release timing. Moving boxes between storage and conveyor areas requires locomotion, torso motion, coordinated bimanual contact, and consistent placement. Human demonstration is a practical starting point only when the experiment, control protocol, synchronized recording, and evaluation criteria are designed as one system.

At Robros, I designed and ran whole-body humanoid experiments across three related task protocols:

- packing soft household items such as socks, shirts, and towels into boxes;
- transferring boxes from a storage rack to a conveyor;
- tabletop pick-and-place used to define consistent grasp, placement, and label boundaries.

Whole-body box transfer combined forward and backward stepping, torso bending and recovery, waist rotation, dual-arm box support, hand control, placement, and return to the initial pose. I designed how these actions were sequenced, collected, segmented, and evaluated—not only how the arm demonstration was recorded.

The experimental challenge was to preserve the right kind of variation. Object position and task conditions needed enough diversity for learning, while initial pose, grasp convention, whole-body motion intent, episode boundaries, and safety procedure had to remain consistent across operators. The work remains an internal research PoC, so this page presents the pipeline I designed at a public-safe level.

<div class="project-protocol-grid" markdown="0">
  <div class="project-protocol-card">
    <span>01 · Deformable objects</span>
    <strong>Soft-goods packing</strong>
    <p>Capture grasp shape, folding behavior, hand opening, and release timing for objects that do not preserve a rigid pose.</p>
  </div>
  <div class="project-protocol-card">
    <span>02 · Whole-body sequence</span>
    <strong>Box transfer</strong>
    <p>Standardize approach, lift, retreat, rotation, placement, and return while preserving safe operator-assistant coordination.</p>
  </div>
  <div class="project-protocol-card">
    <span>03 · Boundary definition</span>
    <strong>Tabletop pick and place</strong>
    <p>Define grasp conventions and frame-level pick/place boundaries while varying object pose within a controlled range.</p>
  </div>
</div>

<section class="project-owned-pipeline" aria-labelledby="owned-pipeline-title" markdown="0">
  <div class="project-owned-pipeline__header">
    <span>End-to-end ownership</span>
    <strong id="owned-pipeline-title">Pipeline I designed and operated</strong>
  </div>
  <div class="project-owned-pipeline__steps" role="list">
    <div role="listitem"><span>01</span><strong>Experiment design</strong><small>Question · variables · success criteria</small></div>
    <div role="listitem"><span>02</span><strong>Task protocol</strong><small>Whole-body sequence · safety · reset</small></div>
    <div role="listitem"><span>03</span><strong>Teleoperation</strong><small>Locomotion · torso · arms · hands</small></div>
    <div role="listitem"><span>04</span><strong>Synchronized capture</strong><small>Video · action · observation · metadata</small></div>
    <div role="listitem"><span>05</span><strong>Validation & labeling</strong><small>Automated check · manual grade · subtasks</small></div>
    <div role="listitem"><span>06</span><strong>Dataset curation</strong><small>Trim · filter · select experiment set</small></div>
    <div role="listitem"><span>07</span><strong>Learning iteration</strong><small>Train · evaluate · revise next experiment</small></div>
  </div>
  <div class="project-owned-pipeline__feedback">Rollout and review findings feed the next experiment design.</div>
</section>

## What I Designed and Owned

I designed the end-to-end experiment and data workflow shown above, then operated it in the field. My responsibilities included:

- **experiment design:** defining the task objective, initial and terminal state, controlled variables, allowed variation, repetition procedure, success criteria, failure cases, and positive, negative, start, and stop conditions;
- **whole-body control:** operating forward and backward stepping, torso bend and recovery, waist rotation, dual-arm motion, hand articulation, placement, reset, and recovery sequences;
- **human-interface design:** fitting and calibrating the master arm, incorporating MANUS finger input, defining pedal-driven state transitions, and coordinating operator-assistant safety roles;
- **collection-pipeline design:** defining preflight, synchronized episode boundaries, recorder states, ingestion checks, and the conditions required before review;
- **evaluation-pipeline design:** separating automated integrity validation from manual task evaluation, then defining grade priority, rejection rules, and pending-review handling;
- **labeling and repair design:** decomposing tasks into frame-level subtasks, setting semantic label boundaries, and defining trim or partial-deletion rules;
- **learning iteration:** curating experiment-ready subsets, reviewing downstream behavior, and feeding failures back into the next protocol and collection round.

I wrote the operating, task, collection, review, and labeling guidelines represented by the source documentation. This converted the experiment from tacit operator knowledge into a reproducible process another collector or reviewer could execute consistently.

The robot platform and model stack were collaborative systems, but the experimental design and end-to-end workflow described on this page were my responsibility. This was not an isolated data-cleaning role; it covered the path from deciding how the robot should perform the task to deciding whether the resulting episode was valid for learning.

## The Episode Data Contract I Defined

I defined each demonstration as a synchronized, reviewable episode rather than a loose collection of files. The episode combined several kinds of information:

| Input | Purpose |
| --- | --- |
| Master-arm state | Captures the operator's gross upper-body motion intent |
| MANUS glove finger state | Captures grasp shape, opening, closure, and release timing |
| Robot joint and hand state | Records what the robot actually executed |
| Multi-view camera streams | Provides task-space, object, hand, and placement context |
| Episode metadata and reviewer notes | Preserves setup, protocol version, outcome, and QA decisions |

These streams needed a shared episode boundary and useful time alignment. A visually successful run could still become poor training data when camera, hand, body, and arm states described different moments, or when the recording itself was incomplete. That requirement shaped the collection state machine and the later validation pipeline.

## Collection Protocol and Operator Setup

I defined the operator and assistant workflow around the same task contract: starting posture, object setup, whole-body movement order, completion condition, reset condition, and safety responsibility. The master-arm linkage was adjusted to the operator's shoulder, upper-arm, and forearm geometry, then checked in neutral and extended poses. This reduced mapping error caused by fit rather than task execution.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/master-arm-operator-fit.webp' | relative_url }}"
         alt="Front, side, and extended-arm views illustrating how the wearable master-arm interface is fitted to an operator"
         width="1800"
         height="896"
         loading="lazy"
         decoding="async">
    <figcaption>Fit check: shoulder alignment, linkage length, neutral wrist posture, and usable range.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/master-arm-hardware.webp' | relative_url }}"
         alt="Wearable bilateral master-arm hardware mounted to a backpack frame"
         width="1200"
         height="936"
         loading="lazy"
         decoding="async">
    <figcaption>The wearable bilateral master-arm interface used to capture upper-body motion intent.</figcaption>
  </figure>
</div>

Finger articulation was captured with MANUS input when grasp shape mattered. I operated the whole-body sequence while maintaining the agreed hand pose and motion intent; the assistant managed the workspace, supported setup changes, and kept immediate access to emergency-stop controls. I designed this division to reduce avoidable interruptions and make episode boundaries more repeatable.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/operator-pedal-controls.webp' | relative_url }}"
         alt="Three-pedal interface used for episode alignment, start, stop, failure marking, and rotation control"
         width="1024"
         height="576"
         loading="lazy"
         decoding="async">
    <figcaption>Foot controls kept episode transitions and failure marking available without leaving the master-arm posture.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/collection-preflight-check.webp' | relative_url }}"
         alt="Preflight checklist confirming configuration, robot, teleoperation, streams, cameras, and storage"
         width="1400"
         height="784"
         loading="lazy"
         decoding="async">
    <figcaption>Recording began only after configuration, robot, teleoperation, streams, cameras, and storage passed preflight.</figcaption>
  </figure>
</div>

I designed each collection cycle around the same operational sequence:

1. prepare the task and confirm the protocol version;
2. fit and initialize the operator interfaces, robot, safety controls, and camera streams;
3. align the system in a known starting pose;
4. record one continuous episode from task start to finish;
5. verify that capture completed and the episode moved through the ingestion pipeline;
6. replay, segment, grade, and document the episode;
7. hand curated data into the learning workflow;
8. use review and rollout findings to tighten the next collection run.

The assistant's role was not incidental: synchronized starts, clear workspace handling, and immediate access to safety controls were part of the experimental protocol and therefore part of a valid episode.

### Recording state was part of the protocol

A saved file was not automatically a usable episode. I incorporated the recorder lifecycle—**queued**, **in flight**, **registered**, or **failed**—into the pipeline and monitored it during collection. A run could proceed to review only after its streams and metadata were present and the episode was registered. This separated task failures from recorder or transfer failures before behavior was judged.

<div class="project-flow" role="list" aria-label="Episode lifecycle from preparation to learning handoff" markdown="0">
  <div role="listitem"><span>01</span><strong>Preflight</strong><small>Robot · teleop · cameras · storage</small></div>
  <div role="listitem"><span>02</span><strong>Record</strong><small>One continuous task episode</small></div>
  <div role="listitem"><span>03</span><strong>Register</strong><small>Confirm streams and metadata</small></div>
  <div role="listitem"><span>04</span><strong>Review</strong><small>Trim · label · grade · diagnose</small></div>
  <div role="listitem"><span>05</span><strong>Curate</strong><small>Filter and hand off to learning</small></div>
</div>

## Protocol Design: Consistency Without Uniformity

I designed the experiment so that trajectories did not need to be numerically identical. Task semantics and evaluation conditions stayed stable while selected variables changed in a controlled way.

| Hold consistent | Vary deliberately |
| --- | --- |
| Initial posture and system alignment | Object position within the allowed workspace |
| Grasp convention and wrist orientation | Object type, geometry, or box placement within protocol |
| Task order and semantic completion condition | Natural operator trajectory within the agreed motion intent |
| Episode start/stop and labeling rules | Repeated coverage of valid task configurations |
| Assistant behavior and intervention rules | Protocol-approved setup variation for learning diversity |

For tabletop pick-and-place, I defined a common grasp convention for each object before collection. Object and target position could then move within a defined range. That created meaningful variation without turning inconsistent grasp choice into label noise.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/tabletop-pick-and-place.webp' | relative_url }}"
       alt="Robot hand beside a bottle during a tabletop pick-and-place setup"
       width="1200"
       height="900"
       loading="lazy"
       decoding="async">
  <figcaption>Representative tabletop setup used to reason about grasp consistency, object-position variation, and clear pick/place boundaries.</figcaption>
</figure>

## Action and No-Action Experiment Design

I also designed the data around **whether the robot should act at all**, not only how it should complete a successful motion. The goal of the no-operation set was to suppress false positives and teach the policy to distinguish a valid target from an empty, ambiguous, unreachable, or changing scene.

The source protocol divided this experiment into three top-level episode types—**Positive**, **True Negative**, and **Borderline / Transition**—with True Negative split into Easy and Hard cases:

| Episode condition | Scene and required response | What it was designed to teach |
| --- | --- | --- |
| **Positive** | A valid target box is visible; approach, support it with both hands, and lift through teleoperation | Perform the intended task when the real target satisfies the task condition |
| **Easy Negative** | The workspace is empty; remain still for roughly the duration of a positive episode | Do not hallucinate a task when no target exists |
| **Hard Negative** | Distractors, a partial or unreachable target, a target outside the valid zone, or a configuration that cannot be supported with both hands; remain still | Reject target-like but invalid conditions instead of producing a false-positive action |
| **Borderline** | The target is absent at first and then placed in the valid zone; wait, then begin the positive task | Start acting when a previously invalid scene becomes valid |
| **Transition** | The target disappears during approach; stop immediately, return to the initial posture, then wait | Stop an initiated action when its precondition disappears |

I set the planned coverage of **Positive : Easy Negative : Hard Negative** to **2 : 1 : 1**, with a smaller number of Borderline and Transition episodes used to cover the action boundary. This was an experimental sampling decision, not a post-hoc label attached after collection.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/action-trigger-target-appearance.webp' | relative_url }}"
         alt="Workspace diagram showing an empty valid zone followed by a target box appearing in that zone"
         width="1800"
         height="530"
         loading="lazy"
         decoding="async">
    <figcaption><strong>Borderline:</strong> wait while the target is absent; begin the task only after it appears in the valid zone.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/action-stop-target-removal.webp' | relative_url }}"
         alt="Workspace diagram showing a target box in the valid zone followed by its removal"
         width="1800"
         height="530"
         loading="lazy"
         decoding="async">
    <figcaption><strong>Transition:</strong> if the target disappears during approach, stop, return to the initial posture, and wait.</figcaption>
  </figure>
</div>

These behavior conditions are separate from the later **FAIL / WARN / PASS** integrity gate. The episode condition defines what behavior the robot should demonstrate; the integrity gate decides whether the resulting recording is technically eligible for labeling and review.

## Task Decomposition and Temporal Labeling

I designed the whole-body box-transfer episode as explicit phases rather than one opaque trajectory: **Step Forward**, **Lift** (including torso bend, grasp, and stand), **Step Backward**, **Rotate**, **Place**, and **Return**. Clear boundaries made episodes easier to reproduce, compare, label, filter, and analyze phase by phase.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/episode-subtask-segmentation.webp' | relative_url }}"
       alt="Example motion traces divided into step forward, lift, step backward, rotate, place, and return phases"
       width="1800"
       height="1090"
       loading="lazy"
       decoding="async">
  <figcaption>Representative episode segmentation. The graph helps inspect motion, while the task video determines the semantic boundary when operator traces differ.</figcaption>
</figure>

I defined **Recovery** as its own semantic segment when the box was supported away from the intended center and the operator had to retry. Recovery continued until the intent to release for the retry became visible; that release intent marked the beginning of the next Lift attempt. Keeping the correction separate prevented a failed grasp and reattempt from being flattened into an apparently clean Lift.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/recovery-label-timeline.webp' | relative_url }}"
       alt="Review interface showing the recovery interval marked on the episode timeline"
       width="800"
       height="388"
       loading="lazy"
       decoding="async">
  <figcaption>The documented Recovery interval on the review timeline, kept distinct from the next Lift attempt.</figcaption>
</figure>

Numeric traces were useful evidence, but they were not enough by themselves. Different operators could produce different motor profiles for the same successful action. I therefore defined the synchronized camera view as the authority for semantic events—such as the first true vertical lift or the start of release—and used state traces to confirm, refine, or diagnose that decision.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/pick-boundary-example.webp' | relative_url }}"
         alt="Illustrated frame sequence showing the end boundary of a pick action"
         width="1400"
         height="554"
         loading="lazy"
         decoding="async">
    <figcaption>Pick boundary: the object is secured and begins moving with the hand.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/place-boundary-example.webp' | relative_url }}"
         alt="Illustrated frame sequence showing the end boundary of a place action"
         width="1400"
         height="554"
         loading="lazy"
         decoding="async">
    <figcaption>Place boundary: support transfers to the target surface and release completes.</figcaption>
  </figure>
</div>

## Quality Gates and Root-Cause Review

I designed validation as two documented stages. The first stage checks whether an uploaded episode can proceed to labeling and manual review. The second stage applies the manual grade decision tree to the episodes that are eligible to continue.

### Stage 1 — automated validation: FAIL / WARN / PASS

Automated validation ran immediately after the episode was uploaded to the review system. It first detected data defects and inconsistencies that could make labeling invalid:

- **FAIL:** labeling is not possible. Representative causes include image-integrity failure, joint-range failure, or an invalid labeling timeline.
- **WARN:** labeling is conditionally possible. A responsible reviewer checks the warning type and decides whether work may continue.
- **PASS:** automated validation passed and labeling can proceed.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/automated-validation-outcomes.webp' | relative_url }}"
       alt="Documented first-stage automated validation outcomes: FAIL means labeling unavailable, WARN requires responsible-reviewer judgment, and PASS allows labeling"
       width="1800"
       height="792"
       loading="lazy"
       decoding="async">
  <figcaption>The actual first-stage validation figure from my workflow documentation: FAIL blocks labeling, WARN requires a judgment call, and PASS allows labeling.</figcaption>
</figure>

Only **PASS** episodes and **WARN** episodes explicitly approved after review entered the second stage.

### Stage 2 — manual grade decision tree

For eligible episodes, I designed and documented a priority-ordered decision tree:

1. **Corrupted or usable?** Remove recordings that cannot serve as valid data, even as failure examples.
2. **Fail criteria?** If the data is usable but the task goal was not achieved, grade it **Fail**.
3. **Dirty Success criteria?** If the task completed with a documented quality issue or deviation, grade it **Dirty Success**.
4. If none of the higher-priority criteria apply, grade it **Success**.

<figure class="project-evidence-figure project-evidence-figure--wide" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/manual-review-decision-tree.webp' | relative_url }}"
       alt="Manual review decision tree from Unverified through Corrupted, Fail, Dirty Success, and Success, with Pending available for uncertain judgments"
       width="1800"
       height="500"
       loading="lazy"
       decoding="async">
  <figcaption>The actual manual-review flowchart I designed. Criteria are checked in priority order; an uncertain decision can be held as Pending instead of being guessed.</figcaption>
</figure>

**Unverified** was the default state before review. **Pending** held cases whose criteria could not be applied confidently and required a later decision. For task-specific workflows, only the grades allowed by that protocol continued to subtask labeling—for example, the pick-and-place guideline labeled Success and Dirty Success episodes, while Fail and Corrupted episodes exited the labeling flow.

### Review video, action, and observation together

Task video answered what happened in the workspace. Commanded **action** state represented my intended target; **observation** represented what the robot actually executed. I designed the review step to compare all three so a reviewer could separate an operator or protocol issue from a robot-side tracking mismatch, recording fault, or environmental interference.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/action-vs-observation.webp' | relative_url }}"
         alt="3D motion comparison between commanded action and observed robot state"
         width="1200"
         height="978"
         loading="lazy"
         decoding="async">
    <figcaption>3D comparison exposed pose or tracking mismatch between operator command and executed state.</figcaption>
  </figure>
  <figure class="project-evidence-card">
    <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/observation-state-traces.webp' | relative_url }}"
         alt="Frame-aligned observation traces for multiple hand joints"
         width="1200"
         height="792"
         loading="lazy"
         decoding="async">
    <figcaption>Frame-aligned state traces made timing, joint discontinuity, and hand-state mismatch easier to isolate.</figcaption>
  </figure>
</div>

I defined downgrade and rejection cases including external intervention, incorrect hand opening, unstable finger posture, inconsistent release timing, deviation from the planned sequence, unintended recovery, or action–observation mismatch. Recording a specific reason made each decision useful for both dataset curation and the next experimental round.

### Edit at the episode and subtask level

I designed review to operate below the episode level when necessary. Using frame navigation, I marked subtask endpoints, revised labels, trimmed invalid leading or trailing segments, and removed a selected corrupted range when the remaining episode was still meaningful. The rule was traceability: every change had to preserve why a sample was accepted, downgraded, repaired, or rejected.

### Treat hand articulation as first-class data

For grasp-sensitive tasks, the arm path is not enough. A delayed opening, asymmetric closure, or inconsistent release can invalidate an otherwise plausible trajectory. I treated glove and robot-hand state as first-class experimental signals so those cases were visible during evaluation.

## Experiment Design and Learning Iteration

I designed the collection as an experiment loop, not a one-way data-generation job:

1. define the behavior or failure mode the next experiment needs to test, including whether the robot should act, wait, start, or stop;
2. specify the whole-body task, positive and no-op conditions, controlled variables, allowed variation, initial state, and evaluation rule;
3. collect synchronized demonstrations under that protocol;
4. run automated FAIL/WARN/PASS validation, then the documented manual grade decision tree and semantic labeling;
5. build a learning-ready subset based on the experiment's inclusion rules;
6. run imitation-learning or reinforcement-learning-oriented experiments;
7. inspect rollout behavior and trace the result back to protocol, control, recording, label, or model issues;
8. revise the next experiment and collection round.

| Experimental question | Design decision I made | Evidence used |
| --- | --- | --- |
| Can the robot reproduce the intended whole-body sequence? | Split the task into locomotion, lift, rotation, placement, and return phases | Multi-view video, phase labels, action and observation state |
| Does the policy act only when the task condition is valid? | Collect Positive, Easy Negative, Hard Negative, Borderline, and Transition episodes with explicit expected responses | Target presence and location, operator response, task video, episode condition |
| Is variation useful or merely operator noise? | Hold posture, grasp convention, sequence, and label semantics constant while varying approved object and target positions | Protocol version, setup metadata, cross-operator episode review |
| Is a bad result a data problem or an execution problem? | Separate automated recording validation from manual task outcome evaluation | PASS/WARN/FAIL gate, Corrupted/Fail/Dirty/Success grade, reviewer reason |
| Can a marginal episode be repaired or reused? | Permit traceable trimming and subtask-level relabeling; preserve Dirty Success separately | Frame timeline, trim range, label boundaries, saved reviewer decision |
| What should change in the next run? | Feed rollout and review findings back into task setup and collection criteria | Failure categories, action–observation mismatch, downstream behavior |

## Evidence and Outcome

I built the end-to-end path from experimental question to learning-ready data and the next iteration. The pipeline connected whole-body control, operator calibration, Positive and no-op experiment design, task protocol, synchronized multi-modal recording, recorder-state monitoring, automated FAIL/WARN/PASS checks, the manual Corrupted/Fail/Dirty Success/Success decision tree, temporal and Recovery labels, action–observation diagnosis, targeted trimming, dataset filtering, and downstream experiment feedback.

The result was a reproducible experimental system across three task protocols, not merely a set of demonstrations. It made whole-body runs easier to reproduce, compare, diagnose, curate, and reuse; it also turned fit, posture, timing, recovery, evaluation, and labeling judgment into explicit operating rules. No dataset-size, policy-success-rate, ablation, production-throughput, or deployment-readiness figure is claimed on this page.

## Public Media Appearances

These are external media features rather than project demo videos or technical evaluations. I appear on camera as the **whole-body teleoperation operator**, executing the humanoid manipulation workflow described on this page.

<div class="project-media-grid" markdown="0">
  <a class="project-media-card"
     href="https://www.youtube.com/watch?v=YJqSagsb_yI&amp;t=595s"
     target="_blank"
     rel="noopener noreferrer">
    <span class="project-media-source">SBS News Story · Broadcast Feature</span>
    <strong>당신 옆자리에 AI가 출근했다…로봇이 대신 일해 준다는데 뭐가 걱정?</strong>
    <span class="project-media-role">Role: on-camera whole-body teleoperation operator demonstrating the humanoid workflow.</span>
    <span class="project-media-cta">Watch the ROBROS segment on YouTube <span aria-hidden="true">↗</span></span>
  </a>

  <a class="project-media-card"
     href="https://www.youtube.com/watch?v=CTDoi9T0g44"
     target="_blank"
     rel="noopener noreferrer">
    <span class="project-media-source">Ministry of Trade, Industry and Energy · Field Feature</span>
    <strong>여러분과 함께 일하러 왔습니다! 사람처럼 걷고 일하는 휴머노이드 실물 직관</strong>
    <span class="project-media-role">Role: on-camera whole-body teleoperation operator executing the humanoid task demonstration.</span>
    <span class="project-media-cta">Watch on YouTube <span aria-hidden="true">↗</span></span>
  </a>
</div>

## Confidentiality and Limitations

Because this work belongs to an active company research program, the public summary intentionally omits:

- dataset volume and storage structure,
- exact robot and control architecture,
- internal recorder and review-tool interfaces,
- model code, checkpoints, hyperparameters, and training infrastructure,
- reinforcement-learning rewards and simulator details,
- customer or deployment information,
- and internal evaluation results.

The workflow diagram is a portfolio abstraction rather than an export of the internal architecture. Supporting figures are documentation-safe illustrations or tightly cropped excerpts with customer, credential, and deployment details omitted. Company source code, raw demonstrations, credentials, and full internal UI captures are not published because they may contain proprietary robot behavior, workspaces, or operational details.

The current scope is whole-body experimental control and learning-pipeline development rather than a claim of autonomous warehouse deployment. Remaining technical risks include gait and manipulation coordination, demonstration variance, sensor synchronization, hand-state mismatch, object diversity, real-hardware safety, and transfer from a controlled setup to deployment-like conditions.

The main lesson was that robot-learning quality is designed before training begins. Positive and no-op conditions, start/stop transitions, whole-body task structure, operator fit, disciplined teleoperation, synchronized sensing, automated FAIL/WARN/PASS validation, and the manual grade decision tree determine whether later model work starts from a useful signal or avoidable noise.
