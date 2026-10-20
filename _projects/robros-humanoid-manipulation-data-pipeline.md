---
layout: project
date: 2026-04-13
featured: true
featured_order: 1
title: "OpenVLA Humanoid Manipulation and Experiment Pipeline at ROBROS"
card_title: "ROBROS: OpenVLA & Humanoid Learning"
description: "My work as a research assistant in the ROBROS AI team: OpenVLA manipulation experiments, whole-body teleoperation, experiment and dataset design, quality assurance, and physical rollout analysis."
card_description: "Worked in the ROBROS AI team on OpenVLA sock and box manipulation, designing the experiment and data pipeline and analyzing target selection, missed grasps, and retry behavior."
share-description: "ROBROS AI-team experience with OpenVLA, whole-body humanoid experiments, dataset design, and rollout failure analysis."
subtitle: "AI-team research spanning OpenVLA manipulation, whole-body demonstrations, dataset quality, and the behavior of learned policies on a real robot."
thumbnail-img: /assets/img/humanoid-teleoperation-workflow-cover.webp
share-img: /assets/img/humanoid-teleoperation-workflow-cover.webp
card_image: /assets/img/humanoid-teleoperation-workflow-cover.webp
card_media_fit: cover
card_media_position: center center
permalink: /projects/robros-humanoid-manipulation-data-pipeline/
filter_categories:
  - robotics
  - ai
category_label: "Humanoid Robotics - Vision-Language-Action - Robot Learning"
impacts:
  - value: "3"
    label: "Task Protocols"
  - value: "Whole-body"
    label: "Teleoperation"
  - value: "2-stage"
    label: "Automated + Manual QA"
tech_tags:
  - label: "OpenVLA"
    style: "ai"
  - label: "Vision-Language-Action"
    style: "ai"
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
quick_summary_note: "AI-team research experience combining OpenVLA manipulation experiments with an end-to-end demonstration and evaluation workflow. Individual rollout observations are reported without inferring a benchmark success rate."
quick_summary:
  - label: "Role"
    value: "Research assistant in the ROBROS AI team; experiment and pipeline designer, whole-body teleoperator, dataset curator, and rollout reviewer"
  - label: "Scope"
    value: "OpenVLA sock pick-and-place and box lift-and-place; whole-body soft-goods packing, box transfer, and tabletop protocols"
  - label: "Inputs"
    value: "Camera images and language instructions for VLA; synchronized robot state, master-arm and MANUS input, video, and episode metadata for demonstrations"
  - label: "Workflow"
    value: "Design task and action/no-action conditions, collect and curate demonstrations, support learning experiments, review inference, and revise the next experiment"
  - label: "Status"
    value: "Real-robot research PoC with OpenVLA; exact checkpoints, training settings, dataset scale, and aggregate performance remain non-public"
---

## Context

Warehouse manipulation is difficult to reduce to a fixed sequence of robot commands. Packing deformable items depends on grasp shape and release timing. Moving boxes between storage and conveyor areas requires locomotion, torso motion, coordinated bimanual contact, and consistent placement. Human demonstration is a practical starting point only when the experiment, control protocol, synchronized recording, and evaluation criteria are designed as one system.

At **ROBROS, I worked as a research assistant in the AI team**, contributing to **OpenVLA-based humanoid manipulation experiments** and designing the experiment and demonstration-data pipeline. Our OpenVLA work included **sock pick-and-place** and **box lift-and-place**, with camera images and language instructions conditioning robot actions. Reviewing physical rollouts was part of the work: I examined which object the robot approached, whether a grasp actually succeeded, and how it behaved after a miss.

Across the broader humanoid program, I designed and ran three related task protocols:

- packing soft household items such as socks, shirts, and towels into boxes;
- transferring boxes from a storage rack to a conveyor;
- tabletop pick-and-place used to define consistent grasp, placement, and label boundaries.

Whole-body box transfer combined forward and backward stepping, torso bending and recovery, waist rotation, dual-arm box support, hand control, placement, and return to the initial pose. I designed how these actions were sequenced, collected, segmented, and evaluated—not only how the arm demonstration was recorded.

The experimental challenge was to preserve the right kind of variation. Object position and task conditions needed enough diversity for learning, while initial pose, grasp convention, whole-body motion intent, episode boundaries, and safety procedure had to remain consistent across operators. The work remains an internal research PoC, so this page presents the pipeline I designed at a public-safe level.

## AI-Team Work with OpenVLA

Vision-language-action (VLA) models connect visual observations and a language instruction to robot actions. **OpenVLA was the model family we used for the sock and box manipulation experiments.** Its public model supports adaptation to robot tasks through demonstration-based fine-tuning; the [OpenVLA project](https://openvla.github.io/) describes that general method. Our company's checkpoint, adaptation settings, prompts, and robot interface are not specified here.

My work connected the team's model experiments to the physical task: define what should happen, collect and review demonstrations under that definition, prepare usable episodes, and inspect the learned behavior during inference. I designed the task protocols and data workflow described below, operated whole-body demonstrations, and used rollout failures to inform subsequent collection and evaluation. The model stack and robot platform were shared team systems.

| Part of the AI workflow | My contribution and its purpose |
| --- | --- |
| Task and experiment design | Defined object setup, action/no-action conditions, completion criteria, and failure cases so the intended behavior could be collected and evaluated consistently |
| Demonstration data | Operated the robot through teleoperation and organized synchronized images, actions, observations, and episode metadata |
| Learning-data preparation | Defined QA gates, semantic labels, recovery boundaries, trimming rules, and subset selection to support model-training experiments |
| OpenVLA inference review | Examined sock and box rollouts for target selection, actual object possession, continued motion after a missed grasp, and regrasp attempts |
| Experimental feedback | Connected observed behavior to questions about demonstration coverage, stopping, recovery, and the next experiment |

VLA and imitation learning describe different aspects of this workflow. VLA describes the model's vision, language, and action interface. Imitation learning describes learning from demonstrations, which can also be used to adapt a VLA. The demonstrations and QA pipeline therefore had a direct role in the AI work.

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
- **learning iteration:** curating experiment-ready subsets, reviewing downstream behavior, and feeding failures back into the next protocol and collection round;
- **VLA experiment review:** working with the AI team's OpenVLA sock and box experiments, analyzing target selection and failed-grasp responses during inference, and using those observations to frame recovery and generalization questions.

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

I designed the collection as an experiment loop connected to the AI team's model work:

1. define the behavior or failure mode the next experiment needs to test, including whether the robot should act, wait, start, or stop;
2. specify the whole-body task, positive and no-op conditions, controlled variables, allowed variation, initial state, and evaluation rule;
3. collect synchronized demonstrations under that protocol;
4. run automated FAIL/WARN/PASS validation, then the documented manual grade decision tree and semantic labeling;
5. build a learning-ready subset based on the experiment's inclusion rules;
6. use curated data in the team's imitation-learning and OpenVLA experiment workflow;
7. inspect rollout behavior and trace the result back to protocol, control, recording, label, or model issues;
8. revise the next experiment and collection round.

| Experimental question | Design decision I made | Evidence used |
| --- | --- | --- |
| Can the robot reproduce the intended whole-body sequence? | Split the task into locomotion, lift, rotation, placement, and return phases | Multi-view video, phase labels, action and observation state |
| How does OpenVLA respond when execution diverges from the demonstrations? | Review target choice, grasp outcome, continuation, and retry as separate events | Sock and box inference recordings, object state, and the task's expected response |
| Does the policy act only when the task condition is valid? | Collect Positive, Easy Negative, Hard Negative, Borderline, and Transition episodes with explicit expected responses | Target presence and location, operator response, task video, episode condition |
| Is variation useful or merely operator noise? | Hold posture, grasp convention, sequence, and label semantics constant while varying approved object and target positions | Protocol version, setup metadata, cross-operator episode review |
| Is a bad result a data problem or an execution problem? | Separate automated recording validation from manual task outcome evaluation | PASS/WARN/FAIL gate, Corrupted/Fail/Dirty/Success grade, reviewer reason |
| Can a marginal episode be repaired or reused? | Permit traceable trimming and subtask-level relabeling; preserve Dirty Success separately | Frame timeline, trim range, label boundaries, saved reviewer decision |
| What should change in the next run? | Feed rollout and review findings back into task setup and collection criteria | Failure categories, action–observation mismatch, downstream behavior |

## From Curated Episodes to a Learning Experiment

The learning workflow depended on an explicit relationship between what the robot observed and what action it was asked to execute. An episode could contain visually convincing behavior while still being unsuitable for a particular training experiment—for example, if the command was shifted relative to the image or if an intervention changed the meaning of the target action.

For the OpenVLA work, the task condition included a language instruction and the policy received visual input. More generally, imitation learning maps an observation and task condition to a robot action; some policy designs also use observation histories or action sequences. The data contract must therefore answer four questions:

| Contract question | Consequence for the experiment |
| --- | --- |
| What does an action represent? | Joint targets, increments, and task-space targets require different interpretation |
| Which observation precedes that action? | Incorrect timing can teach a response to information unavailable at execution time |
| How are episode and subtask boundaries handled? | Histories and targets should not cross an unrelated reset or task transition |
| Which episodes are eligible? | Success, recovery, no-op, and intervention cases have different learning roles |

My protocol and review work made these distinctions available before dataset selection. A successful packing episode could contribute task execution, a clean no-op episode could teach waiting, and a labeled recovery segment could support a separate question about recovering from deviation. Combining them without their conditions would erase the intent of the experiment.

Train/validation organization also needs an episode-level boundary. Adjacent frames from one teleoperated motion are highly related. Putting neighboring frames in different splits can make imitation error look reassuring while saying little about a new object placement, a different operator, or an independently collected episode.

## Runtime Execution and Action–Observation Diagnosis

The runtime question was whether a learned command produced the intended physical response. This extended the same action–observation comparison used during demonstration review into rollout analysis.

An inference output has to retain its action convention, joint order, units, frame, and timing as it passes through the robot interface. Observation history, normalization, execution limits, and reset behavior are part of that boundary. A policy file alone is not the complete deployed behavior.

When a policy uses sequence-based outputs, the execution layer must also define which predicted command is current, how a newly predicted sequence replaces an older one, and how interruption or task completion invalidates pending commands. This is a general interface consideration; it does not specify the action-output format of our OpenVLA setup.

I used the recorded task video, action state, observation state, and semantic timeline together to organize diagnosis:

| Rollout symptom | What the comparison helps investigate |
| --- | --- |
| Correct intention, delayed hand opening | Command timing versus actual hand response |
| Plausible arm path, failed release | Finger state and release boundary rather than arm pose alone |
| Action begins before the task is valid | Positive/no-op data and trigger definition |
| Continued action after target removal | Stop-transition data and execution-state handling |
| Unexpected recovery motion | Whether the segment is intentional recovery, intervention, or a task failure |
| Good offline fit, poor physical rollout | Coverage, alignment, action semantics, and closed-loop error accumulation |

This kept model changes connected to a specific experimental question. A lower imitation loss was useful information, but physical task completion, appropriate waiting, release quality, and recovery behavior still needed their own review.

## OpenVLA Rollouts: Target Selection and Failed-Grasp Responses

I presented these experiments in my October 20, 2026 research report. The report identifies my industry role in the AI team, our use of OpenVLA, and the sock and box tasks. Its recorded examples show why evaluating a policy requires following the object and the task outcome through time.

### Box task: a change in the selected target

In the box experiment, inference began at approximately **15 seconds** in the original recording. The robot first handled boxes placed above a larger supporting container. Once those target boxes had been removed, its hands approached the supporting container itself at approximately **33 seconds**.

We had not used that container as a manipulation target or demonstrated that approach in our task-specific demonstrations. The attempt to grasp it was unsuccessful. The observation was therefore a change in target selection, with the new manipulation task still incomplete.

This raised a concrete question: was the response driven by the visual scene, the language instruction, or a familiar movement pattern? We had not isolated those causes. My account of the task-specific demonstrations also does not establish what was present in the model's pretraining data. The useful finding was the behavior to investigate, rather than a claim of verified zero-shot task success.

### Sock task: continuing after a miss versus trying again

The sock recording contained two different responses after an unsuccessful grasp:

| Original recording interval | Observed behavior | Evaluation consequence |
| --- | --- | --- |
| **16.5–23.5 s** | The sock remained on the table while the hand continued toward the destination box | A placement-like motion occurred without the object. Arm motion alone would misclassify the attempt |
| **47.5–59.6 s** | After a miss, the hand returned to the sock and attempted another grasp; it briefly moved the sock, then lost it again | A retry was observable, but the excerpt ended without a completed placement |

We had not added explicit recovery demonstrations to our task-specific training, which made the retry particularly interesting. That is my account of the experiment's data preparation, rather than an audit of the model's full training history. The recording alone cannot establish whether the policy detected failure internally or why it tried again.

For evaluation, I separated **attempted grasp**, **object possession**, **retry**, and **completed placement**. A return motion can be worth studying even when recovery fails. This distinction also connects directly to the Recovery labels, failure grades, and action/no-action conditions in the data pipeline.

### Research questions arising from the experiments

These observations shaped my interest in reliable recovery, generalization across objects and instructions, learning with fewer demonstrations, and VLA model design. They suggested controlled comparisons: vary the instruction while holding the scene fixed, vary target availability under the same instruction, and compare failed-grasp responses across repeated trials. These are research questions motivated by the observed behavior, not completed ablation results.

## VLA, Imitation Learning, and Reinforcement Learning

The OpenVLA experiments used the team's demonstration and learning workflow to study real robot behavior. The research report also records earlier online reinforcement-learning attempts that did not work as expected. It provides no result tying an RL update to improved performance on these OpenVLA tasks, leaving that connection as a question for further experimentation.

The evaluation structure separated **recording validity**, **task outcome**, **execution quality**, and **experimental coverage**. This made it possible to preserve a useful failed rollout, reject corrupted data, distinguish a dirty success from a clean one, and identify which conditions the next collection round needed.

The resulting contribution connected AI-team experimentation with controlled task definitions, usable demonstrations, traceable data decisions, and physical rollout diagnosis. OpenVLA is identified at the model-family level; exact checkpoints, training settings, and internal numerical results remain within the company program.


## Evidence and Outcome

As a research assistant in the ROBROS AI team, I worked with OpenVLA sock and box manipulation experiments and built the path from experimental question to learning-ready data and the next iteration. The pipeline connected whole-body control, operator calibration, Positive and no-op experiment design, task protocol, synchronized multi-modal recording, recorder-state monitoring, automated FAIL/WARN/PASS checks, the manual Corrupted/Fail/Dirty Success/Success decision tree, temporal and Recovery labels, action–observation diagnosis, targeted trimming, dataset filtering, and downstream experiment feedback.

The result was a reproducible experimental system across three task protocols, not merely a set of demonstrations. It made whole-body runs easier to reproduce, compare, diagnose, curate, and reuse; it also turned fit, posture, timing, recovery, evaluation, and labeling judgment into explicit operating rules. No dataset-size, policy-success-rate, ablation, production-throughput, or deployment-readiness figure is claimed on this page.

The October 20 report's slides 4–6 and speaker notes document the OpenVLA work and the rollout observations summarized above. The named model and my role come from my account of the work. The recordings support the visible motion and object outcomes, while the model's internal reasoning and the causes of those behaviors remain open.

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

The scope includes AI-team OpenVLA manipulation experiments, inference and failure analysis, whole-body experimental control, and learning-pipeline development. Remaining technical risks include target selection, reliable recovery, gait and manipulation coordination, demonstration variance, sensor synchronization, hand-state mismatch, object diversity, real-hardware safety, and transfer from a controlled setup to deployment-like conditions.

The main lesson was that robot-learning quality is designed before training begins. Positive and no-op conditions, start/stop transitions, whole-body task structure, operator fit, disciplined teleoperation, synchronized sensing, automated FAIL/WARN/PASS validation, and the manual grade decision tree determine whether later model work starts from a useful signal or avoidable noise.
