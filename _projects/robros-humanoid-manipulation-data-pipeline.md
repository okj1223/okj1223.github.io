---
layout: project
date: 2026-04-13
featured: true
featured_order: 1
title: "Humanoid Manipulation Data and Learning Workflow Support for Warehouse Task Automation"
card_title: "Humanoid Learning Data Workflow for Warehouse Manipulation"
description: "A public-safe case study of my Robros research-assistant work across teleoperation, collection protocols, temporal labeling, dataset QA, and learning-workflow support for humanoid warehouse tasks."
card_description: "Research-assistant work at Robros connecting master-arm and MANUS teleoperation to structured collection, episode QA, and learning-ready humanoid manipulation data."
share-description: "Robros research-assistant work connecting humanoid teleoperation, structured data collection, episode QA, and robot-learning workflows."
subtitle: "Building the operational layer between human demonstration and learning-ready humanoid data."
thumbnail-img: /project/robros-humanoid-manipulation-data-pipeline/system-overview.webp
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
  - value: "2-stage"
    label: "Quality Gate"
  - value: "Multi-modal"
    label: "Episode Review"
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
quick_summary_note: "An operational case study of the data side of robot learning; customer details, internal tooling, dataset scale, and model performance remain non-public."
quick_summary:
  - label: "Role"
    value: "Research Assistant spanning teleoperation, collection protocol, dataset curation, QA, and learning support"
  - label: "Scope"
    value: "Upper-body humanoid soft-goods packing, box transfer, and tabletop pick-and-place protocols"
  - label: "Inputs"
    value: "Master-arm motion, MANUS finger articulation, robot state, multi-view vision, and episode metadata"
  - label: "Workflow"
    value: "Calibrate, collect, inspect, segment, grade, curate, and feed findings back into the next run"
  - label: "Status"
    value: "Internal research PoC; dataset scale, model details, and performance remain non-public"
---

## Context

Warehouse manipulation is difficult to reduce to a fixed sequence of robot commands. Packing deformable items depends on grasp shape and release timing. Moving boxes between storage and conveyor areas depends on approach pose, contact, object geometry, and consistent placement. Human demonstration is a practical starting point only when the demonstrations are synchronized, repeatable, and useful to learn from.

At Robros, I supported upper-body humanoid proof-of-concept work across three related task protocols:

- packing soft household items such as socks, shirts, and towels into boxes;
- transferring boxes from a storage rack to a conveyor;
- tabletop pick-and-place used to define consistent grasp, placement, and label boundaries.

The operational challenge was to preserve the right kind of variation. Object position and task conditions needed enough diversity for learning, while grasp conventions, motion order, episode boundaries, and safety procedure had to remain consistent across operators. The work remains an internal research PoC, so this page focuses on my responsibilities and a public-safe abstraction of the workflow.

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

<figure class="project-diagram" markdown="0">
  <a class="project-diagram-link"
     href="{{ '/project/robros-humanoid-manipulation-data-pipeline/system-overview.png' | relative_url }}"
     target="_blank"
     rel="noopener"
     aria-label="Open the humanoid manipulation data workflow at full resolution">
    <img class="flowchart"
         src="{{ '/project/robros-humanoid-manipulation-data-pipeline/system-overview.webp' | relative_url }}"
         alt="Closed-loop workflow from humanoid task protocol and teleoperation through synchronized capture, episode QA, dataset curation, and rollout review"
         width="2624"
         height="1632"
         loading="lazy"
         decoding="async">
  </a>
  <figcaption>Public-safe overview of the closed loop from task protocol and synchronized capture to episode QA, dataset curation, and learning feedback. Tap to open the full-resolution diagram.</figcaption>
</figure>

## My Role

I worked at the boundary between field operation and robot learning. My clearest ownership was the operational and data-quality layer of the loop. I directly contributed to:

- fitting and calibrating the master-arm interface to the operator;
- running teleoperation sessions with master-arm and MANUS glove input;
- preparing repeatable task setup, starting posture, and episode procedure;
- monitoring synchronized recording and ingestion state during collection;
- replaying multi-view episodes and comparing commanded motion with executed robot state;
- trimming, labeling, relabeling, grading, and filtering demonstrations;
- documenting failure and downgrade reasons so the next collection run could improve;
- supporting imitation-learning experiments and reinforcement-learning-oriented iteration.

I also converted repeated operating judgment into written guidance: equipment setup, initial posture, assistant movement, task sequencing, acceptance criteria, temporal labels, and recovery handling. This made the workflow less dependent on tacit knowledge held by one operator.

I do not claim sole ownership of the humanoid platform, final model architecture, training infrastructure, reinforcement-learning algorithm, reward design, or deployment decisions. My work made the data and operating procedure more useful to the engineers responsible for those layers.

## What Each Episode Contained

The demonstrations combined several kinds of information:

| Input | Purpose |
| --- | --- |
| Master-arm state | Captures the operator's gross upper-body motion intent |
| MANUS glove finger state | Captures grasp shape, opening, closure, and release timing |
| Robot joint and hand state | Records what the robot actually executed |
| Multi-view camera streams | Provides task-space, object, hand, and placement context |
| Episode metadata and reviewer notes | Preserves setup, protocol version, outcome, and QA decisions |

These streams needed a shared episode boundary and useful time alignment. A visually successful run could still become poor training data when camera, hand, and arm states described different moments, or when the recording itself was incomplete.

## Collection Protocol and Operator Setup

Before recording, the operator and assistant worked from the same task definition: starting posture, object setup, movement order, completion condition, and safety responsibilities. The master-arm linkage was adjusted to the operator's shoulder, upper-arm, and forearm geometry, then checked in neutral and extended poses. This reduced mapping error caused by fit rather than task execution.

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

Finger articulation was captured with MANUS input when grasp shape mattered. The operator maintained the agreed hand pose and motion sequence; the assistant managed the workspace, supported setup changes, and kept immediate access to emergency-stop controls. This division reduced avoidable interruptions and made episode boundaries more repeatable.

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

Each collection cycle followed the same operational sequence:

1. prepare the task and confirm the protocol version;
2. fit and initialize the operator interfaces, robot, safety controls, and camera streams;
3. align the system in a known starting pose;
4. record one continuous episode from task start to finish;
5. verify that capture completed and the episode moved through the ingestion pipeline;
6. replay, segment, grade, and document the episode;
7. hand curated data into the learning workflow;
8. use review and rollout findings to tighten the next collection run.

The assistant's role was not incidental: synchronized starts, clear workspace handling, and immediate access to safety controls were part of a valid episode.

### Recording state was part of the protocol

A saved file was not automatically a usable episode. I monitored the recorder lifecycle from local capture through **queued**, **in flight**, **registered**, or **failed** states. A run could proceed to review only after its streams and metadata were present and the episode was registered. This separated task failures from recorder or transfer failures before anyone judged behavior.

<div class="project-flow" role="list" aria-label="Episode lifecycle from preparation to learning handoff" markdown="0">
  <div role="listitem"><span>01</span><strong>Preflight</strong><small>Robot · teleop · cameras · storage</small></div>
  <div role="listitem"><span>02</span><strong>Record</strong><small>One continuous task episode</small></div>
  <div role="listitem"><span>03</span><strong>Register</strong><small>Confirm streams and metadata</small></div>
  <div role="listitem"><span>04</span><strong>Review</strong><small>Trim · label · grade · diagnose</small></div>
  <div role="listitem"><span>05</span><strong>Curate</strong><small>Filter and hand off to learning</small></div>
</div>

## Protocol Design: Consistency Without Uniformity

The goal was not to make every trajectory numerically identical. It was to keep task semantics stable while allowing controlled diversity.

| Hold consistent | Vary deliberately |
| --- | --- |
| Initial posture and system alignment | Object position within the allowed workspace |
| Grasp convention and wrist orientation | Object type, geometry, or box placement within protocol |
| Task order and semantic completion condition | Natural operator trajectory within the agreed motion intent |
| Episode start/stop and labeling rules | Repeated coverage of valid task configurations |
| Assistant behavior and intervention rules | Protocol-approved setup variation for learning diversity |

For tabletop pick-and-place, operators first agreed on how each object should be grasped. Object and target position could then move within a defined range. That created meaningful variation without turning inconsistent grasp choice into label noise.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/tabletop-pick-and-place.webp' | relative_url }}"
       alt="Robot hand beside a bottle during a tabletop pick-and-place setup"
       width="1200"
       height="900"
       loading="lazy"
       decoding="async">
  <figcaption>Representative tabletop setup used to reason about grasp consistency, object-position variation, and clear pick/place boundaries.</figcaption>
</figure>

## Task Decomposition and Temporal Labeling

Long demonstrations were divided into task-relevant phases instead of being treated as one opaque trajectory. A representative transfer episode could contain approach, lift, retreat, rotate, place, and return phases. Clear boundaries made episodes easier to compare and enabled phase-level filtering or analysis.

<figure class="project-evidence-figure" markdown="0">
  <img src="{{ '/project/robros-humanoid-manipulation-data-pipeline/episode-subtask-segmentation.webp' | relative_url }}"
       alt="Example motion traces divided into step forward, lift, step backward, rotate, place, and return phases"
       width="1800"
       height="1090"
       loading="lazy"
       decoding="async">
  <figcaption>Representative episode segmentation. The graph helps inspect motion, while the task video determines the semantic boundary when operator traces differ.</figcaption>
</figure>

Numeric traces were useful evidence, but they were not enough by themselves. Different operators could produce different motor profiles for the same successful action. I therefore used the synchronized camera view to determine the semantic event—such as the first stable lift or completed placement—and used state traces to confirm, refine, or diagnose that decision.

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

The review workflow made an important distinction that a single success/fail label would hide. I treated episode review as two gates:

1. **Recording integrity:** Is the episode complete, synchronized, and reviewable? Corrupted or incomplete records are separated before task grading.
2. **Execution quality:** For a reviewable episode, is the behavior suitable for the intended learning use?

The second gate used a practical three-level outcome:

| Label | Interpretation |
| --- | --- |
| Clean Success | Completed with behavior consistent with the collection protocol |
| Dirty Success | Completed, but with a recovery or deviation that requires case-by-case use |
| Fail | Violated the task or data-quality criteria |

The middle category prevented a forced binary decision. It kept marginal episodes available for analysis without silently mixing them into the highest-confidence training set. A separate pending state allowed ambiguous cases to be held for review instead of being guessed.

<div class="project-qa-grid" markdown="0">
  <div class="project-qa-card is-corrupted"><span>Integrity gate</span><strong>Corrupted</strong><p>Missing, incomplete, unsynchronized, or otherwise unreviewable recording.</p></div>
  <div class="project-qa-card is-fail"><span>Execution gate</span><strong>Fail</strong><p>Task or protocol criteria were violated.</p></div>
  <div class="project-qa-card is-dirty"><span>Execution gate</span><strong>Dirty Success</strong><p>Completed with recovery or deviation; retain only for case-by-case use.</p></div>
  <div class="project-qa-card is-success"><span>Execution gate</span><strong>Clean Success</strong><p>Completed with behavior consistent with the collection protocol.</p></div>
</div>

### Review video, action, and observation together

Task video answered what happened in the workspace. Commanded **action** state represented the operator's intended target; **observation** represented what the robot actually executed. Comparing all three helped separate an operator or protocol issue from a robot-side tracking mismatch, recording fault, or environmental interference.

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

Typical downgrade or rejection reasons included external intervention, incorrect hand opening, unstable finger posture, inconsistent release timing, deviation from the planned sequence, unintended recovery, or action–observation mismatch. Recording a specific reason made the result useful for both dataset curation and the next operating session.

### Edit at the episode and subtask level

Review was not limited to assigning one label. I used frame navigation to mark subtask endpoints, revise labels, trim invalid leading or trailing segments, and remove a selected corrupted range when the remaining episode was still meaningful. The purpose was traceability: every change had to preserve why a sample was accepted, downgraded, repaired, or rejected.

### Treat hand articulation as first-class data

For grasp-sensitive tasks, the arm path is not enough. A delayed opening, asymmetric closure, or inconsistent release can invalidate an otherwise plausible trajectory. Capturing and reviewing glove and robot-hand state made those cases visible.

## Evidence and Outcome

My contribution established a more disciplined path from raw teleoperation to learning-ready data. The work connected operator calibration, task protocol, synchronized multi-modal recording, recorder-state monitoring, temporal labels, a two-stage quality gate, action–observation diagnosis, targeted trimming, filtering, and downstream experiment feedback.

The outcome claimed here is operational rather than a model benchmark: I helped make demonstrations easier to reproduce, inspect, compare, diagnose, curate, and reuse across three task protocols. The documentation also turned tacit operator knowledge—fit, posture, timing, recovery, and labeling judgment—into a workflow that another contributor could follow. No dataset-size, policy-success-rate, ablation, production-throughput, or deployment-readiness figure is claimed on this page.

## Public Media Appearances

These are external media features rather than project demo videos or technical evaluations. I appear on camera as a **teleoperation operator**, using the master-arm interface to demonstrate the humanoid manipulation workflow described on this page.

<div class="project-media-grid" markdown="0">
  <a class="project-media-card"
     href="https://www.youtube.com/watch?v=YJqSagsb_yI&amp;t=595s"
     target="_blank"
     rel="noopener noreferrer">
    <span class="project-media-source">SBS News Story · Broadcast Feature</span>
    <strong>당신 옆자리에 AI가 출근했다…로봇이 대신 일해 준다는데 뭐가 걱정?</strong>
    <span class="project-media-role">Role: on-camera teleoperation operator demonstrating the master-arm workflow.</span>
    <span class="project-media-cta">Watch the ROBROS segment on YouTube <span aria-hidden="true">↗</span></span>
  </a>

  <a class="project-media-card"
     href="https://www.youtube.com/watch?v=CTDoi9T0g44"
     target="_blank"
     rel="noopener noreferrer">
    <span class="project-media-source">Ministry of Trade, Industry and Energy · Field Feature</span>
    <strong>여러분과 함께 일하러 왔습니다! 사람처럼 걷고 일하는 휴머노이드 실물 직관</strong>
    <span class="project-media-role">Role: on-camera teleoperation operator supporting the humanoid task demonstration.</span>
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

The current scope is upper-body research rather than a claim of autonomous warehouse deployment. Remaining technical risks include demonstration variance, sensor synchronization, hand-state mismatch, object diversity, real-hardware safety, and transfer from a controlled setup to deployment-like conditions.

The main lesson was that robot-learning quality is built before training begins. Operator fit, clear task setup, disciplined teleoperation, synchronized sensing, semantic labels, and explicit review criteria determine whether later model work starts from a useful signal or avoidable noise.
