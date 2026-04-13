---
layout: project
date: 2026-04-13
title: "Humanoid Manipulation Data Collection and Learning Support for Warehouse Task Automation"
description: "A public portfolio summary of my research assistant work at Robros, covering teleoperated data collection, dataset QA, and experiment support for humanoid box-packing and box-transfer tasks."
share-description: "Research assistant work at Robros on teleoperated data collection, dataset QA, and learning workflow support for humanoid warehouse manipulation tasks."
subtitle: "Research assistant work at Robros on teleoperated data collection, dataset QA, and learning workflow support for humanoid warehouse manipulation tasks."
thumbnail-img: /project/robros-humanoid-manipulation-data-pipeline/system-overview.svg
permalink: /projects/robros-humanoid-manipulation-data-pipeline/
---

# Humanoid Manipulation Data Collection and Learning Support for Warehouse Task Automation

## Abstract

This portfolio paper summarizes my ongoing Research Assistant work at Robros, where I have supported early humanoid manipulation PoCs for logistics-oriented warehouse tasks since **March 5, 2026**. The project scope has centered on two upper-body task families: **packing soft household items into boxes** and **transferring boxes from a rolltainer to a conveyor belt**. These tasks were used as practical benchmarks for instruction-conditioned robot behavior learning in settings where demonstration quality, consistency, and operational realism mattered as much as raw task completion.

My primary contribution was not ownership of a single model, but ownership of the **data side of the learning loop**: teleoperated demonstration collection, initial labeling and relabeling, dataset inspection, quality-criteria design, failure-case review, bad-sample filtering, and workflow support for imitation-learning-centered experimentation with reinforcement-learning-aware iteration. Because this project remains in an internal PoC phase, this write-up focuses on public-safe engineering structure and verified responsibilities, while leaving unreleased model details generalized.

![Conceptual system overview for teleoperated humanoid warehouse manipulation learning]({{ '/project/robros-humanoid-manipulation-data-pipeline/system-overview.svg' | relative_url }}){: .flowchart}

*Figure 1. Conceptual system overview of the teleoperation-to-learning workflow used for humanoid warehouse manipulation PoCs. This diagram is a public-safe abstraction rather than an internal architecture export.*

## 1. Background and Operational Context

### 1.1 Deployment Motivation

A key challenge in warehouse automation is that many high-value manipulation tasks remain difficult to hard-code. Tasks such as packing deformable objects into a box or transferring boxes of varying geometry involve:

- variable grasp geometry,
- viewpoint-dependent perception,
- non-rigid or semi-structured object behavior,
- and sensitivity to operator style during demonstration.

For that reason, the work was framed around **learning from human demonstration** rather than relying only on deterministic scripted manipulation.

### 1.2 Platform Scope

The internal PoC work used two Robros humanoid platforms:

- **IGRIS-B** for upper-body box-packing tasks
- **IGRIS-C** for upper-body box-transfer tasks

At the public-portfolio level, the task distinction is more important than exposing unreleased robot internals. The practical split was:

| Platform | Primary Task | Operational Characteristic | Portfolio Note |
| --- | --- | --- | --- |
| IGRIS-B | Item-to-box packing | Handling soft items such as socks, T-shirts, and towels | Upper-body manipulation PoC |
| IGRIS-C | Box transfer | Moving differently shaped boxes from rolltainer to conveyor | Upper-body logistics-transfer PoC |

The public positioning of the IGRIS line is consistent with this task framing: Robros presents IGRIS-C as a compact AI humanoid platform for real-world industrial environments, while IGRIS-B has appeared in public company materials as a dual-arm system used for imitation-learning-driven box-packing demonstrations.

### 1.3 Project Status

At the time of writing, the project remains in an **internal PoC and research phase**. The immediate scope is upper-body manipulation, while the long-term target application is deployment in logistics-company workflows.

## 2. Problem Definition and Learning Objective

The practical goal was to support policies that could map perception and task context into useful warehouse-style manipulation behavior.

At a high level, the target behavior can be expressed as:

$$
\pi(a_t \mid o_t, q_t, c)
$$

where:

- $o_t$ represents multi-view visual observations,
- $q_t$ represents robot and teleoperation state information,
- $c$ represents task or instruction context,
- and $a_t$ represents the action to be executed by the robot.

One representative example was a VLM-style intent such as: **"pick up the sock and place it into the box."** The learning challenge was not only to produce a successful end state, but to produce demonstrations and datasets that were sufficiently consistent to support trainable robot behavior under real hardware constraints.

The project therefore treated **data quality** as a first-class systems problem rather than a post-processing detail.

## 3. Task Matrix

Two task families were emphasized.

### 3.1 Object-to-Box Packing

This task focused on picking and placing deformable or semi-structured objects into a box. Representative objects included:

- socks,
- T-shirts,
- towels,
- and similar household items.

The main difficulty was not the box itself, but the object variability. Deformable items are especially sensitive to grasp pose, hand opening quality, and demonstration consistency.

### 3.2 Box Transfer from Rolltainer to Conveyor

This task focused on moving boxes with different sizes or shapes from a rolltainer to a conveyor belt. Compared with item packing, the box-transfer task introduced a different difficulty profile:

- larger rigid-body motion,
- stronger dependence on approach pose,
- contact-sensitive placement,
- and potentially higher actuation demand.

These two task tracks together provided a useful testbed for comparing learning readiness across soft-object manipulation and logistics-transfer behavior.

## 4. Teleoperated Data Collection Pipeline

### 4.1 Collection Method

The primary data source was **human teleoperation using a master-arm setup**. An operator wore or used the teleoperation interface to generate demonstrations while the robot executed the corresponding motion.

My responsibilities in this stage included:

- carrying out data collection sessions,
- helping define repeatable collection procedures,
- and improving consistency across runs.

### 4.2 Recorded Modalities

The main logged modalities were:

| Modality | Source | Role in Learning Workflow | Public Status |
| --- | --- | --- | --- |
| Master-arm joint values | Teleoperation interface | Capture operator intent and demonstration structure | Confirmed |
| Robot joint angles | Robot-side state logging | Track executed embodiment state | Confirmed |
| Multi-view image streams | Robot-mounted cameras | Provide visual context for manipulation | Confirmed |
| Motor current values | Robot actuators | Candidate future signal for high-load tasks | Planned / generalized |

In practice, the image inputs included multiple viewpoints such as:

- left and right hand-adjacent or operator-relevant views,
- stereo or eye-level views,
- and a head-to-downward view that provided task-centric workspace visibility.

### 4.3 Dataset Scale

Based on my working memory and collection involvement, the project involved approximately:

- **about 10,000 collection attempts** overall, and
- **roughly 300 episodes per object category** in some task settings.

These values should be treated as **[confirm later]** before any public-facing final release.

To make the collection structure more explicit, the practical pipeline can be summarized as:

1. Define task protocol and operator procedure.
2. Collect demonstrations through master-arm teleoperation.
3. Record joint-state and multi-view visual observations.
4. Inspect episodes using the clean/dirty/fail taxonomy.
5. Filter or relabel borderline trajectories.
6. Pass curated subsets into downstream learning experiments.

## 5. Inspection, Labeling, and Dataset QA

### 5.1 Why QA Mattered

In learning-based robot manipulation, not every successful run is equally useful. A trajectory can finish the task while still introducing noise that weakens downstream training. Because of that, the dataset pipeline required a structured acceptance standard rather than a binary pass/fail view.

### 5.2 Three-Level Outcome Taxonomy

I supported and applied a practical three-level labeling system:

| Label | Meaning | Training Value |
| --- | --- | --- |
| **Clean Success** | Task completed as intended with high procedural consistency | High |
| **Dirty Success** | Task completed but with inconsistencies, recoveries, or deviations | Medium / case-dependent |
| **Fail** | Task violated collection criteria or created low-trust data | Low |

This taxonomy preserved information that would be lost in a simple success/failure label.

### 5.3 Failure and Downgrade Conditions

Typical downgrade or rejection triggers included:

- another person interfering during the demonstration,
- the robot hand failing to open correctly,
- the robot deviating from the intended collection design,
- and other irregular execution patterns that damaged consistency.

At a conceptual level, the QA function can be described as:

$$
L(e) \in \{\text{clean}, \text{dirty}, \text{fail}\}
$$

where the episode label $L(e)$ depends on:

- task completion,
- intervention-free execution,
- hand-state validity,
- and trajectory consistency with the intended collection standard.

This formula is a conceptual representation of the decision process, not a deployed scoring model.

### 5.4 My Direct Responsibilities in QA

I directly contributed to:

- initial labeling,
- relabeling and quality review,
- quality-criteria design,
- error-case documentation,
- bad-sample removal,
- and collection-method refinement.

That work helped convert raw demonstrations into a more learning-ready dataset.

## 6. Imitation Learning Workflow Support

### 6.1 Role Definition

My clearest ownership was on the imitation-learning support side. I do **not** claim sole ownership of training code, final model architecture, or production policy optimization. Instead, my work focused on making the demonstration data more usable for those stages.

### 6.2 Practical Support Areas

I directly or materially supported the following:

- teleoperated demonstration collection,
- consistent task repetition across sessions,
- separation of high-quality versus marginal trajectories,
- and dataset structuring for instruction-conditioned or task-conditioned learning workflows.

### 6.3 Why Imitation Learning Was a Strong Fit

The early PoC stage favored imitation learning because the target tasks were human-demonstrable:

- grasping,
- placing,
- moving boxes,
- and executing visually guided upper-body manipulation.

In these settings, demonstration-driven learning provided a practical way to bootstrap behavior without requiring fully specified control logic for every contact condition.

### 6.4 Public-Safe Illustrative Source Snippets

The following examples are **illustrative public-safe snippets**, not internal Robros source code. They are included to make the workflow more concrete while preserving confidential implementation details.

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example ROS2 collection command for a multi-view teleoperation session</button>
<div class="code-toggle-panel">
<pre><code class="language-bash">ros2 bag record \
  /master_arm/joint_states \
  /humanoid/joint_states \
  /humanoid/left_hand/image_raw \
  /humanoid/right_hand/image_raw \
  /humanoid/stereo/left/image_raw \
  /humanoid/stereo/right/image_raw \
  /humanoid/head_down/image_raw \
  /task/context \
  /episode/metadata</code></pre>
</div>
</div>

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example episode QA labeling logic</button>
<div class="code-toggle-panel">

<pre><code class="language-python">def label_episode(run):
    if run.external_intervention:
        return "fail"
    if not run.hand_open_ok:
        return "fail"
    if not run.protocol_compliant:
        return "fail"
    if run.task_success and run.motion_consistency &gt;= 0.95:
        return "clean_success"
    if run.task_success:
        return "dirty_success"
    return "fail"</code></pre>
</div>
</div>

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example curated episode manifest for policy training</button>
<div class="code-toggle-panel">

<pre><code class="language-yaml">episode_id: 000231
task: place_sock_in_box
platform: igris_b
inputs:
  cameras:
    - left_hand_rgb
    - right_hand_rgb
    - stereo_left
    - stereo_right
    - head_down_rgb
  states:
    - master_arm_joint
    - robot_joint
labels:
  outcome: clean_success
  reviewer: public_portfolio_example
  protocol_version: v1
notes:
  - no external intervention
  - hand opening valid
  - trajectory accepted for training</code></pre>
</div>
</div>

### 6.5 Public Reference Implementations

To situate this work within the broader robot-learning ecosystem, the public references below are useful parallels for data recording, demonstration learning, and simulation-based policy iteration:

- ROS 2 data recording tutorial: [Recording and playing back data](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- `rosbag2` official repository: [ros2/rosbag2](https://github.com/ros2/rosbag2)
- Hugging Face LeRobot: [huggingface/lerobot](https://github.com/huggingface/lerobot)
- Robomimic demonstration-learning framework: [ARISE-Initiative/robomimic](https://github.com/ARISE-Initiative/robomimic)
- Isaac Lab reference architecture: [Isaac Lab Documentation](https://docs.robotsfan.com/isaaclab_official/main/source/refs/reference_architecture/index.html)

## 7. Reinforcement-Learning-Oriented Experiment Support

My involvement on the reinforcement-learning side was more indirect and should be described carefully. I was aware of and supported a **hybrid imitation-learning and reinforcement-learning direction**, but I do not claim ownership of the exact RL algorithm, reward implementation, or final training loop.

At the public-summary level, my support is best described in three ways:

1. I helped prepare and curate the demonstrations that could serve as a behavior prior.
2. I helped structure quality labels that can support iterative experiment analysis.
3. I supported a workflow in which imitation learning and reinforcement-oriented refinement were considered complementary rather than isolated.

Exact details for reward design, simulator structure, or RL fine-tuning policy are **[confirm later]** and are intentionally generalized here.

## 8. Evaluation and Validation Strategy

Formal benchmark reporting was still evolving, so the most meaningful evaluation criteria at this stage were operational rather than publication-style leaderboard metrics.

### 8.1 Dataset-Level Validation

The first validation layer asked:

- Was the task completed?
- Was the behavior consistent enough to be learnable?
- Was the episode clean, dirty, or fail?
- Could the collection protocol be reused across sessions?

### 8.2 Experiment-Level Validation

At a practical level, the downstream learning workflow cared about:

- repeatability of demonstration quality,
- reduction of noisy samples,
- task-wise coverage across object categories,
- and whether the data pipeline supported stable experimentation.

Exact quantitative KPIs, success-rate deltas, or ablation outcomes remain **[confirm later]**.

Even without finalized benchmark tables, the evaluation stack was already useful because it linked operator behavior, collection quality, and learning readiness into a single workflow rather than treating them as separate problems.

## 9. Technical Constraints and Safety Considerations

Several real-world constraints shaped the work:

| Constraint | Why It Matters |
| --- | --- |
| Upper-body-only scope | PoCs focused on manipulation before full-body autonomy |
| Demonstration variance | Operator style changes can distort policy learning |
| Hand-state instability | Incomplete opening or grasp irregularity reduces sample value |
| Real-robot mismatch | Intended motion and executed motion do not always align perfectly |
| Task diversity | Soft objects and variable boxes create broad behavior variation |

These constraints reinforced a central engineering lesson: **data collection protocol is part of the robot system design**.

## 10. Contributions and Engineering Impact

Even without claiming final-model ownership, this work contributed to a more disciplined learning pipeline by:

- establishing repeatable teleoperation-based collection procedures,
- introducing a useful three-level QA taxonomy,
- documenting failure modes before they polluted downstream training,
- filtering out low-value episodes,
- and aligning dataset practice with realistic warehouse-manipulation objectives.

I see this as foundational systems work: building the operational layer that makes later policy learning more reliable.

## 11. Lessons Learned

Three lessons became especially clear during the project.

### 11.1 Success Is Not the Same as Learnability

A demonstration that barely succeeds may still be a poor training sample if it includes unstable recovery, interference, or inconsistent motion.

### 11.2 QA Categories Preserve Training-Relevant Information

The distinction between clean success, dirty success, and fail is practically valuable because it preserves nuance that matters for data curation and later analysis.

### 11.3 Real Robot Learning Depends on Procedure Discipline

Operator discipline, task setup, and sample review are as important as model choice when moving from internal PoCs toward deployment-oriented robot learning.

## 12. Future Directions

The next technical steps I see for this work include:

- expanding beyond upper-body-only PoCs,
- incorporating richer actuator-side signals such as motor current for force-sensitive tasks,
- tightening the link between dataset QA and policy evaluation,
- improving transfer from internal research setups to deployment-like logistics environments,
- and formalizing the evaluation stack for hybrid imitation-learning and reinforcement-learning workflows.

As this project matures, its long-term value will likely come not only from one successful demo, but from a scalable process for collecting, curating, and validating robot learning data in real warehouse tasks.
