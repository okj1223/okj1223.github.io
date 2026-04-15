---
layout: project
date: 2026-04-13
title: "Humanoid Manipulation Data and Learning Workflow Support for Warehouse Task Automation"
card_title: "Multi-Modal Humanoid Learning Pipeline for Warehouse Manipulation"
description: "A public portfolio summary of my research assistant work at Robros on master-arm and glove-based teleoperation, dataset QA, and hybrid imitation-learning / reinforcement-learning workflow support for humanoid warehouse manipulation."
card_description: "Research assistant work at Robros on master-arm and MANUS glove teleoperation, dataset QA taxonomy design, and hybrid imitation-learning / reinforcement-learning workflow support for humanoid packing and box-transfer PoCs."
share-description: "Research assistant work at Robros on master-arm and glove-based teleoperation, dataset QA, and hybrid imitation-learning / reinforcement-learning support for humanoid warehouse manipulation."
subtitle: "Research assistant work at Robros on master-arm and glove-based teleoperation, dataset QA, and hybrid imitation-learning / reinforcement-learning support for humanoid warehouse manipulation."
thumbnail-img: /project/robros-humanoid-manipulation-data-pipeline/system-overview.svg
permalink: /projects/robros-humanoid-manipulation-data-pipeline/
filter_categories:
  - robotics
  - ai
category_label: "Humanoid Robotics - Imitation Learning - Logistics AI"
impacts:
  - value: "~10k"
    label: "Collection Attempts"
  - value: "3-tier"
    label: "QA Taxonomy"
  - value: "2"
    label: "Task Families"
tech_tags:
  - label: "Humanoid"
    style: "robot-hw"
  - label: "Teleoperation"
    style: "eng"
  - label: "MANUS Glove"
    style: "sensor"
  - label: "Imitation Learning"
    style: "ai"
  - label: "VLM Conditioning"
    style: "ai"
  - label: "Dataset QA"
    style: "data"
  - label: "Robot Vision"
    style: "sensor"
  - label: "ROS2 Workflow"
    style: "prog ros"
  - label: "Logistics"
    style: "system"
quick_summary_note: "A compact operational summary for a research-oriented project that spans teleoperation, dataset curation, imitation learning, and runtime execution."
quick_summary:
  - label: "Role"
    value: "Research Assistant supporting data collection, QA, and learning workflow design"
  - label: "Scope"
    value: "Humanoid box packing and box transfer tasks for logistics-oriented PoCs"
  - label: "Inputs"
    value: "Master-arm teleoperation, MANUS glove finger joints, robot state, and multi-view vision"
  - label: "Learning"
    value: "Imitation learning first, with reinforcement-oriented refinement in a hybrid workflow"
  - label: "Current Stage"
    value: "Upper-body internal PoC for warehouse task automation"
---

# Humanoid Manipulation Data and Learning Workflow Support for Warehouse Task Automation

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

The primary data source was **human teleoperation**, but in practice the setup was richer than a simple arm-mirroring loop. Demonstrations were collected through a combination of:

- a **master-arm interface** for upper-body motion,
- robot-side joint-state logging for executed embodiment tracking,
- and, in relevant sessions, a **MANUS glove-based hand interface** for finger articulation and grasp-shape capture.

That distinction matters. For logistics manipulation, a task can fail not only because the arm trajectory is wrong, but because **hand posture timing** is wrong: fingers open too late, close asymmetrically, or do not form the intended grasp geometry before contact. The glove input therefore provided a more realistic bridge between human demonstration and robot hand behavior than arm joints alone.

My responsibilities in this stage included:

- carrying out teleoperated data-collection sessions,
- helping define repeatable collection procedures,
- refining how demonstrations were segmented and logged,
- and improving consistency across operators, sessions, and task conditions.

### 4.2 Recorded Modalities

The practical collection setup combined perception, operator-side motion signals, and robot-side embodiment state:

| Modality | Source | Role in Learning Workflow | Public Status |
| --- | --- | --- | --- |
| Master-arm joint values | Teleoperation interface | Capture operator arm intent and gross motion structure | Confirmed |
| MANUS glove finger-joint signals | Hand teleoperation interface | Capture grasp timing and finger articulation | Confirmed |
| Robot joint angles | Robot-side state logging | Track executed upper-body embodiment state | Confirmed |
| Robot hand / finger states | Robot-side hand articulation state | Compare commanded vs. executed hand posture | Generalized but aligned with workflow |
| Multi-view image streams | Robot-mounted cameras | Provide visual context for reach, grasp, and placement | Confirmed |
| Task / episode metadata | Collection protocol and reviewer notes | Enable filtering, replay, and downstream experiment grouping | Generalized but aligned with workflow |
| Motor current values | Robot actuators | Candidate future signal for force-sensitive tasks | Planned / generalized |

In practice, the image inputs included:

- left and right hand-adjacent views,
- stereo or eye-level views,
- and a head-to-downward task-space view for workspace visibility.

Taken together, the observation stream was closer to a structured tuple than to a single image:

$$
x_t = \{I_t^{multi}, q_t^{master}, q_t^{robot}, h_t^{glove}, h_t^{robot}, c_t\}
$$

where:

- $I_t^{multi}$ denotes synchronized multi-view images,
- $q_t^{master}$ denotes operator arm-state signals,
- $q_t^{robot}$ denotes robot joint states,
- $h_t^{glove}$ denotes glove-derived finger-articulation signals,
- $h_t^{robot}$ denotes robot hand states,
- and $c_t$ denotes task metadata or instruction context.

### 4.3 Episode Structure and Synchronization

One practical challenge in this kind of work is that robot-learning data is only as useful as its time alignment. Multi-view images, master-arm motion, glove-derived finger states, and robot embodiment data all need to agree on **which moment in the demonstration** they represent.

At a public-safe level, the episode schema can be summarized as:

1. initialize task metadata and protocol version,
2. start synchronized recording for arm, hand, and camera streams,
3. collect the teleoperated demonstration,
4. mark terminal outcome and reviewer notes,
5. replay the run for QA,
6. assign `clean_success`, `dirty_success`, or `fail`,
7. export only curated episodes to learning-ready storage.

This synchronization step became even more important once finger-joint information was included, because hand posture often changes at a faster and more contact-sensitive timescale than gross arm motion.

### 4.4 Dataset Scale and Working Volume

Based on my working memory and collection involvement, the project involved approximately:

- **about 10,000 collection attempts** overall, and
- **roughly 300 episodes per object category** in some task settings.

These values should be treated as **[confirm later]** before any final public release.

### 4.5 Public-Safe Collection Snippets

The following examples are **illustrative public-safe snippets**, not internal Robros source code. They are included to show the engineering shape of the workflow while preserving confidential implementation details.

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example ROS2 recording command for master-arm, glove, robot-state, and multi-view camera topics</button>
<div class="code-toggle-panel">
<pre><code class="language-bash">ros2 bag record \
  /master_arm/left/joint_states \
  /master_arm/right/joint_states \
  /manus/left/finger_joints \
  /manus/right/finger_joints \
  /igris/joint_states \
  /igris/hands/joint_states \
  /igris/camera/left_hand/image_raw \
  /igris/camera/right_hand/image_raw \
  /igris/camera/stereo_left/image_raw \
  /igris/camera/stereo_right/image_raw \
  /igris/camera/head_down/image_raw \
  /task/context \
  /episode/marker \
  /episode/reviewer_note</code></pre>
</div>
</div>

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example synchronized episode packet builder with finger-joint support</button>
<div class="code-toggle-panel">
<pre><code class="language-python">from dataclasses import dataclass


@dataclass
class EpisodeFrame:
    timestamp: float
    left_hand_rgb: object
    right_hand_rgb: object
    stereo_left: object
    stereo_right: object
    head_down_rgb: object
    master_arm_joint: list[float]
    robot_joint: list[float]
    glove_finger_joint: list[float]
    robot_finger_joint: list[float]
    instruction: str


def build_episode_packet(sync_buffer, instruction):
    frames = []
    for sample in sync_buffer:
        frame = EpisodeFrame(
            timestamp=sample.timestamp,
            left_hand_rgb=sample.images["left_hand"],
            right_hand_rgb=sample.images["right_hand"],
            stereo_left=sample.images["stereo_left"],
            stereo_right=sample.images["stereo_right"],
            head_down_rgb=sample.images["head_down"],
            master_arm_joint=sample.master_arm_joint,
            robot_joint=sample.robot_joint,
            glove_finger_joint=sample.glove_finger_joint,
            robot_finger_joint=sample.robot_finger_joint,
            instruction=instruction,
        )
        frames.append(frame)
    return {"frames": frames, "num_frames": len(frames), "instruction": instruction}</code></pre>
</div>
</div>

## 5. Inspection, Labeling, and Dataset QA

### 5.1 Why QA Mattered

In learning-based robot manipulation, not every successful run is equally useful. A trajectory can finish the task while still introducing noise that weakens downstream training. Because of that, the dataset pipeline required a structured acceptance standard rather than a binary pass/fail view.

This became even more important once hand articulation entered the loop. A run could appear visually successful while still containing:

- unstable finger closure,
- delayed release timing,
- inconsistent grasp posture,
- or corrective recovery motions that a policy should not necessarily imitate.

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
- finger posture or release timing deviating from the intended collection pattern,
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
- finger-articulation consistency,
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

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example QA labeling logic that includes protocol adherence and finger-state consistency</button>
<div class="code-toggle-panel">
<pre><code class="language-python">def label_episode(run):
    if run.external_intervention:
        return "fail"
    if not run.hand_open_ok:
        return "fail"
    if not run.finger_state_consistent:
        return "dirty_success" if run.task_success else "fail"
    if not run.protocol_compliant:
        return "fail"
    if run.task_success and run.motion_consistency &gt;= 0.95:
        return "clean_success"
    if run.task_success:
        return "dirty_success"
    return "fail"</code></pre>
</div>
</div>

## 6. Imitation Learning Workflow Support

### 6.1 Role Definition

My clearest ownership was on the imitation-learning support side. I do **not** claim sole ownership of training code, final model architecture, or production policy optimization. Instead, my work focused on making the demonstration data more usable for those stages and on helping structure experiments around learnable behavior.

### 6.2 Learning Formulation

At a public-safe level, the imitation-learning formulation can be described as learning a policy from synchronized observation-action pairs:

$$
\mathcal{D} = \{(x_t, a_t)\}_{t=1}^{T}
$$

with:

- observation $x_t$: multi-view images, arm-state signals, hand-state signals, and task context,
- action $a_t$: robot arm command, hand / finger command, or equivalent control target.

For the kinds of tasks I worked around, this structure is more realistic than an image-only setup. Packing a sock into a box or transferring a box is not just about "where the object is" but also about:

- how the hand is shaped before contact,
- when release happens,
- whether the arm path remains consistent,
- and whether the demonstration preserves a repeatable strategy.

### 6.3 Why Imitation Learning Was a Strong Fit

The early PoC stage favored imitation learning because the target tasks were human-demonstrable:

- grasping,
- placing,
- moving boxes,
- and executing visually guided upper-body manipulation.

In these settings, demonstration-driven learning provided a practical way to bootstrap behavior without requiring fully specified control logic for every contact condition.

### 6.4 Practical Support Areas

I directly or materially supported the following:

- teleoperated demonstration collection,
- consistent task repetition across sessions,
- separation of high-quality versus marginal trajectories,
- incorporation of hand-articulation information for grasp-sensitive tasks,
- and dataset structuring for instruction-conditioned or task-conditioned learning workflows.

### 6.5 Public-Safe Illustrative Source Snippets

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example dataset sample builder for multi-view imitation learning with glove-derived finger joints</button>
<div class="code-toggle-panel">
<pre><code class="language-python">import torch


def make_bc_sample(frame):
    image_dict = {
        "left_hand_rgb": torch.as_tensor(frame.left_hand_rgb).float() / 255.0,
        "right_hand_rgb": torch.as_tensor(frame.right_hand_rgb).float() / 255.0,
        "stereo_left": torch.as_tensor(frame.stereo_left).float() / 255.0,
        "stereo_right": torch.as_tensor(frame.stereo_right).float() / 255.0,
        "head_down_rgb": torch.as_tensor(frame.head_down_rgb).float() / 255.0,
    }

    state = torch.tensor(
        frame.master_arm_joint
        + frame.robot_joint
        + frame.glove_finger_joint
        + frame.robot_finger_joint,
        dtype=torch.float32,
    )

    target_action = torch.tensor(
        frame.robot_joint + frame.robot_finger_joint,
        dtype=torch.float32,
    )

    return {
        "images": image_dict,
        "state": state,
        "instruction": frame.instruction,
        "action": target_action,
    }</code></pre>
</div>
</div>

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example behavior cloning training step for arm and hand action prediction</button>
<div class="code-toggle-panel">
<pre><code class="language-python">import torch.nn.functional as F


def bc_train_step(model, batch, optimizer):
    pred_action = model(
        images=batch["images"],
        state=batch["state"],
        instruction=batch["instruction"],
    )

    target_action = batch["action"]

    arm_dim = model.arm_action_dim
    pred_arm, pred_hand = pred_action[:, :arm_dim], pred_action[:, arm_dim:]
    tgt_arm, tgt_hand = target_action[:, :arm_dim], target_action[:, arm_dim:]

    arm_loss = F.smooth_l1_loss(pred_arm, tgt_arm)
    hand_loss = F.smooth_l1_loss(pred_hand, tgt_hand)
    loss = arm_loss + 0.6 * hand_loss

    optimizer.zero_grad(set_to_none=True)
    loss.backward()
    optimizer.step()

    return {
        "loss": float(loss.item()),
        "arm_loss": float(arm_loss.item()),
        "hand_loss": float(hand_loss.item()),
    }</code></pre>
</div>
</div>

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example curated episode manifest for instruction-conditioned policy training</button>
<div class="code-toggle-panel">
<pre><code class="language-yaml">episode_id: 000231
task: place_sock_in_box
platform: igris_b
instruction: "pick up the sock and place it into the box"
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
    - glove_finger_joint
    - robot_finger_joint
labels:
  outcome: clean_success
  reviewer: public_portfolio_example
  protocol_version: v1
notes:
  - no external intervention
  - grasp posture stable
  - release timing acceptable
  - accepted for policy training</code></pre>
</div>
</div>

## 7. Reinforcement-Learning-Oriented Experiment Support

### 7.1 Why RL Was Considered

My involvement on the reinforcement-learning side was more indirect and should be described carefully. I was aware of and supported a **hybrid imitation-learning and reinforcement-learning direction**, but I do not claim ownership of the exact RL algorithm, reward implementation, or final training loop.

Even so, the motivation for RL was easy to understand from the task structure itself. A teleoperated demo can provide a strong initial behavior prior, but real manipulation still contains local adaptation problems such as:

- recovering from slightly off-nominal grasps,
- adjusting placement under object variation,
- stabilizing release timing,
- and improving behavior after imitation alone has plateaued.

This is why imitation learning and reinforcement learning were better viewed as **complementary stages** rather than competing choices.

### 7.2 A Public-Safe Hybrid Interpretation

At a conceptual level, the workflow can be written as:

1. collect demonstrations,
2. train an imitation-learning prior,
3. evaluate on robot or in simulation,
4. use reinforcement-oriented fine-tuning where imitation alone is insufficient.

One simple way to express the logic is:

$$
\pi_{\text{init}} \leftarrow \pi_{\text{IL}}, \qquad
\pi_{\text{final}} \leftarrow \text{RL-finetune}(\pi_{\text{init}})
$$

where the goal of RL is not to replace the demonstration prior, but to refine it under task success, stability, and safety constraints.

### 7.3 Support Boundaries

At the public-summary level, my RL-side support is best described in three ways:

1. I helped prepare and curate the demonstrations that could serve as a behavior prior.
2. I helped structure quality labels that can support iterative experiment analysis.
3. I supported a workflow in which imitation learning and reinforcement-oriented refinement were considered complementary rather than isolated.

Exact details for reward design, simulator structure, or RL fine-tuning policy are **[confirm later]** and are intentionally generalized here.

### 7.4 Public-Safe Illustrative Source Snippets

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example reward shaping skeleton for box-packing or box-transfer refinement</button>
<div class="code-toggle-panel">
<pre><code class="language-python">def compute_reward(obs, action, info):
    reward = 0.0

    if info["grasp_established"]:
        reward += 0.4
    if info["object_in_goal_region"]:
        reward += 0.6
    if info["stable_release"]:
        reward += 0.5

    reward -= 0.01 * info["joint_velocity_norm"]
    reward -= 0.02 * info["finger_slip_score"]
    reward -= 0.20 if info["unsafe_contact"] else 0.0
    reward -= 1.00 if info["episode_failed"] else 0.0

    return reward</code></pre>
</div>
</div>

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example hybrid IL-to-RL fine-tuning loop</button>
<div class="code-toggle-panel">
<pre><code class="language-python">policy.load_state_dict(il_checkpoint)
replay_buffer.seed_with_demonstrations(curated_demo_dataset)

for step in range(max_steps):
    obs = env.get_observation()
    action = policy.act(obs, explore=True)
    next_obs, reward, done, info = env.step(action)

    replay_buffer.add(obs, action, reward, next_obs, done)
    rl_trainer.update(policy, replay_buffer)

    if done:
        env.reset()

    if step % eval_interval == 0:
        metrics = evaluator.run(policy)
        print(
            f"step={step} "
            f"success={metrics['success_rate']:.3f} "
            f"grasp={metrics['grasp_stability']:.3f}"
        )</code></pre>
</div>
</div>

## 8. Runtime Policy Execution and Robot Control

### 8.1 Why Runtime Execution Deserves Its Own Section

For a portfolio piece like this, it is easy to over-focus on "training" and under-explain what happens when a learned policy is actually attached to a real robot. In practice, deployment-minded robot learning depends on a control loop that:

- reads synchronized camera and state observations,
- assembles the policy input tensor,
- performs inference at a stable rate,
- publishes arm and hand commands,
- and checks simple safety conditions before executing the next action.

This runtime layer is where data assumptions meet hardware reality.

### 8.2 Public-Safe Runtime Example

<div class="code-toggle">
<button class="code-toggle-button" type="button" aria-expanded="false">Toggle code: Example ROS2 runtime node for policy inference with arm and finger commands</button>
<div class="code-toggle-panel">
<pre><code class="language-python">import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PolicyExecutor(Node):
    def __init__(self, policy):
        super().__init__("policy_executor")
        self.policy = policy
        self.latest_obs = None
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            "/igris/policy_action",
            10,
        )
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def update_observation(self, obs):
        self.latest_obs = obs

    def tick(self):
        if self.latest_obs is None:
            return

        with torch.no_grad():
            action = self.policy(self.latest_obs)

        arm_target = action["arm_target"]
        finger_target = action["finger_target"]

        if self._unsafe(arm_target, finger_target):
            self.get_logger().warn("Unsafe action detected; skipping publish")
            return

        msg = Float32MultiArray()
        msg.data = list(arm_target) + list(finger_target)
        self.cmd_pub.publish(msg)

    def _unsafe(self, arm_target, finger_target):
        max_arm = max(abs(v) for v in arm_target)
        max_finger = max(abs(v) for v in finger_target)
        return max_arm &gt; 1.0 or max_finger &gt; 1.0


def main():
    rclpy.init()
    # Helper intentionally omitted in this public-safe example.
    policy = load_policy_checkpoint("public_safe_example.ckpt")
    policy.eval()
    node = PolicyExecutor(policy=policy)
    rclpy.spin(node)
    rclpy.shutdown()</code></pre>
</div>
</div>

### 8.3 Practical Runtime Checks

Before a learned policy is trustworthy on hardware, several simple checks matter:

- command-range clipping,
- joint-limit verification,
- basic contact or collision guards,
- stale-sensor detection,
- and rollback to a safe teleoperation or hold mode.

In other words, a learned manipulation stack is not only a model. It is a model wrapped in instrumentation, supervision, and fallback behavior.

## 9. Evaluation and Validation Strategy

Formal benchmark reporting was still evolving, so the most meaningful evaluation criteria at this stage were operational rather than publication-style leaderboard metrics.

### 9.1 Dataset-Level Validation

The first validation layer asked:

- Was the task completed?
- Was the behavior consistent enough to be learnable?
- Was the episode clean, dirty, or fail?
- Did the arm-hand timing remain usable for training?
- Could the collection protocol be reused across sessions?

### 9.2 Experiment-Level Validation

At a practical level, the downstream learning workflow cared about:

- repeatability of demonstration quality,
- reduction of noisy samples,
- task-wise coverage across object categories,
- and whether the data pipeline supported stable experimentation.

Exact quantitative KPIs, success-rate deltas, or ablation outcomes remain **[confirm later]**.

### 9.3 Runtime-Level Validation

From a systems perspective, a policy also needed to be judged on whether it:

- executed without obvious instability,
- preserved grasp and release timing,
- generalized across object instances,
- and remained compatible with real robot control constraints.

Even without finalized benchmark tables, the evaluation stack was already useful because it linked operator behavior, collection quality, and learning readiness into a single workflow rather than treating them as separate problems.

## 10. Technical Constraints and Safety Considerations

Several real-world constraints shaped the work:

| Constraint | Why It Matters |
| --- | --- |
| Upper-body-only scope | PoCs focused on manipulation before full-body autonomy |
| Demonstration variance | Operator style changes can distort policy learning |
| Hand-state instability | Incomplete opening, finger lag, or irregular grasp posture reduces sample value |
| Multi-stream synchronization | Cameras, arms, and finger joints must align in time to be learnable |
| Real-robot mismatch | Intended motion and executed motion do not always align perfectly |
| Task diversity | Soft objects and variable boxes create broad behavior variation |

These constraints reinforced a central engineering lesson: **data collection protocol is part of the robot system design**.

## 11. Contributions and Engineering Impact

Even without claiming final-model ownership, this work contributed to a more disciplined learning pipeline by:

- establishing repeatable teleoperation-based collection procedures,
- supporting a multi-modal recording workflow that included arm motion, hand articulation, and multi-view vision,
- introducing a useful three-level QA taxonomy,
- documenting failure modes before they polluted downstream training,
- filtering out low-value episodes,
- and aligning dataset practice with realistic warehouse-manipulation objectives.

I see this as foundational systems work: building the operational layer that makes later policy learning more reliable.

## 12. Lessons Learned

Three lessons became especially clear during the project.

### 12.1 Success Is Not the Same as Learnability

A demonstration that barely succeeds may still be a poor training sample if it includes unstable recovery, interference, or inconsistent motion.

### 12.2 Hand Articulation Quality Matters More Than It First Appears

For grasp-sensitive tasks, arm trajectory alone is not enough. Finger posture, hand opening quality, and release timing all influence whether a run becomes a useful training sample.

### 12.3 Real Robot Learning Depends on Procedure Discipline

Operator discipline, task setup, and sample review are as important as model choice when moving from internal PoCs toward deployment-oriented robot learning.

## 13. Future Directions

The next technical steps I see for this work include:

- expanding beyond upper-body-only PoCs,
- incorporating richer actuator-side signals such as motor current for force-sensitive tasks,
- tightening the link between dataset QA and policy evaluation,
- improving transfer from internal research setups to deployment-like logistics environments,
- and formalizing the evaluation stack for hybrid imitation-learning and reinforcement-learning workflows.

As this project matures, its long-term value will likely come not only from one successful demo, but from a scalable process for collecting, curating, and validating robot learning data in real warehouse tasks.

Public reference implementations that align with this overall workflow include:

- ROS 2 data recording tutorial: [Recording and playing back data](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- `rosbag2` official repository: [ros2/rosbag2](https://github.com/ros2/rosbag2)
- Hugging Face LeRobot: [huggingface/lerobot](https://github.com/huggingface/lerobot)
- Robomimic demonstration-learning framework: [ARISE-Initiative/robomimic](https://github.com/ARISE-Initiative/robomimic)
- Isaac Lab reference architecture: [Isaac Lab Documentation](https://docs.robotsfan.com/isaaclab_official/main/source/refs/reference_architecture/index.html)
