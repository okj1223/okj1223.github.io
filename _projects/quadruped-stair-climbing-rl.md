---
layout: project
date: 2026-10-15
featured: false
title: "Quadruped Stair Locomotion: From Restored Models to Learned Control"
card_title: "Quadruped Stair Climbing with Reinforcement Learning"
description: "A simulation research project combining PPO, terrain-aware foot trajectories, actuator modeling, and contact-level diagnostics to develop a quadruped stair-climbing controller."
card_description: "Reconstructed a locomotion stack, developed 20 cm reference motions, trained PPO and imitation policies, and connected terrain perception to actuator-aware stair control."
share-description: "Quadruped stair-climbing research: PPO, hardware-aware simulation, foothold planning, and measured controller evaluation."
subtitle: "Model restoration, human-height reference motion, imitation learning, PPO, and the mechanics of supported stair ascent."
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
  - value: "20 cm"
    label: "Four-Step Reference"
  - value: "7 cm"
    label: "Learned Four-Step Ascent"
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
quick_summary_note: "Simulation research, September–October 2026: distinct reference, learned-policy, and restoration results, with their execution conditions."
quick_summary:
  - label: "Role"
    value: "Model and controller restoration, perception integration, reference control, PPO and imitation learning, and physical failure analysis"
  - label: "Platform"
    value: "ASR/Titan locomotion research in Isaac Sim / PhysX; versioned 12-joint models, fitted motors, and delayed actuation"
  - label: "Demonstrated"
    value: "20 cm four-step references, learned 20 cm two-step trials, and later learned 7 cm four-step ascent"
  - label: "Status"
    value: "Simulation research; each result identifies its controller, sensing assumptions, finish criterion, and validation scope"
---

## The Project: Reconstruct, Teach, Learn, and Explain

This project grew from restoring a quadruped locomotion stack into a sustained investigation of **how a learned controller can move a heavy, delayed-actuation robot up stairs**. The work spans model reconstruction, command tracking, depth-based terrain perception, model-based demonstrations, imitation learning, PPO, and contact-level diagnosis. Stair height alone does not describe the result: the controller must also respect its command interface, transfer weight between feet, finish the climb, and run within its timing budget.

My work connected these pieces into one experimental workflow. I reconstructed observation and action contracts, built and trained learning environments, developed reference motions and recovery tasks, integrated terrain-conditioned execution, and traced failures through the actual simulated motion. The ASRobotics robot models, supplied policies, and published learning methods were starting points; the reconstruction, integration, experiments, and analysis described here are the project contributions.

The research produced several different kinds of achievement:

| Research milestone | Recorded result | What the result establishes |
| --- | --- | --- |
| Restored supplied rough-terrain policy | Reached the top of a 7 cm-riser inverted-pyramid terrain in 3/3 tested seeds after restoring the matching controller semantics | A working supplied-policy replay and an identified controller-version mismatch |
| Fresh flat-ground PPO | 4.26 m forward travel in 10 s; post-start mean speed 0.440 m/s for a 0.4 m/s command | A learned locomotion baseline for the ASR model in one fixed evaluation |
| Camera-depth stair policy | 5/5 completions on three 3 cm steps | A simulated depth-to-height-map-to-policy pipeline; the same round did not solve 6 cm stairs |
| Model-based 20 cm staircase | Four 20 cm steps and a standing finish; later neutral-posture refinement reached 2.98° joint RMS from neutral | Full-height reference motion in simulation, with substantial replay and offline planning |
| Learned 20 cm two-step controller | 7/10 held-out trials in its fixed-IK controller family | A learned two-step result, with rough contact and command saturation still present |
| Later terrain-conditioned PPO/controller | Four 7 cm steps in the live-physics recording below; 14/20 moving ascents in a separate mild-randomization development comparison | A working lower-stair learned control loop and a repeatable way to evaluate changes |

These rows use different model revisions, terrain layouts, sensors, and finish criteria. They describe the research progression, not a single benchmark ranking. All results on this page are simulation results.

## A Continuous Learned Stair Ascent

<figure class="project-evidence-figure" markdown="0">
  <video class="project-video" controls playsinline muted preload="metadata" poster="{{ '/project/quadruped-stair-climbing-rl/stair-climbing-cover.webp' | relative_url }}" width="960" height="540" aria-label="Live Isaac simulation of a learned controller ascending four 7 centimetre steps">
    <source src="{{ '/project/quadruped-stair-climbing-rl/stair-ascent-7cm.mp4' | relative_url }}" type="video/mp4">
    <a href="{{ '/project/quadruped-stair-climbing-rl/stair-ascent-7cm.mp4' | relative_url }}">Watch the stair-climbing simulation.</a>
  </video>
  <figcaption>Four 7 cm risers with 30 cm treads, from a nominal floor start. This recording uses live Isaac physics at 1× speed: 13.44 s of video and ascent completion at 13.46 s in the control log. Orange = front right, cyan = front left, purple = rear right, yellow = rear left. The colors and grids change rendering only. The demonstration shows moving ascent; stable stopping is evaluated separately.</figcaption>
</figure>

## 1. Recovering the Model and Controller Contract

Before training a new policy, I had to establish what the restored material actually expected. The supplied actor used **93 inputs and 13 outputs**, combining 61 basic features with a 32-feature latent representation. The thesis reconstruction used **315 actor inputs and 13 outputs**, including joint-position and joint-velocity history sampled at 10 ms intervals. Similar filenames and a shared robot did not make these interchangeable interfaces.

I reconstructed feature order, units, history timing, action interpretation, coordinate transforms, and delay behavior. The supplied weights were kept separate from fresh thesis-configuration training: rebuilding the 315-input pipeline did not recover the original historical weights.

A particularly useful diagnosis came from pairing a supplied rough-terrain policy with different controller revisions. The later action manager interpreted motion through a world-yaw convention that did not match the supplied policy. Restoring the older body-frame manager recovered stair-top traversal in all three tested seeds on the 7 cm inverted-pyramid terrain. Replacing the saved motor model alone did not fix the failing pairing. That experiment isolated an interface mismatch that could otherwise have been mistaken for a weak learned policy.

The reconstruction also led to a **12-run development campaign**: four configuration variants across three training seeds, with paired standing and forward-command evaluations. Eight completed runs were preserved and four were newly trained from fresh initialization. Completion records distinguish those fresh jobs from an exact resume. The campaign made history timing and action-penalty choices testable; its completion is not evidence that the entire historical thesis benchmark was reproduced.

## 2. Building Locomotion and Perception Baselines

### Flat walking before stairs

I first trained a fresh PPO walking baseline on the approximately 60.4 kg ASR model. A 512-environment, 24-step, 300-update run collected **3,686,400 control transitions**. In the retained 10 s evaluation it traveled **4.260 m**, tracked a 0.4 m/s forward command at **0.440 m/s** after the initial 2 s, and recorded no fall, body-contact termination, or joint-limit event. Diagonal foot-contact correlations of approximately 0.92 supported the observed trot pattern.

This first baseline did not yet use the later fitted actuator-delay model. A separate fitted-motor baseline subsequently traveled **4.221 m in 10 s** with nominal 40 ms hip/thigh and 50 ms calf delay. Keeping the two records separate made the cost of changing the execution contract visible.

Command behavior needed its own work. Low-speed tracking, acceleration, and stopping are different operating conditions from steady forward motion. The command-control experiments combined learned locomotion with explicit foot-load features and a classical stop-feedback layer. The combined configuration completed **30/30 known development conditions**; that result belongs to the combined controller and its idealized load inputs. It is not a pure-policy or physical-sensor result.

### From camera rays to terrain features

An early perceptive branch replaced direct terrain-height observations with a simulated depth camera: **64 × 48 depth rays at 10 Hz**, transformed with robot attitude into **247 local height cells** for the 50 Hz policy. The actor combined these cells with 45 proprioceptive features and four idealized foot-load channels.

After **2,457,600 additional transitions**, this branch completed three 3 cm steps in **5/5 evaluations**. It completed **0/5** on the 6 cm extension. Masking terrain information in the same checkpoint removed all five full-completion successes at 3 cm, although four trials still visited the top region. This distinguished finishing the task from simply reaching its upper area, without treating the ablation as proof that a separately trained blind policy could never succeed.

The camera branch matters because it tested an actual perception-to-control interface in simulation. The later terrain-conditioned controller uses an ideal map and localization instead; the earlier camera experiment does not make that later configuration sensor-deployable.

## 3. Constructing a Full 20 cm Reference Motion

Learning on human-scale stairs was difficult to interpret without a reference showing how this robot could place its feet and shift its body. I developed a model-based sequence around foot placement, inverse kinematics, body motion, load transfer, and a terminal standing posture. The staircase had **four 20 cm risers and 28 cm treads**.

The R34 reference completed the staircase and held a stable finish. Its measured motion lasted **271.84 s**, comprising **178.28 s of physical command replay** and **93.56 s of online continuation**. It did not teleport the robot between steps. Offline planning paused the simulated physics, however, and the measured median control computation exceeded the 20 ms target. This was a useful physical reference under its execution conditions, rather than a real-time autonomous learned controller.

<div class="project-evidence-grid" markdown="0">
  <figure class="project-evidence-figure">
    <a href="{{ '/project/quadruped-stair-climbing-rl/reference-20cm-ascent.webp' | relative_url }}"><img src="{{ '/project/quadruped-stair-climbing-rl/reference-20cm-ascent.webp' | relative_url }}" alt="ASR quadruped partway through its model-based four-step 20 cm stair reference" width="1280" height="800" loading="lazy" decoding="async"></a>
    <figcaption>The R34 full-height reference during ascent. This image is a graphics-only rendering of retained Isaac state measurements, not a new physics rollout.</figcaption>
  </figure>
  <figure class="project-evidence-figure">
    <a href="{{ '/project/quadruped-stair-climbing-rl/reference-20cm-neutral-finish.webp' | relative_url }}"><img src="{{ '/project/quadruped-stair-climbing-rl/reference-20cm-neutral-finish.webp' | relative_url }}" alt="Later reference refinement standing on the top platform with feet moved toward a neutral configuration" width="1280" height="800" loading="lazy" decoding="async"></a>
    <figcaption>The later R73 finish refinement. Its displayed clock belongs to the rendered continuation segment; the full floor-to-finish physical sequence lasted 300.34 s.</figcaption>
  </figure>
</div>

The first complete reference stopped with a wide, non-neutral stance. I then refined the finish by advancing the front feet and lowering the body. The later R73 result reduced joint RMS distance from the neutral posture from about **22.5° to 2.98°**. Its final foot spacing was **0.761 m**, close to the **0.776 m** neutral reference, and the last 2 s satisfied both the standing and neutral-posture checks on all 100 control samples. That was a distinct task improvement: completing the last riser and returning to a useful standing configuration.

### Command feasibility was a separate discovery

The reference also exposed a critical distinction between **actual joint angle** and **the target sent to a position servo**. An inverse-PD calculation may request a virtual target beyond the joint's physical travel to generate torque while the measured joint remains inside its limits. In one audit, **9,446 of 13,592 reference targets** lay outside the then-current ROS command envelope.

I therefore made the stair command profile explicit instead of silently treating the reference as deployable through the existing interface. The subsequent R43 collection retained physical joint limits, fitted motor behavior, and actuator delay, while using a finite stairs-specific command envelope. It completed the four-step task in **250.08 s** and produced **12,504 observation–command pairs**, with zero command-clipped ticks and zero measured joint-bound excess. Intermediate body angles and non-foot contact remained rough. This was a checked simulation command contract, not hardware approval.

## 4. Turning Demonstrations into Closed-Loop Policies

The full reference created training data, but imitation loss was a poor substitute for closed-loop evaluation. A 143-input, two-layer behavior-cloning network trained for 3,000 updates reached approximately **0.84° command RMSE**, and its saved/restored output matched exactly. When it controlled the robot alone, it failed after **1.28 s** in the recorded test.

That result drove several distinct experiments:

| Experiment | What I changed | What the rollout revealed |
| --- | --- | --- |
| Demonstration-assisted PPO | Compared demonstration-assisted and unassisted PPO, each with 819,200 transitions | Both completed 0/8 full-task evaluations; low imitation influence and missing trajectory coverage mattered |
| Stronger cloning and recurrent imitation | Trained feed-forward, delta-action, and GRU variants, using teacher-assisted acquisition data | The teacher could recover the collection trajectory, while the resulting students still completed 0/6 alone |
| Corrective collection | Let the policy act briefly, then used the reference controller to recover and collect corrective examples | Short intervention schedules produced usable trajectories; longer policy bursts exposed a rapidly shrinking recovery region |
| Short recovery PPO | Restored selected physical and actuator-history states and optimized a 2.5 s local recovery task | An eight-episode smoke evaluation completed 8/8 after a small 1,024-transition run; this verified the recovery workflow, not superiority over a baseline |

The corrective collection was DAgger-style in the sense that it gathered examples around states the policy actually visited. The teacher still depended on the reference and intervention schedule; it was not a general expert capable of recovering from every state.

Reliable local recovery required restoring more than a pose. I restored base and joint velocities, command history, and the fitted 40/50 ms delay buffers. The replay check achieved roughly **0.23° joint RMSE and 0.8 mm body-position RMSE** against the selected original segment, with identical duplicate replays. Contact-solver internals were not claimed to be fully restored.

I also tested whether learning could improve reference execution speed. A 122,880-transition PPO experiment initially looked favorable against one fixed-speed controller, but a subsequent matched comparison favored a simple fixed 1.2× schedule. A later segment-wise 1.6× schedule completed a retained nominal trial in **205.28 s** and passed the four tested perturbations. The useful engineering outcome was to keep the simpler schedule when the learned alternative did not retain its advantage.

## 5. Learning Higher Steps and Defining the Actual Task

The project did obtain a learned result on **two 20 cm steps** in a fixed-IK diagonal-swing controller family: **7/10 held-out trials**. Those trials used 30 cm treads and a different control formulation from the full-height reference above. High foot impacts and frequent command-boundary use remained, so the result is reported as a two-step task achievement rather than smooth general stair locomotion.

Extending this family to four steps was much harder. Full-scan and local-view training arms each collected **3,686,400 transitions**, and neither produced a successful selected development evaluation across 14 checkpoint tests per arm. Reducing distant terrain information did not resolve the continuation failure.

I also compared a phase-guided direct-joint PPO formulation with a no-phase counterpart on an 8 cm single step. Each arm received the same 3,686,400-transition budget. The selected phase-guided policy achieved **3/5 held-out landing-proxy successes**, but **0/5** under the strict contact-quality criterion. This was an ASR implementation comparison inspired by phase-guided learning, not an exact reproduction of another robot's published benchmark.

The broader PPO work included a **24,576,000-transition curriculum run**, not only short adaptation tests. It stalled at the low-step stages and did not acquire the intended higher-stair behavior. Other task formulations exposed a simpler problem: forward progress could reward walking around the staircase. Constraining the corridor removed that shortcut but did not by itself teach ascent. I used these results to separate reward exploitation, insufficient curriculum exposure, and physical execution failure.

## 6. Repairing the Simulation and Motion Interfaces

### Collision geometry changes the learning problem

A model revision introduced a body and thigh collision representation based on single convex hulls. The hulls filled empty space in the visual geometry, creating apparent body–thigh interference even when the source triangle meshes did not intersect. Kinematic checks were accurate to within approximately one micrometre in the diagnosed comparison, so this was not explained by a gross forward-kinematics error.

I traced the issue to collision assets and prepared a separate thigh-decomposition layer while preserving the original model. This diagnosis established a specific false-interference mechanism; it did not explain every earlier fall. Model revisions also changed the declared mass—from approximately 60.4 kg in the earlier baseline to 58.7 kg in a later asset—so comparisons needed explicit model identity.

### A foot command has geometry, dynamics, and timing

The actuator work connected requested torque, PD targets, delayed commands, and measured motion. In one handoff experiment, an inconsistent velocity feed-forward term turned a 20 ms target jump into a large torque request. Reconciling the inverse-PD target and command semantics made the intended motion testable again.

The landing work then moved from a foot-centre point to the **whole sole**. A point could clear an edge while part of the sole crossed the riser. The body also translated and pitched between target construction and arrival, changing the world-space landing geometry. I separated target geometry, intervening body motion, and joint tracking when explaining the resulting foot height. Whole-sole feasibility improved the question being asked, but more expensive constraints could exceed the 20 ms control budget or push the solution against joint bounds.

These investigations made failures actionable: a policy problem, a coordinate-frame error, a collision artifact, a motor-interface error, and a missed support transition require different fixes.

## 7. Terrain-Conditioned PPO and Supported Motion

The later controller combines terrain-map features, joint state and history, IMU-related state, velocity commands, and controller state. Its **1,651-feature actor** produces **28 learned outputs** that adjust gait timing, foot placement, and body feedback. A trajectory and constrained inverse-kinematics layer converts these decisions into 12 joint targets.

<figure class="project-evidence-figure" markdown="0">
  <a href="{{ '/project/quadruped-stair-climbing-rl/control-pipeline.svg' | relative_url }}" aria-label="Open the control-loop diagram at full size"><img src="{{ '/project/quadruped-stair-climbing-rl/control-pipeline.svg' | relative_url }}" alt="Terrain and robot observations feed a PPO actor; gait and foot-placement outputs pass through constrained inverse kinematics, delayed joint control, and Isaac physics. Measured state closes the loop, with privileged training information separate from actor inputs." width="1440" height="1000" loading="lazy" decoding="async"></a>
  <figcaption>The later control family. Simulator contact truth is available to training rewards and diagnostics; the actor and runtime support logic use their declared public inputs. Map and localization inputs are idealized.</figcaption>
</figure>

The simulator runs at **200 Hz physics and 50 Hz control**, with nominal **40 ms hip/thigh and 50 ms calf actuation delay**. One retained continuation used 256 environments × 32 rollout steps × 32 updates, adding **262,144 control transitions**. Checkpoint and optimizer restoration, actual update counts, physical-step counts, and continuous floor-start rollouts were recorded separately from training completion.

The foot trajectories and IK provide geometric and position-control assistance. This is not a complete whole-body dynamics controller. A valid kinematic solution still has to produce load-bearing contact in the simulated dynamics.

### A measured landing repair

With one learned policy held fixed, I changed two execution details: preserving the lowering reference when a planned-stance foot was still airborne, and making the public support estimate more conservative using rotor acceleration inferred from joint-velocity history. Motor effort that accelerates a leg should not automatically be interpreted as ground support.

Both configurations completed the nominal 7 cm staircase. On the common **0.5–12.0 s** interval:

| Recorded quantity | Before | After |
| --- | ---: | ---: |
| Body roll RMS | 5.45° | 4.66° |
| Body roll-rate RMS | 0.349 rad/s | 0.320 rad/s |
| Foot-impact 90th percentile | 839.9 N | 793.7 N |
| Joint-torque RMS | 32.59 N·m | 32.79 N·m |
| Full-rollout ascent completion time | 12.02 s | 13.46 s |

<figure class="project-evidence-figure" markdown="0">
  <a href="{{ '/project/quadruped-stair-climbing-rl/landing-comparison.svg' | relative_url }}" aria-label="Open the measured landing comparison at full size"><img src="{{ '/project/quadruped-stair-climbing-rl/landing-comparison.svg' | relative_url }}" alt="Paired charts show roll RMS decreasing from 5.45 to 4.66 degrees, foot-impact P90 decreasing from 839.9 to 793.7 newtons, and torque RMS increasing slightly from 32.59 to 32.79 newton metres." width="1440" height="600" loading="lazy" decoding="async"></a>
  <figcaption>One nominal condition, the same policy, and two bundled controller changes. Lower roll and impact coexist with slightly higher torque and slower completion; this is not a new PPO benchmark gain.</figcaption>
</figure>

Front-right loading delay decreased from **0.272 to 0.208 s** across 12 landing events per rollout, using two consecutive 20 ms samples above 30 N as the loading criterion. Some upper-stair regions still became rougher. The comparison retained those spatial regressions instead of reducing the result to one favorable average.

## 8. Evaluation That Keeps Improvements Honest

### Coordinate-frame corrections can regress the policy

A landing goal stored in the body frame can move in the world as the body moves. One diagnosed failure accumulated approximately **5.3 cm forward and 6.7 cm upward world-goal drift**. Refreshing the vertical target against the current terrain map fixed three failed conditions but broke ten previous successes.

The paired comparison reused the same policy and 20 mild-randomization draws, varying mass, inertia, centre of mass, friction, gains, delay, and selected sensor biases:

| Controller variant | Moving-ascent completions | Change from baseline |
| --- | ---: | --- |
| Retained baseline | 14/20 | Reference |
| Pre-lift body preparation | 11/20 | 0 gained; 3 lost |
| Current-map vertical landing refresh | 7/20 | 3 gained; 10 lost |

The variants were separate controller changes, not sequential learning improvements. Neither replaced the baseline. These were familiar development conditions; held-out robustness remains a separate test.

### Higher stairs require a feasible path, not just an endpoint

The later family's 12 cm extension exposed swing paths that exceeded the joint workspace even when their endpoints were accepted. Whole-path checks removed a sequence of rejected targets, and timing changes produced a brief loaded first-tread contact. The full ascent still failed.

<figure class="project-evidence-figure" markdown="0">
  <video class="project-video" controls playsinline muted preload="metadata" poster="{{ '/project/quadruped-stair-climbing-rl/height-extension-poster.webp' | relative_url }}" width="960" height="540" aria-label="Failed 12 centimetre stair extension after a short PPO adaptation">
    <source src="{{ '/project/quadruped-stair-climbing-rl/stair-extension-12cm-failure.mp4' | relative_url }}" type="video/mp4">
    <a href="{{ '/project/quadruped-stair-climbing-rl/stair-extension-12cm-failure.mp4' | relative_url }}">Watch the higher-stair evaluation.</a>
  </video>
  <figcaption>The 12 cm extension after eight actual PPO updates and 32,768 transitions failed at about 2.30 s. The run's original update-count discrepancy and failed status were preserved, and a separate certificate checked the actual eight updates. No 17 or 20 cm result is inferred for this controller family.</figcaption>
</figure>

### Top visits, standing, and contact quality are separate outcomes

An earlier direct-joint experiment visited the top region in **20/20 development conditions**, yet achieved **0/20 stable finishes**. A diagnosed case supported much of the robot's weight on a thigh. Its finish reward was nearly inactive on visited states while the forward command encouraged continued motion.

I consequently report ascent, standing, foot-only support, impact, and actuator effort separately. The stable model-based 20 cm finish, the rough learned two-step result, and the later moving 7 cm ascent answer different questions. Joint torque also does not directly establish electrical efficiency or motor temperature.

## Outcome and Continuing Research

The project delivered a reconstructed and testable locomotion stack, fresh walking and depth-perception baselines, complete human-height reference motions, imitation and recovery datasets, learned two-step and lower-stair results, and a control-diagnostics workflow that connects policy decisions to physical support. The most useful progress often came from making a hidden assumption explicit: action-frame semantics, virtual PD targets, collision geometry, delayed commands, sole clearance, or the difference between scheduled stance and actual loading.

The next technical target is to combine these strengths in one controller: coordinated body–foot transitions, reliable neutral stopping, higher-stair continuation, and robustness under a consistent actuator and sensing contract. Hardware deployment and the later controller's real perception pipeline remain unverified. They require their own latency, sensing, and robot trials rather than extrapolation from these simulations.

## Evidence and Reproducibility

This case study uses retained September–October 2026 records, including the earlier restoration archive and the final October 15 comparison. Preparing the page started no new research run and changed no research source, checkpoint, or original result. The [results and media provenance summary]({{ '/project/quadruped-stair-climbing-rl/results-summary.json' | relative_url }}) records experiment identifiers, measurements, evaluation scope, and media hashes. Full traces, source, and weights remain in the research archive.
