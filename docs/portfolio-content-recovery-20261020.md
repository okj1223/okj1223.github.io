# Project content recovery audit — 2026-10-20

## Change history and method

Commit `beda9b7996fe5952c3a96f7606c32314b5a11f18`, **2026-07-10**,
“Unify portfolio experience and project presentation”, replaced the ten existing
long-form project documents with much shorter case studies. It also moved shared
presentation code into layouts/assets and converted or removed media. Raw line-count
reductions are therefore **not** a measurement of technical content loss. This audit
compares substantive subjects, implementation explanations, fabrication records,
and actual media references against that commit's parent.

The Robros case subsequently regained substantial detail in `6ef9280` and
`d9c0001` on 2026-08-28. Its public-safe protocol, quality-gate, and whole-body
experiment account is preserved. The stair case was newly added in `b63fb39` on
2026-10-20, then corrected for SVG whitespace in `5ee534a`; its first version
covered too little of the research history and needs expansion rather than rollback.

Recovery begins from `5ee534ab69ca944e65ea1936d8764d5ffd7ae5b8`. The local backup
branch `backup/portfolio-before-recovery-20261020` preserves that state. No reset
or history rewrite is used. Research originals and confidential material are unchanged.

## Coverage ledger

These entries map the substantive older topics to the repaired cases. Long
illustrative source blocks are explained through their algorithms and interfaces;
they are not presented as a newly verified source repository. Repeated marketing
text, speculative roadmaps, unsupported metrics, and inconsistent operating
instructions are not restored as accomplished work.

| Project | Historical topics restored or retained | Current location and treatment |
| --- | --- | --- |
| Catalyst cleaning | Motion/firmware; nozzle flow and pressure loss; structure/drive calculations; grinding, machining, tapping, CAD and assembly; workshop checks | **CAD, Machining, and Assembly**; **Motion Sequencing and Firmware Structure**; **Nozzle, Structure, and Drive Calculations**; **Workshop Verification and Iteration**. Direct team-lead role and build account retained. Command resolution is distinguished from measured positioning accuracy. |
| Automated mixing | State machine; temperature/level/pressure/flow sensors; filtering; PID saturation and cascade; Arduino/Raspberry Pi; serial/MQTT/logging; pump head, piping, electrical interfaces; staged testing | **Batch Sequence and State Transitions** through **Validation as a Sequence of Observable Checks**. A five-stage flow replaces unavailable illustrations. Cascade semantics and anti-windup are explained; numerical SIL, capability, availability and savings require evidence. |
| Crack/PPE robot | Person/PPE association; tracking; HSV/morphology; depth interpretation; navigation/coordination; MQTT events/commands; dashboard/mobile app; evaluation | **Person Detection, PPE Association, and Tracking** through **Evaluation Boundaries**. Camera range is not called crack depth. Team integration scope retained. Unsupported patents, publications, fleet-scale performance and ROI excluded. |
| Dry-ice feeder | Two-phase conveying; pellet condition; hopper/frame/TIG fabrication; geared drive/control chain; bridging, moisture and nozzle behavior; commissioning | **Pellet Handling and Pneumatic Delivery** through **What the Build Demonstrates**. AC-drive power and DC-controller reference signal distinguished. Physical field-build evidence retained; no unverified vessel certification, weld qualification or endurance result. |
| Dual gear pumps | Nozzle/pipe/head model; hydraulic power and suction/NPSH; pump/drive selection; wiring; frame/mobility; suction/discharge; commissioning and measurement locations | The restored hydraulic-sizing, pump-selection, control/wiring, packaging and commissioning sections complement the retained independent-train, relief/isolation and serviceability account. Numerical field results require test sheets. |
| Liquid injection | Load-cell fixture/printing; bridge/ADC/electronics; calibration/data flow; concentration balance; tilted-container flow; feedback; EKF/UKF/evaluation | **Designing the Measurement Fixture** through **Filtering and Evaluation**. Concentration derivative and target water mass corrected; final settled mass and in-flight liquid distinguished. No inferred numerical filter superiority. |
| PCB inspection | Conveyor CAD/printing/drive/friction; detector versus board judgment; orientation/normalization/matching; RealSense/robot transforms; inspection specification; wake-word/speaker verification/STT/intent/MQTT | **Conveyor Design and Fabrication** through **Voice Interface: Wake Word to Structured Command**. All older extant media subjects are present, including recovered printing and pre-belt assembly. Team integration scope retained. |
| TurtleBot3 | HSV/reflection handling; polynomial lane/PD; traffic/barrier states; marker transforms; arm IK/Jacobian; sensor fusion/arbitration; simulation-to-hardware iteration | **Lane Perception: From Pixels to a Steering Error** through **State Estimation and Behavior Arbitration**. Incorrect general square-Jacobian inversion replaced by a damped pseudoinverse explanation. General models separated from unpreserved tuned coefficients. |
| SCR apparatus | Gas mixture/flow control; reactor/sample/thermal zones; analyzer path; experiment sequence; NOx conversion/space velocity; uncertainty | **Integrating Gas Delivery, Reactor, and Measurement** through **Repeatability and Uncertainty**. Flow-weighted concentration and GHSV defined consistently. Apparent conversion is not intrinsic kinetics. No unverified compliance or round-robin result. |
| Robros humanoid | AI-team OpenVLA work; whole-body protocol/teleoperation; synchronized episodes; labeling/quality gates; curation; learning/runtime interfaces; target-selection and failed-grasp analysis | August expansion retained. **AI-Team Work with OpenVLA**, **From Curated Episodes to a Learning Experiment**, **Runtime Execution and Action–Observation Diagnosis**, **OpenVLA Rollouts: Target Selection and Failed-Grasp Responses**, and **VLA, Imitation Learning, and Reinforcement Learning** connect the data pipeline to the model experiments. See the follow-up source audit below. |
| Quadruped stairs | Restoration; actor contracts; flat/command/depth baselines; full-height reference/neutral finish; command limits; BC/GRU/corrective collection; recovery/speed; two-step learning; PPO/curriculum; asset/motor/sole diagnostics; later terrain control/randomization/height extension | Entire case rebuilt as **Reconstruct, Teach, Learn, and Explain**, followed by eight technical stages. Earlier 20 cm reference and learned two-step results restored with their execution conditions. Existing 7 cm live recording, measured comparison and failed extension retained. Expanded JSON records scopes, sources, seed results and hashes. |

## Media coverage

Every older `/project/` reference in the ten historical articles is classified in
`portfolio-media-coverage-20261020.json`. A reference is not evidence that a file
existed: the mixing article referenced **39 illustrations absent from the pre-rewrite
Git tree**, and SCR referenced **two absent files**. These are not counted as files
deleted by July's rewrite.

Recovered from Git history in this revision:

| Historical file | Published replacement |
| --- | --- |
| Catalyst `3dworking1.gif` | `cad-motion.mp4` |
| Catalyst `tap_machine.jpg` | `tap-machine.webp` |
| Liquid `3d_printing_process.gif` | `fixture-printing.mp4` |
| PCB `conveyor-belt-prototype-v1-frame-3d-printing.gif` | `conveyor-printing.mp4` |
| PCB `conveyor-belt-prototype-v1-frame-motors-no-belt.jpg` | `conveyor-drive-assembly.webp` |
| Crack/PPE `aplication.gif` | `operator-application.mp4` |

Animations retain timing in compact H.264; photos are resized WebP. Existing
conversions remain for catalyst component/nozzle photos and grinding, dry-ice
hardware, pump/panel photos, SCR hardware, the crack dashboard and TurtleBot ArUco
recording. Unlinked technical images are reconnected to their explanations.

The crack/PPE accident, fatality-rate and model-comparison graphics are not used
as evidence without their underlying statistics or evaluations. The old Robros
overview is superseded by its fuller public-safe pipeline and protocol figures.
These are explicit dispositions, not unnoticed omissions.

Two stair-reference images are also added: R34 ascent and R73 neutral finish.
Captions identify graphics-only rendering of retained state measurements, not
new physics or learned-policy videos. The existing live-physics videos are unchanged.

## Verification completed before publication

- All 11 project cards pass the repository validator; the existing three featured projects remain unchanged.
- Production Jekyll build and Doctor pass using the existing system Ruby environment without dependency or lockfile changes.
- All 11 rendered cases pass local-reference checks: 350 references, 73 images, nine videos, heading IDs, and search-index membership.
- All four recovered MP4s pass full-frame decoding; seven external demo/media links return their expected YouTube metadata.
- Desktop (1920 px) and mobile (390 px) checks cover all 11 cases: no global horizontal overflow, no broken loaded images, all nine video metadata checks pass, and table-of-contents links match headings.
- Project filters, demo-modal open/close, mobile process flow, and horizontally scrollable tables pass browser checks.
- Browser QA exposed an additional omission: search indexed descriptions but not the detailed project bodies. A separate `content` field now indexes all 11 full case studies while retaining short result descriptions. Live preview queries for `Jacobian`, `anti-windup`, `GRU`, `mass fraction`, and `stair` return the expected cases; mobile search also passes.

The working browser, media, and rendered-link receipts are retained separately.
Publication requires a clean staged diff, normal push, successful current Pages
workflow, and verification of the served cases, search index, and changed media.

## Follow-up: omitted ROBROS AI-team and VLA experience

The first recovery at `3c26765` still omitted the named OpenVLA work and the
AI-team role. Its ROBROS audit relied on the existing protocol account and added
general learning interfaces without checking the presenter's Downloads deck.
The restored article now covers those missing subjects explicitly.

Primary source: `02_Kyungjun_Oh_weekly_report_261020.pptx`, slides **4–6**
and their speaker notes, SHA-256
`3b601912cd9192dbb84107e5faba134cd5033e12ec70c04348fc1954ad6e25a3`.
The source deck and its original embedded media remain outside the public repository.

| Source | Content reflected in the project |
| --- | --- |
| Slide 4 and notes | Research assistant working in the AI team; OpenVLA sock pick-and-place and box lift-and-place; images and instructions conditioning actions |
| Slide 5 and notes | Original 15.0–37.6 s box excerpt; approach to the supporting container at about 33 s after target removal; unsuccessful grasp; target selection and its cause treated separately |
| Slide 6 and notes | Original 16.5–23.5 s sock miss followed by continuation; 47.5–59.6 s retry, brief movement, then loss; no completed placement in the retry excerpt |
| Slide 6 and notes | Presenter account of no added task-specific recovery demonstrations, earlier online-RL difficulty, and interests in recovery, generalization, data efficiency, and model design |
| OpenVLA project website | General VLA interface and demonstration-based adaptation, linked as model background rather than evidence of the company's implementation |

Role, model attribution, inference timing, and task-specific demonstration coverage
come from the presenter's account. The inspected clips support visible motion and
object outcomes; they do not independently establish the checkpoint, prompt,
training recipe, internal failure awareness, or causal mechanism. The text retains
that distinction and makes no successful recovery or zero-shot task-success claim.
The later lab-platform proposal is not attributed to the ROBROS employment period.

The case title, project-card copy, technology tags, role and workflow summary,
homepage career entry, and About introduction now identify the AI-team/OpenVLA
work consistently. Existing protocol, QA, labeling, whole-body control, and media
coverage are retained.

Follow-up validation: all 11 project-card records, the production Jekyll build,
and Doctor pass. All 15 existing ROBROS project-image references are retained;
the rendered case's 29 local asset references resolve. Desktop and 390 px mobile
browser checks pass without page overflow, with 27 matching heading/TOC entries
and scrollable tables. `OpenVLA` search returns the updated case. The homepage
card and AI-team career entry, and the About introduction, display the new scope.
