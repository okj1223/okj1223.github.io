---
layout: project
title: "Weaveflow"
subtitle: "A local-first AI work factory for recoverable long-running Codex work through OpenClaw, structured artifacts, checkpoints, and Korean reports."
description: "Technical report for Weaveflow, a local-first workflow kernel and OpenClaw/Codex automation experiment for starting, checking, cancelling, recovering, and reviewing long-running AI work with local artifacts and conservative safety gates."
share-description: "Weaveflow technical report: a local-first AI work factory for recoverable long-running Codex work through OpenClaw/Discord, checkpoints, policy gates, and Korean reports."
thumbnail-img: /assets/img/side-builds/weaveflow.png
share-img: /assets/img/side-builds/weaveflow.png
permalink: /side-builds/weaveflow/
body-class: side-build-report
back_url: /home.html#side-builds-title
back_label: "Back to Side Builds"
css:
  - "/assets/css/project-detail.css?v=20260415-1"
ext-css:
  - "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/github-dark.min.css"
ext-js:
  - "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/highlight.min.js"
js:
  - "/assets/js/project-detail.js?v=20260415-3"
  - "/assets/js/project-code-highlight.js"
quick_summary_note: "A technical and product summary for a side build focused on safe, recoverable, unattended AI work."
quick_summary:
  - label: "Problem"
    value: "Long-running agent work is risky when the session can stop, drift, or report progress without durable evidence."
  - label: "Core Idea"
    value: "Turn remote OpenClaw/Codex requests into local job artifacts, bounded worker segments, checkpoints, and reviewable Korean reports."
  - label: "Main Controls"
    value: "Start, check, cancel, recover, morning review, and operator actions."
  - label: "Safety"
    value: "Runtime validation, worker preflight, git checks, policy gates, quality gates, denied risky actions, and truthful blocked/start-failed outcomes."
  - label: "Stack"
    value: "Python CLI kernel, SQLite task index, file-based artifacts, Node/OpenClaw plugin POC, Codex CLI worker, and local git worktrees."
---

# Weaveflow Technical Report

## Abstract

Weaveflow is a local-first workflow kernel and personal AI work-factory experiment. I built it for a very practical reason: I wanted to start work remotely through OpenClaw and Codex, then let it keep moving while I was asleep, commuting, or at the company. That kind of work is useful only if it is honest. A long-running agent that disappears, forgets context, mutates the wrong scope, or says "I will handle it" without a real running worker is more dangerous than helpful.

The project solves that by making long-running AI work startable, checkable, cancellable, recoverable, and reviewable through local records. Core Weaveflow turns requests into task files, plans, Codex briefs, verification records, reports, and a SQLite index. The current OpenClaw/Codex branch adds a personal automation layer that can create `JOB-*` artifacts, validate runtime and repo state, start a controlled Codex worker, split long work into checkpointed segments, summarize progress in Korean, and prepare recovery prompts when a run stops.

![Weaveflow long-running AI control flow]({{ '/assets/img/side-builds/weaveflow-flow.svg' | relative_url }}){: .flowchart}

## 1. Product Motivation

The project came from an uncomfortable failure mode in AI-assisted work: the user asks for a large repair, refactor, documentation pass, or data review, then has to stay nearby because the session may stop or the agent may lose the thread. That defeats the point of delegation.

The target use case is "start this from OpenClaw or Discord, then let it advance while I am away." The system is designed for:

- overnight work while the user is sleeping,
- company-time work while the user is doing something else,
- broad but bounded repair jobs,
- data-review jobs that need multiple passes,
- and follow-up review where the user needs a short Korean summary instead of a raw terminal transcript.

The important distinction is that Weaveflow is not trying to make Codex reckless. It is trying to make Codex work **observable**. Every useful claim should be backed by a local artifact: a job id, a prompt, a policy decision, a worker preflight result, a checkpoint, a result file, a report, or a recovery capsule.

## 2. System Shape

Weaveflow has three layers.

| Layer | Role | State |
| --- | --- | --- |
| Core workflow kernel | Local task creation, planning, worker brief generation, artifact attachment, verification, final report, and memory proposal. | `.weaveflow/tasks/` and SQLite task index. |
| Adapter / stdio boundary | Structured request handling, JSON contracts, diagnostics, confirmations, replay-aware wrapper behavior, and local bridge execution. | Adapter responses, transcripts, diagnostics, and bridge events. |
| OpenClaw/Codex automation layer | Personal long-running job control from OpenClaw/Discord with start/check/cancel/recover/review operations. | `.weaveflow/jobs/JOB-*`, chain directories, checkpoints, reports, and operator actions. |

This separation matters because the core kernel remains a local workflow system even without automatic Codex execution. The OpenClaw/Codex layer is the current branch's personal automation experiment on top of that kernel, not a public SaaS or a multi-tenant orchestration platform.

## 3. Core Workflow Kernel

The core Python layer turns a natural-language request into a durable local task workspace. A task can contain:

| Artifact | Purpose |
| --- | --- |
| `task_spec.yaml` | Captures the request, scope, assumptions, and user-facing task definition. |
| `plan.yaml` | Breaks the task into concrete steps. |
| `worker_brief_codex.md` | Produces a handoff brief that Codex can use in a separate session. |
| `artifacts.yaml` | Records files, evidence, outputs, and relevant references. |
| `verification_record.yaml` | Stores checks and validation results. |
| `final_report.md` | Summarizes outcome, changes, limits, and follow-up work. |
| `memory_diff.md` | Proposes durable memory updates instead of silently applying them. |

The source of truth is intentionally boring: files under `.weaveflow/tasks/` plus a SQLite index. That gives the system a property chat-only workflows do not have. If the agent stops, the task record still exists. If a report looks suspicious, the user can inspect the underlying artifacts.

## 4. OpenClaw And Stdio Boundary

The first integration step was a narrow OpenClaw plugin proof of concept. It validated that an OpenClaw-side JavaScript plugin could spawn the Weaveflow Python stdio bridge, send line-delimited JSON requests, receive structured JSON responses, and shut down cleanly.

The current integration lives under `integrations/openclaw-weaveflow-stdio-poc/` and exposes a broader tool surface:

| Tool | Function |
| --- | --- |
| `weaveflow_stdio_poc` | Smoke-test the Python stdio bridge through OpenClaw. |
| `weaveflow_runtime_doctor` | Diagnose runtime root, Python executable, module import, and bridge command setup. |
| `weaveflow_start_codex_job` | Start a controlled long-running Codex job when runtime and worker preflight pass. |
| `weaveflow_check_codex_job` | Report job or chain status with conservative liveness interpretation. |
| `weaveflow_cancel_codex_job` | Request cancellation for a job or chain and record cancellation artifacts. |
| `weaveflow_recover_codex_job` | Inspect recovery state, prepare a next prompt, or start the next segment when allowed. |
| `weaveflow_morning_review` | Generate a report-only summary of recent jobs and chains after a long unattended period. |
| `weaveflow_operator_action` | Show or execute token-bound follow-up actions such as inspect, check, prepare recover, cancel, pause, or continue. |

The key rule is **no fake delegation**. If the system says a job started, it must have a job id, a worker start record, and a running process. If setup fails, the response should say what failed: missing repo, ambiguous target, runtime unavailable, Codex command unavailable, git preflight failed, blocked policy, worker script missing, or worker start failure.

## 5. Long-Running Job Lifecycle

A long-running job starts as a remote instruction from OpenClaw or Discord. Weaveflow then turns it into a concrete local job contract.

```text
OpenClaw request
  -> normalize request and target workspace
  -> classify job type and risk
  -> choose run profile
  -> write job_request / policy / phase_plan / initial_prompt
  -> validate Weaveflow runtime
  -> validate Codex command, target git repo, worker script, and git state
  -> spawn controlled Codex worker
  -> write worker_start and start_outcome
  -> checkpoint, report, recover, or complete
```

Every start attempt writes the artifacts it can under `.weaveflow/jobs/JOB-*`. Even a blocked start is useful because it leaves `runtime_diagnostics.json`, `worker_preflight.json`, or `start_outcome.json` explaining why no worker is running.

For large repair jobs, the phase plan is shaped around safe progress: preflight git sync, bug inventory, root cause pass, minimal fix pass, regression pass, verification pass, and Korean report. For data-review jobs, the same pattern becomes inventory, meaning-quality review, minimal fix, verification, and report.

## 6. Run Profiles

Weaveflow does not treat all jobs the same. The OpenClaw/Codex layer has run profiles for different kinds of work.

| Profile | Session Limit | Total Budget | Checkpoint Rhythm | Use Case |
| --- | ---: | ---: | ---: | --- |
| `quick` | 20 min | 20 min | 10 min | Small, low-usage work. |
| `focused` | 60 min | 90 min | 20 min | Normal development work. |
| `company` | 45 min | 240 min | 15 min | Work while the user is away for a few hours. |
| `overnight` | 45 min | 480 min | 20 min | Checkpoint-based overnight progress. |

The `company` and `overnight` profiles are not one giant uncontrolled session. They are segmented chains. One `CHAIN-*` represents the long job; each `JOB-*` is one concrete Codex worker segment. When a segment reaches its boundary, the system can write a checkpoint and resume capsule, then decide whether a next segment is safe.

## 7. Checkpoints And Resume Capsules

The checkpoint system exists because long-running work should be resumable. If Codex hits a usage limit, fails a check, reaches a session boundary, gets cancelled, or completes a phase, the runner can write:

- `checkpoints/checkpoint-0001.json`
- `checkpoints/checkpoint-0001.md`
- `resume_capsule.json`
- `resume_capsule.md`
- `next_suggested_prompt.md`

The resume capsule records the current objective, completed work, changed files, checks run, failures, repeated-failure count, fix attempts used, remaining budget, unsafe actions skipped, recommended next action, and the exact next prompt for Codex.

That is the practical answer to the "agent turned off" problem. The next worker should not have to guess from chat history. It should read the capsule, inspect the repo, and continue from a known point.

## 8. Safety Model

Weaveflow's safety model is conservative because the target environment is a real local workspace.

| Gate | What It Protects |
| --- | --- |
| Runtime validation | Confirms the Weaveflow Python package can actually be imported from the configured runtime root. |
| Worker preflight | Confirms the Codex command, target workspace, git repo state, and worker script are usable before claiming a job started. |
| Policy decision | Allows broad work with constraints, but blocks or denies risky actions. |
| Safe worktree execution | Keeps long-running edits inside a controlled git worktree boundary. |
| Usage Limit Guard | Watches for quota/rate-limit signals and checkpoints instead of blindly retrying. |
| Quality gate | Looks for missing requirements, failed checks, risky changes, unrelated changes, and scope drift. |
| Operator token | Binds follow-up actions to a job or chain to reduce accidental replay from OpenClaw/Discord. |

Default denied actions include production deploys, secret changes, destructive database migrations, uncontrolled commits, uncontrolled pushes, force pushes, and destructive cleanup. One-click continue/recover means the control step is convenient; it does not mean dangerous operations are automatically approved.

## 9. Check, Review, And Recovery

The check path is designed around truthfulness. A stale heartbeat is not reported as healthy running work. A blocked or start-failed job is not presented as delegated work. Missing artifacts are reported as missing rather than guessed.

When the user returns in the morning or after work, `weaveflow_morning_review` can scan recent jobs and chains and write an operator review report. Its job is not to mutate tasks or restart workers. It tells the user what needs attention:

- jobs ready for review,
- chains that can continue,
- items waiting for limit reset,
- blocked setup problems,
- stale or dead workers,
- unknown jobs that need manual inspection,
- and reports or next prompts worth opening.

`weaveflow_operator_action` then provides a follow-up menu. Read-only actions such as inspect, check, show next prompt, and open report can run directly. Safe mutations such as prepare recover, mark reviewed, pause chain, cancel job, or cancel chain require confirmation. Starting another worker segment requires confirmation and a token, then re-runs runtime validation, worker preflight, and continuation policy.

## 10. Current Validation State

The project has passed deterministic local validation for much of the OpenClaw/Codex integration surface: runtime resolution, worker preflight behavior, start/check/cancel/recover flows, segmented chains, checkpoint and resume capsule generation, morning review, operator action menus, and dangerous-action denial.

There has also been live validation showing Discord/OpenClaw can trigger Weaveflow Codex job-runner flows and produce Korean status/results with local job artifacts. Still, the current portfolio framing should stay honest: this is a personal automation POC, not a production orchestration product. Some contracts, especially around liveness artifacts and real-world worker supervision, are the kind of details that should keep getting tightened as the tool is used.

## 11. Why This Matters

The interesting part of Weaveflow is not that it can run a command. The interesting part is the product posture: AI work should become more autonomous without becoming less accountable.

For short tasks, chat is enough. For long work, the user needs a control plane:

- what exactly was started,
- where it ran,
- what it changed,
- what it checked,
- why it stopped,
- whether it is safe to continue,
- and what a human should review next.

Weaveflow is my attempt to make that control plane local, inspectable, and useful for real personal work.

## 12. What I Would Improve Next

The next useful improvements would be:

- tighten the heartbeat, job-status, and session-log contract around live worker liveness,
- run more real OpenClaw + real Codex end-to-end pilots instead of only deterministic harnesses,
- improve Korean summaries so they highlight only human-review decisions,
- make recovery plans and quality-gate results easier to compare across segments,
- add clearer workspace selection and repo registry UX,
- and keep commit/push policy explicit enough that unattended work never surprises the user.

## 13. Screenshots To Add Later

The report would become much stronger with real screenshots or captured artifacts. These are the images I would add when available:

| Needed Image | Why It Helps |
| --- | --- |
| OpenClaw or Discord command starting a Weaveflow job | Shows the remote-control surface that motivated the project. |
| `started_job` response with job id and artifact paths | Proves that delegation is tied to a concrete worker start. |
| Blocked preflight response | Shows the "truth over optimism" behavior when runtime, git, or Codex setup is wrong. |
| `.weaveflow/jobs/JOB-*` artifact tree | Makes the local audit trail visible. |
| `check` response for a running or stale job | Shows how the user can inspect progress while away. |
| Checkpoint and resume capsule files | Shows how stopped work can continue without starting over. |
| Morning review report | Shows the after-sleep / after-work summary loop. |
| Operator action menu with token-bound recover/continue | Shows safe follow-up control. |
| Quality gate or Korean result report | Shows what the human reviews before accepting work. |
| Chain directory with multiple segments | Shows how company/overnight work is split instead of run as one uncontrolled session. |

## 14. Related Links

- [Weaveflow repository](https://github.com/okj1223/weaveflow)
