---
layout: project
title: "Weaveflow"
subtitle: "A local-first proof of concept for recoverable, inspectable long-running Codex work."
description: "A personal workflow-kernel and OpenClaw/Codex automation proof of concept for starting, checking, cancelling, recovering, and reviewing bounded AI work through local artifacts and conservative gates."
share-description: "Weaveflow is a local-first AI workflow proof of concept built around durable artifacts, checkpoints, conservative job state, and human review."
thumbnail-img: /assets/img/side-builds/weaveflow.webp
share-img: /assets/img/side-builds/weaveflow.webp
permalink: /side-builds/weaveflow/
category_label: "AI Workflows - Proof of Concept"
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
quick_summary_note: "A personal automation proof of concept, not a production orchestration platform."
quick_summary:
  - label: "Status"
    value: "Local personal proof of concept with deterministic integration tests."
  - label: "Problem"
    value: "Long-running agent sessions are difficult to trust when progress, liveness, and recovery exist only in chat."
  - label: "Core Idea"
    value: "Represent every job with local requests, policy decisions, worker records, checkpoints, and review artifacts."
  - label: "Controls"
    value: "Start, check, cancel, recover, review, and token-bound operator actions."
  - label: "Stack"
    value: "Python CLI kernel, SQLite index, file artifacts, Node/OpenClaw plugin POC, Codex CLI worker, and git worktrees."
---

## Status and Scope

Weaveflow is a local personal proof of concept for controlling longer Codex tasks through OpenClaw-style requests. It is not a production scheduler, hosted service, or multi-tenant orchestration platform. The current implementation and deterministic test harness explore job contracts, local evidence, recovery, and conservative safety gates; this report does not claim unattended production reliability.

![Weaveflow control flow]({{ '/assets/img/side-builds/weaveflow-flow.svg' | relative_url }}){: .flowchart}

## Problem

Chat is adequate for short work, but long-running agent work needs state outside the conversation. A session can stop, lose context, operate in the wrong workspace, or report progress after its worker is no longer healthy. Without durable evidence, the user cannot answer basic questions: what started, where it ran, what changed, why it stopped, and what is safe to do next.

Weaveflow makes those questions part of the job model. A request becomes a local workspace containing a specification, plan, worker brief, policy decision, verification evidence, result report, and recovery material. A SQLite index makes tasks discoverable while files remain the inspectable source of truth.

## System Shape

The prototype has three layers:

| Layer | Responsibility |
| --- | --- |
| Workflow kernel | Creates task specifications, plans, briefs, artifact records, verification records, and final reports. |
| Structured adapter | Moves line-delimited JSON requests and responses across the OpenClaw/Node and Python boundary. |
| Codex automation layer | Starts, checks, cancels, recovers, and reviews bounded worker jobs. |

The adapter proof of concept keeps the integration narrow: a JavaScript plugin starts a Python stdio bridge and exchanges structured messages. Higher-level tools use that boundary to run diagnostics and create job records under `.weaveflow/jobs/JOB-*`.

## Job Lifecycle

A start request is normalized against a target workspace, classified by job type and risk, and assigned a bounded run profile. Weaveflow writes the request, policy, phase plan, and initial prompt before checking the runtime, Codex command, git repository, worker script, and worktree state.

Only a successful preflight can produce a started state. A failed attempt still records available diagnostics and a start outcome, making **not started** distinct from **running**. Check operations interpret worker and heartbeat evidence conservatively; missing or stale evidence is surfaced rather than guessed.

Larger runs can be split into a `CHAIN-*` with individual `JOB-*` segments. At a segment boundary, the runner can write JSON and Markdown checkpoints, a resume capsule, and a suggested next prompt. The capsule records completed work, changed files, checks, failures, remaining scope, skipped unsafe actions, and the recommended continuation. Recovery therefore begins from local evidence rather than reconstructed chat history.

## Safety and Review

The safety model is designed for a real local workspace:

| Gate | Purpose |
| --- | --- |
| Runtime and worker preflight | Prevents a start claim when required commands or repositories are unavailable. |
| Policy decision | Constrains scope and denies configured risky operations. |
| Worktree boundary | Separates job edits from an uncontrolled working directory. |
| Usage and quality gates | Checkpoints on limit signals and records failed requirements or verification. |
| Operator token | Binds mutating follow-up actions to the intended job or chain. |

Configured denied operations include production deployment, secret changes, destructive database migration, force pushing, and destructive cleanup. Read-only inspection can remain direct, while cancellation, recovery preparation, or continuation follows explicit operator controls.

The review path summarizes recent jobs and chains into a Korean operator report. It distinguishes completed work, review-ready work, blocked setup, interrupted segments, and unknown states. The report is an inspection aid; it does not silently restart workers or accept their changes.

## Validation Boundary

The local test surface covers runtime resolution, preflight outcomes, start/check/cancel/recover contracts, segmented chains, checkpoint generation, review reports, operator actions, and denial of configured dangerous actions. These checks validate the prototype's state transitions and artifacts. They do not replace long-duration operational testing, so the project remains explicitly labeled as a personal POC.

## Engineering Takeaway

Weaveflow explores a control-plane principle for agentic development: more autonomy should create more inspectable state, not less. Durable artifacts, truthful failure states, bounded segments, and human review make a long task recoverable even when the worker itself is temporary.
