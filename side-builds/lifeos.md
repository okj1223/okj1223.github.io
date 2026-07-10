---
layout: project
title: "LifeOS"
subtitle: "A private personal operating system built around fast capture, later clarification, and visible relationships."
description: "A private, single-owner Next.js application for capturing ideas, organizing work, reviewing commitments, and inspecting relationships between tasks, projects, goals, and resources."
share-description: "LifeOS is a private personal workflow system built with Next.js and Supabase around quick capture, deliberate clarification, owner-scoped data, and relationship-aware planning."
thumbnail-img: /assets/img/side-builds/lifeos.png
share-img: /assets/img/side-builds/lifeos.png
permalink: /side-builds/lifeos/
category_label: "Personal Systems - Private Application"
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
quick_summary_note: "A private personal application, presented here as a product and systems case study."
quick_summary:
  - label: "Status"
    value: "Private, single-owner personal application."
  - label: "Problem"
    value: "Planning tools often require too much classification at the moment of capture."
  - label: "Core Loop"
    value: "Capture, clarify, organize, execute, and review."
  - label: "Key Surface"
    value: "Structure exposes missing next actions and disconnected work without requiring a dense graph."
  - label: "Stack"
    value: "Next.js App Router, TypeScript, Supabase Auth/Postgres/RLS, Server Actions, Tailwind, and PWA support."
---

## Status and Scope

LifeOS is a private, single-owner application I built to reduce the maintenance cost of personal planning. It is not a public productivity service or a multi-tenant product. The portfolio version focuses on the product model, security boundary, and technical decisions without exposing personal data or a private source repository.

![LifeOS operating loop]({{ '/assets/img/side-builds/lifeos-flow.svg' | relative_url }}){: .flowchart}

## Problem

Many planning tools ask the user to classify an idea before it can be captured: choose a project, due date, priority, area, or tag, then decide whether the item is a task or reference. That is useful during planning but expensive during capture. The result is either abandoned input or a carefully organized system that takes too much effort to maintain.

LifeOS separates those moments. Raw thoughts enter an Inbox with minimal ceremony. Later, when context is available, an item can become a Task, Project, Goal, or Resource, or move to Archive. The governing loop is:

```text
Capture -> Clarify -> Organize -> Execute -> Review
```

This distinction shaped the interface and data model: loose input is valid, relationships are optional, and structure is added only when it helps execution or retrieval.

## Product Approach

The application models several types of personal work rather than treating everything as a task:

| Entity | Responsibility |
| --- | --- |
| Inbox Item | Holds raw, unprocessed input. |
| Task | Represents one executable action. |
| Project | Groups multiple actions around an outcome and next action. |
| Goal | Provides time-bounded direction for projects. |
| Area | Represents an ongoing context that is maintained rather than completed. |
| Resource | Stores information that should remain recoverable. |
| Review and Habit | Close feedback loops around commitments and recurring behavior. |

The Dashboard combines quick capture, today's tasks, unresolved Inbox items, active projects, goals, habits, resources, and review prompts. Inbox triage supports search, editing, archive and restore, and conversion into the appropriate entity. Search and Archive keep inactive material recoverable without leaving it in active views, while export paths provide copies of the owner's data in JSON, CSV, or text-oriented formats.

## Structure Without Graph Clutter

The differentiating surface is **Structure**. It helps inspect questions that flat task lists hide: which projects have no tasks, which goals have not produced projects, which projects lack a next action, and which resources or tasks are disconnected from larger work.

Structure provides overview, read-only database, focused relationship inspection, cleanup candidates, and scoped map modes. The map uses lanes such as Context, Execution, Knowledge, Routine, and Signals instead of drawing every possible edge. This keeps optional, cross-cutting relationships inspectable without turning the screen into a dense graph.

## Technical Architecture

LifeOS uses Next.js App Router, TypeScript, React Server Components, and Server Actions. Zod validators guard mutation inputs. Supabase provides Postgres and authentication, while Row Level Security and server-side owner checks scope records to the authenticated owner. The client does not assign ownership during Inbox conversion; the server derives it from the session and verifies related records before writing links.

The schema centers on owner-scoped tables for areas, goals, projects, tasks, resources, inbox items, reviews, habits, habit logs, tags, and settings. Relationship columns remain explicit rather than using a generic graph table, which keeps common reads and permitted edits understandable.

The responsive application includes PWA installation support and a deliberately narrow offline feature: quick captures can wait locally and sync into the Inbox after connectivity returns. Offline behavior is limited to the workflow where interruption would be most costly rather than implying that every application surface works offline.

## Engineering Takeaway

This build demonstrates product modeling as much as feature implementation. The main decision was to preserve a low-friction mode for uncertain input while still supporting a structured mode for planning and review. The result is a personal system whose complexity is concentrated in relationship inspection, ownership checks, and recovery paths instead of being imposed on every capture.
