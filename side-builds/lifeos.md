---
layout: project
title: "LifeOS"
subtitle: "A private personal operating system built around fast capture, later clarification, visible relationships, and calmer execution."
description: "Technical report for LifeOS, a private Next.js and Supabase personal operating system for capture, inbox triage, tasks, projects, goals, resources, reviews, habits, search, archive, export, PWA install, offline quick capture, and relationship inspection through Structure."
share-description: "LifeOS technical report: a private personal operating system for reducing planning friction through quick capture, inbox conversion, relationship-aware structure views, reviews, and owner-only data control."
thumbnail-img: /assets/img/side-builds/lifeos.png
share-img: /assets/img/side-builds/lifeos.png
permalink: /side-builds/lifeos/
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
quick_summary_note: "A technical and product summary for a side build focused on reducing planning overhead without losing structure."
quick_summary:
  - label: "Problem"
    value: "Many planners make users plan before they can capture, turning a quick thought into a form-filling session."
  - label: "Core Idea"
    value: "Capture loosely first, clarify later, and connect entities only when the connection makes future execution clearer."
  - label: "Main Loop"
    value: "Capture → Clarify → Organize → Execute → Review."
  - label: "Key Surface"
    value: "Structure shows Areas, Goals, Projects, Tasks, Resources, Habits, Reviews, and Inbox items as an inspectable system."
  - label: "Stack"
    value: "Next.js App Router, TypeScript, Supabase Auth/Postgres/RLS, Server Actions, Tailwind, and PWA/offline capture support."
---

# LifeOS Technical Report

## Abstract

LifeOS is a private productivity system I built because conventional planners often make planning itself too expensive. In many tools, even a small thought asks for a title, project, deadline, calendar slot, priority, tags, and a parent category before it feels "properly captured." That is friction at the worst possible moment: the moment when the user simply needs to get an idea out of their head.

The core product stance is different: **throw things in quickly, clarify them later, and only add structure when it helps execution or future retrieval.** LifeOS starts with fast capture into an Inbox, then converts raw inputs into Tasks, Projects, Goals, Resources, or Archive when the user has time to think. The Structure surface then makes the overall planning shape visible, so the system does not become a pile of disconnected tasks.

![LifeOS operating loop]({{ '/assets/img/side-builds/lifeos-flow.svg' | relative_url }}){: .flowchart}

## 1. Product Motivation

The problem is not that people lack planning apps. The problem is that many planners confuse **capturing reality** with **pre-designing reality**. If every input must immediately become a perfectly titled task with a due date, project, area, priority, and calendar block, the user spends too much energy maintaining the system instead of doing the work.

LifeOS is built around a more forgiving rhythm:

- capture before judgment,
- separate action from information,
- keep projects tied to next actions,
- treat areas as maintained contexts rather than strict folders,
- use reviews to repair drift,
- archive old items to keep active views clear,
- and use search/export so data ownership stays trustworthy.

This makes LifeOS closer to a personal operating system than a todo list. It is not trying to force every thought into the right container immediately. It gives thoughts a safe temporary place, then provides tools to refine them later.

## 2. Operating Model

The LifeOS loop is:

```text
Capture → Clarify → Organize → Execute → Review
```

| Stage | Meaning | Product Surface |
| --- | --- | --- |
| Capture | Get thoughts, tasks, links, and ideas out of the head quickly. | Dashboard Quick Capture, Inbox capture form, floating `+`, offline capture. |
| Clarify | Decide what the input actually is. | Inbox list, filters, edit, Triage Mode, Convert modal. |
| Organize | Attach only the useful relationships. | Task/Project/Goal/Resource forms, relation selects, Structure Focus. |
| Execute | Show what matters now. | Dashboard, Today Tasks, Active Projects, Goals Snapshot. |
| Review | Close the loop and repair drift. | Daily Review, Weekly Review, Review-to-Action. |

The essential rule is that capture should be cheap. Planning depth comes later.

## 3. Entity System

LifeOS models personal work through a small set of entity types.

| Entity | Role |
| --- | --- |
| Inbox Item | Raw, unprocessed input. This is where unclear thoughts belong first. |
| Task | A single executable action. |
| Project | A multi-step outcome that needs Tasks and a next action. |
| Goal | A time-bounded direction that can group Projects. |
| Area | A maintained life domain such as research, health, career, English, or personal projects. |
| Resource | Information that should be recoverable later: links, notes, papers, videos, ideas, code. |
| Review | A daily or weekly feedback loop that can produce next actions. |
| Habit | A success log for recurring behavior. |
| Archive | A clarity layer for things that should be hidden but recoverable. |

This model avoids one common planner failure: treating every input as a task. A paper link is not a task. "Build robot" is not a task. "Health" is not a project. LifeOS gives each kind of input a place, but does not require the user to know the right place at capture time.

## 4. Fast Capture and Inbox Triage

Quick Capture is the front door. It appears on the Dashboard, on the Inbox page, and as a floating capture button inside the protected app. The mobile/PWA path also supports a narrow offline capture queue: when the device is offline, capture items are held locally and later synced into the owner-guarded Inbox.

The Inbox keeps raw input intentionally unprocessed. From there the user can:

- filter by unprocessed, converted, archived, or all,
- search raw text,
- edit captured text,
- archive or restore items,
- permanently delete truly bad inputs,
- move through one item at a time in Triage Mode,
- and convert an item into a Task, Project, Goal, or Resource.

The important design choice is that conversion happens when the user is ready to think. A raw capture can be messy. During conversion, LifeOS seeds useful defaults from the raw text but lets the user decide whether it is an action, outcome, direction, or reference.

## 5. Conversion Rules

LifeOS uses simple decision rules:

| Raw Input Type | Better Destination | Why |
| --- | --- | --- |
| One executable action | Task | It can be done directly. |
| Two or more steps toward a result | Project | It needs multiple Tasks. |
| Time-bounded direction | Goal | It groups effort over a period. |
| Link, note, paper, video, idea, code | Resource | It is information, not execution. |
| Maintained life domain | Area | It is ongoing context, not a finishable outcome. |
| Not needed now | Archive | It should leave active views without being destroyed. |

Server Actions perform conversion with the current owner session. The client does not provide `user_id`; the server assigns ownership and updates the Inbox item as converted. This keeps the capture pipeline private and owner-scoped.

## 6. Execution Layer

The Dashboard is not just a homepage. It is the command surface:

- Quick Capture for new inputs,
- Today Tasks for due/planned/critical work,
- Inbox count for unresolved inputs,
- Habit status for daily routine,
- Active Projects with progress and health signals,
- Goals Snapshot,
- Recent Resources,
- Review call-to-action,
- PWA install and sync status cards.

Tasks have statuses such as `next`, `scheduled`, `waiting`, `done`, and `archived`. They also carry kind, priority, energy, planned time, due date, estimated minutes, actual minutes, and optional relationships to Project, Goal, and Area.

Today Tasks are designed to stay clear. They include work due today or earlier, work intentionally planned for today, and critical tasks. That makes the daily execution view about the next movement, not about every possible responsibility.

## 7. Projects, Goals, and Areas

Projects are the bridge between big outcomes and executable tasks. A Project stores a desired outcome and a next action hint because the most common failure mode is not "I forgot the project exists." It is "I opened the project and still did not know what to do next."

Goals provide time-bounded direction. Areas provide ongoing context. The distinction matters:

- a Goal can complete,
- a Project can complete,
- a Task can complete,
- but an Area is maintained.

LifeOS intentionally does not force every item into an Area. Areas are useful when they clarify context, but mandatory taxonomy would recreate the planning friction the app is trying to remove.

## 8. Structure

Structure is the feature that makes the system more than a set of lists. It answers questions like:

- Which Projects have no Tasks?
- Which Projects have no next action?
- Which Goals have not become Projects yet?
- Which Resources are floating without Area, Goal, or Project context?
- Which Tasks are detached from larger work?
- Is this Area cluttered with direct Tasks that should become Projects?

The Structure surface includes:

| Mode | Purpose |
| --- | --- |
| Overview | Counts, active system summary, relationship health, and common flow cards. |
| Database | Read-only table browser for Areas, Goals, Projects, Tasks, Resources, Habits, Reviews, and Inbox. |
| Focus | Center one entity and inspect related items, with explicit Manage Links for allowed relationship fields. |
| Cleanup | Calm relationship cleanup candidates, such as projects without tasks or resources without links. |
| Structure Map | Scope-based lanes for Area, Goal, or Project so relationships can be inspected without a dense graph. |

The Structure Map intentionally moved away from a line-heavy graph. In personal systems, relationships are optional and cross-cutting, so too many edge lines can make the truth harder to read. LifeOS uses lanes instead: Context, Execution, Knowledge, Routine, and Signals.

## 9. Reviews and Habits

Reviews keep the system honest. Daily Review closes the day; Weekly Review inspects Inbox, Projects, Resources, Habits, and next actions. Review is not just journaling. It exists to produce the next actionable movement.

Habits are treated as success logs. LifeOS stores completed days, not failure logs. That makes the habit system psychologically lighter: a missed day is simply an empty day, not a red mark that makes the system harder to return to.

## 10. Search, Archive, Export

Search is memory recovery. LifeOS can search across Tasks, Projects, Goals, Areas, Resources, Reviews, Habits, and Inbox items, including archived items when needed. This prevents the common problem where users keep old material active just because they are afraid they will not find it later.

Archive is part of clarity. It hides things that no longer need active attention while keeping them restorable and searchable.

Export protects data ownership. Settings provides export paths for full JSON, task/project CSV, resource/review text, and habit data. Import is not implemented yet, but the export surface makes the private system more trustworthy.

## 11. Technical Architecture

LifeOS is a private Next.js application:

| Layer | Implementation |
| --- | --- |
| App framework | Next.js App Router with TypeScript and React Server Components. |
| Mutations | Server Actions with Zod validators. |
| Database | Supabase Postgres with user-owned tables and relational columns. |
| Auth | Supabase Auth plus an `OWNER_EMAIL` allowlist and protected routes. |
| Security | Supabase Row Level Security and server-side owner checks. |
| UI | Tailwind CSS, local shadcn/base-ui style components, Lucide icons. |
| Mobile | Responsive shell, mobile bottom navigation, PWA manifest, service worker, install prompt. |
| Offline | Narrow offline quick capture queue that syncs into Inbox when online. |

The database schema centers on user-owned tables: `areas`, `goals`, `projects`, `tasks`, `resources`, `inbox_items`, `reviews`, `habits`, `habit_logs`, `tags`, `taggings`, and `app_settings`.

Core relationship columns include:

- `goals.primary_area_id`,
- `projects.goal_id` and `projects.primary_area_id`,
- `tasks.project_id`, `tasks.goal_id`, and `tasks.primary_area_id`,
- `resources.project_id`, `resources.goal_id`, and `resources.primary_area_id`,
- `habits.goal_id` and `habits.primary_area_id`,
- `inbox_items.converted_type` and `converted_id`.

Structure reads and relationship writes verify that every involved entity belongs to the current owner. This keeps the relationship manager narrow and explicit instead of becoming a risky graph editor.

## 12. Product Judgment

The best part of LifeOS is not the number of modules. It is the restraint in the operating model. The user can start with messy input and still recover structure later.

That means the system supports two different modes of thinking:

- **low-friction mode:** "I just need to throw this somewhere safe."
- **structural mode:** "Now I want to see what this belongs to and what it implies."

Most planners over-index on the second mode. LifeOS tries to protect the first mode so the second mode can happen at the right time.

## 13. What I Would Improve Next

The next useful improvements would be:

- a smarter Inbox suggestion layer for likely Task/Project/Resource classification,
- saved Structure views for recurring weekly checks,
- relationship history so major structure edits are auditable,
- explicit Review-to-Action lineage,
- import support to match the current export ownership story,
- and more offline surfaces beyond Quick Capture if the app becomes a true travel/phone-first system.

## 14. Screenshots To Add Later

The report would be much stronger with real screenshots from the private app. These are the images I would add when available:

| Needed Image | Why It Helps |
| --- | --- |
| Dashboard with Quick Capture and Today Tasks | Shows the "capture fast, execute clearly" center of the app. |
| Floating capture on mobile | Proves the low-friction input path. |
| Inbox Triage Mode | Shows raw items being clarified one by one. |
| Convert Inbox Item modal | Makes the Task/Project/Goal/Resource conversion model visible. |
| Project detail page with next action and linked Tasks | Shows how big outcomes become execution. |
| Structure Overview | Shows system-wide counts and relationship health. |
| Structure Map for a Project or Goal | Shows the lane-based view of Context, Execution, Knowledge, Routine, and Signals. |
| Cleanup mode | Shows how LifeOS surfaces gaps calmly rather than as failure warnings. |
| Daily or Weekly Review | Shows the feedback loop that turns drift back into action. |
| Settings export/PWA/offline status | Shows private ownership, backup, and mobile use. |

## 15. Related Links

- [LifeOS repository](https://github.com/okj1223/LifeOS)
