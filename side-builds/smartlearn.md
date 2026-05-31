---
layout: project
title: "Smart Learn"
subtitle: "A multi-deck study player built for keyboard-friendly review, English-only TTS, mobile PWA access, and durable local progress."
description: "Technical report for Smart Learn, a local Python and browser-based multi-deck study PWA with structured TSV decks, smart review queues, SQLite progress, offline pending reviews, English-only TTS, keyboard and swipe controls, mini window study, and iPhone access through Tailscale."
share-description: "Smart Learn technical report: a multi-deck study PWA built after notebook-style study workflows became too brittle, with smart scheduling, keyboard-friendly controls, TTS, offline sync, and phone access."
thumbnail-img: /assets/img/side-builds/smartlearn.png
share-img: /assets/img/side-builds/smartlearn.png
permalink: /side-builds/smartlearn/
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
quick_summary_note: "A technical and product summary for a side build focused on turning vocabulary review into a controllable, phone-ready, low-friction study loop."
quick_summary:
  - label: "Problem"
    value: "Notebook-style study workflows were too brittle: text rendering could break, audio was unreliable, and the interaction model was poor for treadmill or phone study."
  - label: "Core Idea"
    value: "Build a study player that reads structured deck files, schedules only useful review cards, and can be controlled through simple keys, swipes, and TTS."
  - label: "Input"
    value: "Structured TSV deck folders. The current repo includes TOEFL, AWL, TOEIC, and zh-TW overlays, but the model is not limited to those exams."
  - label: "Runtime"
    value: "Lightweight Python HTTP server, browser UI, PWA shell, service worker, IndexedDB pending queue, and SQLite progress database."
  - label: "Controls"
    value: "Space to flip, arrow/number keys to rate, S or up arrow for English TTS, M for mini mode, mobile tap/swipe controls, and controller-friendly key mapping."
---

# Smart Learn Technical Report

## Abstract

Smart Learn is a self-hosted multi-deck study player I built after notebook-style vocabulary study became too annoying to tolerate. The original workflow used GoodNotes, but the actual study experience kept fighting back: text rendering could break, audio output was not trustworthy enough, and the controls were not flexible for the situations where I wanted to study.

The product goal is simple: make vocabulary review feel like a controllable player, not a fragile notebook page. It runs from a lightweight local Python server, opens in a desktop browser or phone PWA, reads structured TSV deck folders, stores progress in SQLite, queues review events offline when the network drops, and uses a Smart scheduler to bring back due, weak, recovery, relearning, attention, and new cards.

The important detail is that it is not only a TOEIC or TOEFL tool. The current repository contains TOEFL, AWL, TOEIC, and zh-TW overlay data, but the app itself is a multi-deck study surface for structured vocabulary decks.

![Smart Learn system flow]({{ '/assets/img/side-builds/smartlearn-flow.svg' | relative_url }}){: .flowchart}

## 1. Product Motivation

GoodNotes was useful as a place to hold study material, but it was not built for this exact job. When text rendering is unreliable or audio pronunciation feels wrong, the study loop becomes noisy. The user stops thinking about the word and starts managing the tool.

Smart Learn was built around the opposite assumption: the study interface should stay out of the way. It should be usable while walking on a treadmill, sitting away from the keyboard, or checking a few cards on a phone. That required:

- predictable text rendering,
- English-only word pronunciation,
- very simple review actions,
- keyboard shortcuts that can be remapped to a small external controller,
- mobile gestures for quick phone use,
- a mini study window for desktop focus,
- and durable progress that survives refreshes, offline periods, and device switches.

The app is self-hosted and designed for a small profile-based deployment. It is not a public learning platform. That makes the architecture much simpler and more honest: one Ubuntu machine can serve the app, SQLite can own progress, and Tailscale can provide phone access without paying for cloud infrastructure.

## 2. Study Loop

The core loop is intentionally small.

```text
Choose folder or set
  -> start Smart or All mode
  -> flip the card
  -> mark Still Learning or Knew It
  -> persist review event
  -> update progress and scheduler state
  -> show the next useful card
```

The main buttons are only `Still Learning` and `Knew It`. Internally those normalize to `again` and `good`. The app still has advanced `Again`, `Hard`, `Good`, and `Easy` controls, but they start hidden because the default review path should be fast enough to use while moving.

This is also why the keyboard map is simple:

| Input | Action |
| --- | --- |
| `Space` | Flip front/back. |
| `1` or left arrow | Still Learning. |
| `2` or right arrow | Knew It. |
| `S` or up arrow | Speak the current English learning item. |
| `M` | Open the mini study window. |
| Mobile tap | Flip the card. |
| Mobile left swipe | Still Learning. |
| Mobile right swipe | Knew It. |

Because the controls are ordinary keyboard events, an external controller or remote can be mapped to the same keys. That makes treadmill study practical without adding a native gamepad layer.

## 3. Data Model And Deck Loading

Smart Learn reads TSV decks directly from the repository. The current deck loader supports TOEFL, AWL, TOEIC, and related structured file patterns, but the important product model is broader: each deck is a structured folder of cards, not a hardcoded exam screen.

Supported card fields include:

| Field Family | Examples |
| --- | --- |
| Front text | `word`, `english`, `phrase` |
| Meaning | `meaning`, `meaning_ko`, `meaning_zh_tw`, `back_zh_tw` |
| Pronunciation and examples | `ipa`, `example`, `example_en`, `example_zh_tw` |
| Classification | `part_of_speech`, `topic`, `tag`, `difficulty`, `source` |
| Deck metadata | `set_id`, `set_no`, `set_title` |

The current repository contains 82 TSV files under deck folders and overlays. It includes TOEFL ETS 2026 sets, AWL sets, TOEIC vocabulary sets, and zh-TW overlay files. The app displays meaning fields according to the current user language, but it does not use paid translation APIs or automatic translation during study.

## 4. Smart Scheduler

Smart mode is not just "show today's deck." It decides which cards are currently worth seeing. The scheduler tracks card-level progress through `progress.sqlite3` and append-only `review_log` events.

Important scheduler states include:

| State | Meaning |
| --- | --- |
| `new` | A card that has not been learned yet. |
| `due` / `overdue` | A learned card whose review interval has arrived. |
| `weak` | A card with difficulty, low retrievability, or weak-pass signals. |
| `relearning` | A card currently unresolved after a miss. |
| `recovery` | A card recovered after one or more misses. |
| `attention` | A card repeatedly recovered across sessions and worth extra focus. |
| `known` | Stable enough to stay out of today's active queue. |

The scheduler treats the path inside a session as real signal. A clean first-attempt `Knew It` is stronger evidence than a card that needed one or more `Still Learning` presses before recovery. Review events store session attempt count, misses, recovered status, immediate pass status, elapsed time, and first-success latency so later scheduling can distinguish clean recall from fragile recall.

The default target retention in code is `DESIRED_RETENTION = 0.92`. That does not mean every day gets a fixed workload cap. Smart count means "cards currently eligible by bucket," not "force this many cards today."

## 5. TTS Design

TTS was one of the reasons to build the app. The design rule is strict: Smart Learn speaks only the current English learning item. It should not read Korean meanings, Chinese meanings, IPA, examples, notes, UI labels, or the entire card back.

The app uses two free paths:

| Provider | Behavior |
| --- | --- |
| Browser `speechSynthesis` | Default free TTS path, usually enough for desktop and phone. |
| Local Piper cache | Optional local TTS cache if Piper and an English model are installed. |

The server and browser both validate TTS text. They reject non-English scripts and reject text that looks like IPA, examples, notes, or full meaning text. The voice is normalized toward English such as `en-US`, even when the user interface language is zh-TW.

## 6. Desktop, Mini Window, And Phone PWA

Smart Learn has three study surfaces.

| Surface | Purpose |
| --- | --- |
| Full desktop app | Folder/set browsing, stats, options, word list, session info, and normal study. |
| Mini window | A small review surface for focused desktop use. It can use Document Picture-in-Picture when available, otherwise a popup. |
| iPhone PWA | Safari Add to Home Screen flow through a Tailscale URL. |

The mobile layout is not an afterthought. It supports safe-area padding for the iPhone home indicator, standalone PWA display, card tap to flip, swipe ratings, undo toast for swipe reviews, and a service worker shell cache.

Tailscale is the network access layer. The Ubuntu machine runs the server, Tailscale Serve exposes the app inside the tailnet, and the iPhone opens the HTTPS tailnet URL. This keeps the app off the public internet unless the user explicitly chooses otherwise.

## 7. Offline Sync And Durability

Progress should not depend on perfect connectivity. The browser stores pending review events in IndexedDB before the UI advances. If the server is reachable, events are pushed immediately. If Tailscale or the network drops, the app keeps the events in `pending_reviews` and syncs later.

The sync model has two main tables:

| Store | Role |
| --- | --- |
| `review_log` | Append-only event history. It records each rating event, device/session metadata, timing, and session-path signals. |
| `progress` | Current scheduling state per user and card. It mirrors the latest scheduling summary. |

The API makes event push idempotent with `event_id`, so repeated sync attempts should not duplicate progress. Sync triggers include app load, rating input, browser online event, focus/resume, visibility changes, and a periodic retry while pending events exist.

## 8. Multi-User Profiles

The app supports a small multi-user profile model. The same SQLite database separates progress by `user_id`, so another learner can use the same server and deck files without mixing review history.

| User Profile | Default Behavior |
| --- | --- |
| `learner_ko` | Korean meaning preference, TOEFL-first ordering. |
| `learner_zh_tw` | zh-TW meaning preference, TOEIC-first ordering. |

This is not a hard visibility restriction. Each learner can open TOEFL, TOEIC, AWL, or any compatible deck folder. The default exam type only controls initial ordering and default direction. Study progress and review history stay separate per profile.

Tokens can be configured through environment variables or CLI options. Token values are not printed in the app UI, and the token login screen is intentionally simple.

## 9. Technical Architecture

The stack is deliberately compact.

| Layer | Implementation |
| --- | --- |
| Server | Python `ThreadingHTTPServer` with deck loading, sync APIs, auth token handling, TTS endpoints, and static file serving. |
| Data | Structured TSV deck files plus `progress.sqlite3`. |
| Client | Plain browser app in `app.js`, `index.html`, and CSS files. |
| Mini mode | `mini.html`, `mini.js`, `mini.css` with the same review-event durability model. |
| PWA | Web manifest, service worker, icon set, network-first shell updates, and cached static assets. |
| Offline | IndexedDB stores for card cache, progress cache, active sessions, pending reviews, app metadata, and audio cache metadata. |
| Access | Localhost on desktop; Tailscale Serve for iPhone/PWA access. |

The validation scripts cover important contracts such as Smart counts, TOEIC/TOEFL data shape, zh-TW overlays, sharing behavior, TTS English-only behavior, track validation, and phase-level PWA/sync contracts.

## 10. Product Judgment

The best part of Smart Learn is that it removes the small frictions that make studying collapse. A vocabulary app can have excellent content and still fail if the interaction loop is awkward. The study surface has to be fast, predictable, and forgiving.

For this project, that means:

- no fragile document rendering,
- no accidental reading of the wrong text,
- no complex rating ritual as the default,
- no cloud requirement for phone/PWA access,
- no loss of progress when the network drops,
- and no assumption that study happens only while sitting still at a desk.

It is a small tool, but the product idea is strong: make study behave like a reliable control surface.

## 11. What I Would Improve Next

The next useful improvements would be:

- add a true gamepad/controller settings panel if controller use becomes frequent,
- make deck import more explicit for arbitrary new TSV folders,
- add a calibration view for retention, recovery, and attention thresholds,
- improve local Piper setup guidance and voice selection,
- add richer progress history charts from `review_log`,
- and create a screenshot-driven mobile validation checklist for each PWA release.

## 12. Screenshots To Add Later

The report would become much stronger with real screenshots. These are the images I would add when available:

| Needed Image | Why It Helps |
| --- | --- |
| GoodNotes-style source workflow problem | Shows why the custom player was worth building. |
| Desktop folder/set lobby | Shows the multi-deck structure. |
| Study card front and back | Shows the clean review surface. |
| Keyboard help panel | Shows the controller-friendly shortcut design. |
| Mini/PiP study window | Shows the treadmill or low-attention desktop mode. |
| iPhone PWA home-screen launch | Shows phone-first access. |
| Mobile swipe review with undo toast | Shows the touch interaction model. |
| TTS speaker button and English-only behavior | Shows that audio is scoped to the current word. |
| Offline pending sync indicator | Shows durability while Tailscale/network is down. |
| SQLite or review-log summary | Shows that progress is durable and auditable. |

## 13. Related Links

- [Smart Learn repository path](https://github.com/okj1223/toefl/tree/main/smartlearn)
