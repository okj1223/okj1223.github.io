---
layout: project
title: "Smart Learn"
subtitle: "A self-hosted study player for structured decks, durable review events, and low-friction desktop or mobile practice."
description: "A personal Python and browser-based study PWA with TSV decks, SQLite progress, smart review queues, offline event sync, English-only TTS, keyboard controls, and mobile access through Tailscale."
share-description: "Smart Learn is a self-hosted multi-deck study PWA with local progress, durable review events, focused controls, and desktop or phone access."
thumbnail-img: /assets/img/side-builds/smartlearn.png
share-img: /assets/img/side-builds/smartlearn.png
permalink: /side-builds/smartlearn/
category_label: "Study Tools - Personal PWA"
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
quick_summary_note: "A self-hosted personal study application, not a public learning platform."
quick_summary:
  - label: "Status"
    value: "Personal study application served from one Ubuntu machine."
  - label: "Problem"
    value: "Notebook-style vocabulary workflows made review controls, audio, and progress tracking unnecessarily fragile."
  - label: "Study Loop"
    value: "Choose a deck, flip a card, rate recall, persist the event, and schedule the next useful card."
  - label: "Access"
    value: "Desktop browser, compact mini window, and phone PWA through a private Tailscale connection."
  - label: "Stack"
    value: "Python HTTP server, structured TSV data, SQLite, plain JavaScript, IndexedDB, service worker, and browser or local TTS."
---

## Status and Scope

Smart Learn is a self-hosted personal study application. It runs from one Ubuntu machine and is intended for a small number of known users, not public registration or commercial deployment. The source is part of a private study workspace, so this page documents the implemented product and architecture without linking to a public repository.

![Smart Learn system flow]({{ '/assets/img/side-builds/smartlearn-flow.svg' | relative_url }}){: .flowchart}

## Problem

My earlier vocabulary workflow used notebook-style pages. That kept material in one place but made the actual review loop brittle: controls were not optimized for repeated recall, audio behavior was difficult to constrain, and progress was not represented as durable scheduling data.

Smart Learn treats study as a small player rather than a document. A normal session follows one short loop:

```text
Choose deck -> show card -> flip -> rate recall
-> persist review event -> update schedule -> show next card
```

The default controls are deliberately limited to **Still Learning** and **Knew It**. They map to the underlying scheduler while keeping the common interaction fast. Keyboard events support flipping, rating, speaking the current item, and opening a compact study window. On mobile, tap and swipe gestures provide the same core actions.

## Deck and Scheduling Model

Decks are structured TSV folders rather than hardcoded exam screens. The loader accepts common front-text, meaning, pronunciation, example, classification, and set metadata fields. The current personal collection includes TOEFL, AWL, TOEIC, and Traditional Chinese overlay data, while the application model remains usable for compatible structured decks.

Smart mode assembles a review queue from card state instead of simply opening a fixed daily file. It distinguishes new, due or overdue, weak, relearning, recovery, attention, and known cards. The scheduler also records the path through a session: immediate recall is different from a correct answer reached after one or more misses.

Review events therefore include identifiers, rating, timing, session attempts, misses, and recovery signals. An append-only `review_log` preserves event history, while a `progress` table stores the latest scheduling state for each user and card. User profiles share deck files but keep progress separated by `user_id`.

## Audio and Interaction Design

Text-to-speech is restricted to the current English learning item. The application avoids reading translations, IPA, examples, notes, interface labels, or the full card back. Browser `speechSynthesis` is the default free path, with an optional locally generated Piper cache when an English model is configured. Both server and client apply text checks before speech playback.

The same review model appears in three surfaces:

| Surface | Use |
| --- | --- |
| Desktop application | Deck selection, normal study, session details, and progress views. |
| Mini window | Compact review through Picture-in-Picture support or a popup fallback. |
| Phone PWA | Touch-first review through an HTTPS Tailscale address. |

Tailscale keeps phone access within the private network. The mobile layout accounts for standalone PWA display, safe areas, touch gestures, and undo feedback after swipe ratings.

## Offline Durability

The browser writes a pending review event to IndexedDB before advancing the interface. When the server is reachable, that event is pushed immediately; when connectivity drops, it remains queued and retries after reconnection, focus, resume, or a periodic sync trigger. Each event has an `event_id`, allowing the server to handle repeated delivery without intentionally applying the same review twice.

This boundary matters because the server owns the durable SQLite state while the PWA may temporarily lose its Tailscale connection. Offline support protects review input; it does not claim that every server-backed feature remains available without the host.

## Technical Architecture

The server uses Python's `ThreadingHTTPServer` for deck loading, static assets, progress APIs, token handling, and optional TTS endpoints. The browser client is plain HTML, CSS, and JavaScript. SQLite stores review history and current progress, while IndexedDB holds pending events, cached cards, active session data, and selected client metadata. A web manifest and service worker provide installable PWA behavior.

## Engineering Takeaway

Smart Learn shows how interaction design and data durability reinforce each other. A minimal control surface makes frequent study practical, while append-only events, idempotent sync, and explicit scheduler states make that simple interaction recoverable across desktop, phone, and intermittent connectivity.
