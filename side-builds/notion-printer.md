---
layout: project
title: "Notion Printer"
subtitle: "A local print pipeline that turns exported Notion pages into adjustable, print-ready HTML."
description: "A personal document automation utility that converts Notion HTML and ZIP exports into print-focused HTML with asset handling, paginated preview controls, and Ubuntu packaging."
share-description: "Notion Printer is a local Python and browser-based pipeline for converting exported Notion documents into adjustable, print-ready HTML."
thumbnail-img: /assets/img/side-builds/notion-printer.png
share-img: /assets/img/side-builds/notion-printer.png
permalink: /side-builds/notion-printer/
category_label: "Document Automation - Personal Utility"
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
quick_summary_note: "An active personal utility for repeatable, local document conversion."
quick_summary:
  - label: "Status"
    value: "Active personal utility with an Ubuntu packaging path."
  - label: "Problem"
    value: "Screen-oriented Notion exports require repeated manual work to become readable print documents."
  - label: "Input"
    value: "One or more Notion HTML files, ZIP exports, or mixed selections."
  - label: "Output"
    value: "Standard, compact, fast-preview, and compact fast-preview HTML variants."
  - label: "Stack"
    value: "Python conversion scripts, browser preview controls, Paged.js, local assets, and Ubuntu packaging."
---

## Status and Scope

Notion Printer is an active personal document automation utility. I built it to make repeatable print preparation easier for long, image-heavy Notion exports. It runs locally, produces ordinary HTML and asset files, and includes an Ubuntu packaging path. It is not a hosted conversion service, and its source repository is private.

![Notion Printer processing flow]({{ '/assets/img/side-builds/notion-printer-flow.svg' | relative_url }}){: .flowchart}

## Problem

Notion pages are designed for screen reading. When an exported page must become a paper packet or PDF, useful web layout choices can produce inefficient spacing, oversized images, awkward table breaks, and headings or list items separated from the content they introduce. Repeating those corrections in a general-purpose editor turns every export into a manual layout task.

The goal was not to replace a full publishing system. It was to automate the recurring conversion work while preserving a focused human correction step for pagination decisions that software cannot reliably infer.

## End-to-End Workflow

The user selects one or more `.html` or `.zip` exports through an Ubuntu launcher. The preparation layer validates the inputs, extracts archives, finds eligible Notion page HTML, removes duplicates, and preserves source order. The integrated pipeline then combines the selected documents and generates print-focused variants:

| Variant | Purpose |
| --- | --- |
| Standard print | Normal print-ready layout. |
| Compact print | Denser typography and spacing for longer documents. |
| Fast preview | Uses optimized local image assets for a more responsive preview. |
| Compact fast preview | Combines the denser layout with optimized preview assets. |

The result opens in a local browser preview. The user can inspect paper-like pages, adjust page starts, change image widths and spacing, make small page edits, and then print or save to PDF through the browser.

## Conversion Architecture

The project is separated into six practical layers:

| Layer | Responsibility |
| --- | --- |
| GUI launcher | Collects file selections and conversion options. |
| Input preparation | Validates HTML, safely extracts ZIP files, and discovers source pages. |
| Integrated pipeline | Orders and combines multiple source documents. |
| HTML converter | Parses document structure, applies print rules, and writes variants. |
| Preview runtime | Paginates the result and exposes correction controls. |
| Packaging | Builds the Ubuntu application package and desktop entry. |

The file-based boundary is intentional. Generated documents remain inspectable and usable without a hidden server dependency. ZIP member paths are checked before extraction, generated output is excluded from future source discovery, and only pages matching the expected Notion document structure enter the conversion pipeline.

## Print-Specific Decisions

The converter does more than attach a stylesheet. It interprets block structure so it can keep descriptions near related images, preserve nested list hierarchy, reduce unwanted table splits, repeat table headers where appropriate, and retain meaningful status-label styling. Optional page numbers and document metadata are injected with the runtime configuration.

Images receive a separate fast-preview path. Local sources can be resized into generated WebP assets, which improves repeated browser preview cycles while leaving the original files available. Preview controls allow individual or grouped width adjustments and can suppress images that do not belong in the final print packet.

The paginated preview is the main product decision. Fully automatic pagination is brittle because the correct break depends on meaning, not only element height. The preview therefore automates routine layout first and exposes targeted controls for page-start overrides, image sizing, spacing, page insertion or merging, and print-mode cleanup. User adjustments are associated with the document so the layout can be revisited without starting from zero.

## Engineering Takeaway

Notion Printer demonstrates a pragmatic automation boundary: structured parsing handles repeatable transformations, while the browser provides a direct correction loop for subjective layout choices. The work combines defensive file handling, HTML transformation, print CSS, browser state, and desktop distribution in one local workflow without presenting the utility as a general cloud product.
