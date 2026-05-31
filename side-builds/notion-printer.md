---
layout: project
title: "Notion Printer"
subtitle: "A self-contained print pipeline that turns exported Notion pages into readable, adjustable, print-ready HTML documents."
description: "Technical report for Notion Printer, a personal document automation tool for converting Notion HTML and ZIP exports into compact print-ready HTML with image handling, preview controls, page editing, and Ubuntu team distribution."
share-description: "Notion Printer technical report: a Python and browser-based print pipeline for converting exported Notion HTML and ZIP files into readable, adjustable, print-ready documents."
thumbnail-img: /assets/img/side-builds/notion-printer.png
share-img: /assets/img/side-builds/notion-printer.png
permalink: /side-builds/notion-printer/
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
quick_summary_note: "A technical summary for a side build focused on document automation, local conversion, and print readability."
quick_summary:
  - label: "Problem"
    value: "Notion pages are optimized for screen reading, so direct print/export workflows can produce oversized text, awkward image placement, and poor page breaks."
  - label: "Input"
    value: "Single or multiple Notion HTML files, ZIP exports, or mixed HTML/ZIP selections."
  - label: "Output"
    value: "Print, compact, fast preview, and compact-fast HTML variants with local preview assets."
  - label: "Runtime"
    value: "Python conversion pipeline plus browser preview/editor UI with Paged.js-based pagination."
  - label: "Distribution"
    value: "Ubuntu GUI launcher, desktop entry, and .deb package for team-friendly installation."
---

# Notion Printer Technical Report

## Abstract

Notion Printer is a local document automation tool I built because the normal Notion-to-paper workflow was too brittle for long, image-heavy, operation-style documents. A Notion page can look clean on screen, but printing it often turns into a manual layout problem: text scale feels wrong, images occupy awkward space, tables break poorly, and page boundaries appear in places that hurt readability.

The tool converts exported Notion HTML or ZIP files into a set of print-ready HTML documents. It preserves the document's useful structure, applies print-focused CSS, creates compact variants, generates fast preview assets for large images, opens a local browser preview, and gives the user controls to adjust page breaks, image sizes, spacing, and page-level edits before saving to PDF or printing.

![Notion Printer processing flow]({{ '/assets/img/side-builds/notion-printer-flow.svg' | relative_url }}){: .flowchart}

## 1. Background

Notion is excellent for writing, collecting references, and keeping operational documents alive, but that same screen-first design becomes a liability when the final artifact has to be read on paper. The practical problem was not "how do I print one page once?" It was "how do I repeatedly turn real Notion exports into readable documents without rebuilding the whole thing in Word, Google Docs, or a separate layout tool?"

The official routes are still export or browser print oriented: Notion supports PDF export, HTML export, and browser printing, while desktop direct printing depends on exporting first. That is enough for basic cases, but it does not solve the specific readability problems I kept hitting:

- body text and spacing that feel oversized or inefficient on A4 paper,
- images that land too large, too small, or detached from the sentence they explain,
- table rows and headers that break in visually confusing places,
- long nested lists that lose hierarchy when printed,
- multiple exported pages that need to become one coherent packet,
- and repeated documents that should not require manual relayout every time.

Notion Printer was designed as the missing middle layer between "exported Notion data" and "a document I can actually hand to someone."

## 2. Design Goals

The project targets five goals:

| Goal | Meaning |
| --- | --- |
| Print-first readability | Generate HTML that behaves like a paper document, not a web page squeezed into a printer dialog. |
| Low-friction input | Accept Notion HTML and ZIP exports directly, including multiple files selected in order. |
| Layout preservation | Keep useful Notion semantics such as status labels, nested bullets, image-caption groups, and table structure. |
| Manual correction loop | Let the user fix page breaks and image sizing inside a browser preview when automation cannot infer intent. |
| Team distribution | Package the tool as an Ubuntu launcher and `.deb` install so a teammate does not need to run scripts manually. |

The important design constraint is that this is not a cloud converter. It is a local workflow: exported files stay on the machine, the Python scripts transform local HTML, and the preview opens through a localhost server.

## 3. Existing Public Surface

The project has its own lightweight GitHub Pages site at [okj1223.github.io/notion_printer](https://okj1223.github.io/notion_printer/). That site is written for team installation and usage rather than portfolio explanation. It exposes:

- the current `v1.0.2` installation file,
- a "first use" guide,
- update/download pages,
- and release notes for changes such as ZIP input stabilization, fast preview defaults, and install/version guidance.

This portfolio page is different. It explains the engineering shape of the project: why the tool exists, how the pipeline works, what the converter changes, and which screenshots would make the report stronger later.

## 4. System Architecture

At a high level, the system has six layers.

| Layer | Main Files | Responsibility |
| --- | --- | --- |
| GUI launcher | `notion_print_export_launcher.sh` | Presents file selection and advanced options through Ubuntu GUI dialogs. |
| Input preparation | `scripts/prepare_notion_inputs.py` | Validates HTML input, extracts ZIP files safely, discovers real Notion page HTML, and deduplicates selected sources. |
| Integrated pipeline | `scripts/print_integrated_factory.py` | Combines one or more source documents into a single ordered print run and manages preferred outputs. |
| HTML converter | `notion_print_export.py` | Parses exported HTML, normalizes content, injects print CSS/runtime scripts, builds variants, and generates image preview assets. |
| Preview/editor runtime | `notion_print_export/runtime.js`, `theme.css`, `paged.polyfill.js` | Renders paginated preview, page controls, image resizing, break overrides, and print-mode cleanup in the browser. |
| Distribution | `scripts/build_deb_package.sh`, `.desktop` files | Builds an Ubuntu `.deb` package and exposes one team-facing app menu entry. |

The converter is intentionally file-based. A run produces normal HTML files plus local assets, which means the result can be opened, inspected, copied, archived, or printed without a hidden service dependency.

## 5. End-to-End Workflow

The standard user flow is:

1. Export a Notion page or workspace section as HTML or ZIP.
2. Open `Notion Printer` from the Ubuntu app menu.
3. Select one or more `.html` or `.zip` files.
4. Keep the default profile or adjust advanced options.
5. Let the pipeline generate print-ready variants.
6. Review the generated document in the browser preview.
7. Adjust page breaks, image sizes, or spacing when needed.
8. Print or save as PDF from the browser.

The default team-facing profile is compact and fast:

```text
Output profile: Compact Print + Fast Preview
TOC: off by default
Page numbers: on
Fast image quality: 1200px / WEBP 68
Open after completion: browser preview
```

This default exists because most exported Notion documents are too spacious for paper. The compact-fast output makes the preview responsive while giving the user a denser print layout as the starting point.

## 6. Input Handling

The input layer accepts both single-document and multi-document cases:

| Input Case | Behavior |
| --- | --- |
| Single HTML | Treated as one Notion source document and passed through the integrated pipeline. |
| Single ZIP | Extracted into `_notion_printer_imported_sources/`, then scanned for valid Notion page HTML. |
| Multiple HTML files | Combined in the selection order. |
| Multiple ZIP files | Extracted, scanned, and combined. |
| Mixed HTML + ZIP | Prepared into one deduplicated ordered source list. |

The ZIP handler is defensive. It validates member paths before extraction, rejects unsafe paths, extracts nested archives, and checks whether HTML candidates contain a Notion-style `.page-body` structure. Generated output HTML is not allowed back in as source input, which prevents accidental recursive conversion.

## 7. Conversion Strategy

The core converter does more than inject a stylesheet. It builds a lightweight HTML tree, annotates block-level structure, and records document metadata before writing the output variant. This matters because Notion export HTML is not a print layout. It is a web document that needs structural interpretation.

Important transformations include:

- removing the top property table when it is noise for printed reading,
- applying white-background, low-ink print styling,
- keeping description text and image blocks together where possible,
- preserving useful status label colors such as `PASS`, `WARN`, `FAIL`, `Success`, and `Pending`,
- removing unwanted bullet styling from numbered process markers such as `1-1`, `2-1`, or circled step symbols,
- keeping child bullets under their parent bullet instead of flattening the hierarchy,
- reducing table break issues with repeated headers and safer row handling,
- adding bottom page numbers when enabled,
- and injecting a manifest so the browser runtime knows the document, variant, hashes, and available controls.

The output variants are:

| Variant | Purpose |
| --- | --- |
| `_print.html` | Standard print-ready document. |
| `_print_compact.html` | Denser paper layout for long documents. |
| `_print_fast.html` | Standard layout with optimized preview assets. |
| `_print_compact_fast.html` | Default practical preview: compact layout plus faster image handling. |

## 8. Image and Asset Handling

Images are one of the main reasons the tool exists. Notion pages often contain screenshots, diagrams, and reference images that make sense on a wide screen but become inefficient on paper.

Fast variants create preview assets by downscaling local images and writing WEBP versions into a generated preview asset folder. The default fast setting uses a 1200px longest edge and WEBP quality 68. Generated preview assets are reused on later runs when possible, so repeated preview cycles are faster.

At preview time, the browser runtime also adds image controls. The user can adjust image scale, unify image widths across a selected group, or remove images that are not needed in a print packet. These changes are stored in browser local storage using document-specific keys, so the preview can reflow while preserving user intent.

## 9. Preview Editing

The browser preview is the most important part of the system because automatic pagination can never fully know what a human wants to preserve visually. Notion Printer uses Paged.js-style pagination to show paper pages on screen and then layers editing controls on top.

Preview controls include:

- page-start override candidates,
- drag-based page movement,
- "new page" versus "attach to previous page" choices,
- page sidebar thumbnails,
- image width controls,
- spacing controls,
- optional table-of-contents generation,
- text/edit mode for small cleanup,
- page insert, delete, and merge actions,
- and print-mode cleanup that hides editor UI before the browser print dialog.

This is the central product decision: the tool does not pretend that automatic print conversion is always perfect. It automates the repetitive 80 percent, then gives the user a focused correction interface for the last 20 percent.

## 10. Packaging and Release Flow

For personal use, the project can run directly from the repository:

```bash
./notion_print_export_launcher.sh
```

For team use, it builds an Ubuntu package:

```bash
./scripts/build_deb_package.sh
```

The package installs the app under `/opt/notion-printer`, exposes one launcher named `Notion Printer`, and opens the advanced GUI by default. This avoids asking non-developer users to clone the repository, install from source, or remember command-line options.

The release guide keeps versioning aligned across:

- the root `VERSION` file,
- the `.deb` file name,
- the Debian package metadata,
- the installed `/opt/notion-printer/VERSION`,
- and the public download/update pages.

## 11. What I Would Improve Next

The current tool is already practical, but the next product-quality pass would focus on making the preview loop more transparent:

- show a clearer before/after comparison for the same Notion export,
- export a small "print adjustments" JSON beside the final HTML,
- add a recoverable history panel for page edits,
- provide a test document set that demonstrates tables, images, callouts, and nested lists,
- and add a visual regression harness for preview page rendering.

## 12. Screenshots To Add Later

The report would become much stronger with a few real screenshots. These are the images I would add when available:

| Needed Image | Why It Helps |
| --- | --- |
| Original Notion browser/PDF print output | Shows the readability problem: oversized text, awkward image scale, or bad page breaks. |
| Same document after Notion Printer conversion | Makes the improvement immediately visible. |
| Ubuntu launcher / advanced options GUI | Shows that this is a usable app, not only a script. |
| ZIP or multi-file selection flow | Explains why the tool handles real Notion export folders better than manual file picking. |
| Browser preview with page frames | Demonstrates the paper-like preview surface. |
| Page edit panel / drag break controls | Shows how manual pagination correction works. |
| Image resize controls | Supports the key claim that image size and placement can be adjusted for print. |
| Download/update page or `.deb` install view | Shows the team-distribution side of the project. |

## 13. Related Links

- [Notion Printer project site](https://okj1223.github.io/notion_printer/)
- [Notion Printer repository](https://github.com/okj1223/notion_printer)
- [Notion Help: export and print options](https://www.notion.com/help/export-your-content)
