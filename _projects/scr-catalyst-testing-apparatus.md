---
layout: project
date: 2024-04-06
title: "Bench-Scale SCR Catalyst Test Apparatus"
card_title: "SCR Catalyst Test Apparatus"
description: "A built bench-scale apparatus for controlling simulated flue-gas inputs, reactor temperature, and analyzer sampling during SCR catalyst tests."
card_description: "An integrated laboratory apparatus combining multi-gas delivery, thermal control, a catalyst reactor, and flue-gas measurement."
share-description: "A bench-scale SCR catalyst test apparatus with controlled gas delivery, multiple heating zones, and flue-gas analysis."
subtitle: "Laboratory hardware for repeatable gas delivery, thermal conditioning, and catalyst-test measurements."
thumbnail-img: /project/scr-catalyst-testing-apparatus/completed_apparatus.webp
permalink: /projects/scr-catalyst-testing-apparatus/
filter_categories:
  - environmental
  - control
category_label: "Environmental Engineering - Catalyst Technology"
impacts:
  - value: "Built"
    label: "Integrated Apparatus"
  - value: "Multi-Gas"
    label: "Controlled Supply"
  - value: "Bench"
    label: "Test Scale"
tech_tags:
  - label: "SCR"
    style: "eng"
  - label: "MFC Control"
    style: "sensor"
  - label: "Thermal Control"
    style: "eng"
  - label: "Testo 350K"
    style: "sensor"
  - label: "Gas Analysis"
    style: "data"
quick_summary_note: "The page documents the physical apparatus, test workflow, and evidence still required for quantitative reporting."
quick_summary:
  - label: "Context"
    value: "Bench-scale environmental engineering apparatus build"
  - label: "My Role"
    value: "Apparatus-level design and integration contribution"
  - label: "Hardware"
    value: "Gas supply panel, flow control, heated reactor, analyzer, and control panel"
  - label: "Method"
    value: "Condition gas and temperature, expose a sample, and compare analyzer readings"
  - label: "Status"
    value: "Completed apparatus; calibration and result records are not included here"
---

## Project Overview

This project developed a bench-scale apparatus for testing SCR catalyst samples under controlled gas and temperature conditions. The physical system combines gas supply and flow control, a heated reactor section, an operator panel, and a Testo 350K flue-gas analyzer.

The project notes cite VGB-R 302 and EPRI test guidance as design references. Because the archive does not include a formal compliance assessment, traceable calibration package, or third-party verification, this page does **not** describe the apparatus as standards-compliant. It documents the engineering build and intended test workflow instead.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/scr-catalyst-testing-apparatus/completed_apparatus.webp' | relative_url }}"
       alt="Completed bench-scale SCR catalyst test apparatus with control panel, gas controls, heated section, and analyzer"
       loading="lazy">
  <figcaption>The completed apparatus integrating gas handling, thermal control, the reactor section, and flue-gas measurement.</figcaption>
</figure>

## Context and Role

My contribution was the apparatus-level engineering represented here: translating the test concept into an integrated arrangement of gas handling, heating, sample exposure, controls, and measurement. The archived material does not preserve a reliable subsystem-by-subsystem ownership breakdown, so I do not claim that every fabricated part, instrument setting, or procedure was solely my work.

The useful portfolio evidence is the completed hardware and the way the subsystems were organized. Raw commissioning and catalyst-test records are not available in this site repository, so the case study does not report quantitative results.

## Test Objective

An SCR catalyst test needs more than an analyzer reading. The sample must see a defined gas mixture at a controlled temperature, flow must be stable enough to compare runs, and the measurement path must distinguish upstream conditions from the gas leaving the reactor.

The apparatus was organized around that sequence:

1. select and regulate the required gas supplies;
2. set the flow-control channels;
3. heat and stabilize the reactor zones;
4. place a prepared catalyst sample in the test section;
5. establish the baseline gas condition;
6. introduce the reducing gas according to the test plan;
7. record analyzer readings and operating conditions; and
8. purge and shut down the apparatus in a controlled order.

This workflow provides a basis for repeatable experiments, but repeatability must be demonstrated with calibration and run data rather than inferred from the equipment layout.

## Apparatus Architecture

### Gas Delivery

The project notes identify nitrogen, oxygen, nitric oxide, sulfur dioxide, and ammonia among the simulated flue-gas inputs. The front panel provides separate pressure and flow controls for the supply paths. Mass-flow control was used so that a test operator could set each component independently before mixing and sending the stream toward the reactor.

Handling these gases is a significant safety responsibility. Regulators, compatible tubing and seals, ventilation, gas detection, check valves, purge logic, and shutdown procedures all need to be selected and verified for the actual installation. The presence of controls in the prototype photograph is not, by itself, proof of a completed process-safety review.

### Thermal Section and Reactor

The apparatus uses several independently monitored heating sections around the reactor path. Multiple zones allow the inlet, sample region, and outlet path to be conditioned instead of assuming one heater produces a uniform temperature everywhere.

The operator panel exposes the heater controllers and temperature readouts. For defensible test work, those readings would need to be checked against calibrated sensors at defined locations, and the sample temperature would need to be related to the displayed controller values.

### Gas Analysis

A Testo 350K analyzer was connected to the setup for flue-gas measurement. The intended method compares gas conditions before and after sample exposure while retaining the flow, temperature, and timing context for each reading.

An analyzer does not make the overall result valid on its own. Sampling-line temperature, condensation, response delay, zero and span checks, cross-sensitivity, and measurement uncertainty can all affect the result. Those factors belong in the test record for each run.

## Engineering Decisions

**Integrate controls at the apparatus level.** Keeping flow and temperature status visible on the front panel made setup and troubleshooting more direct.

**Use multiple thermal zones.** Separate zones provide a way to manage temperature along the gas path and investigate gradients rather than relying on a single furnace setpoint.

**Design around a documented sequence.** Leak checks, purge order, stabilization, sample exposure, measurement, and shutdown are part of the experiment, not administrative details after the hardware is built.

**Separate methodology references from compliance.** Standards and industry guidance can inform a design. Compliance requires a traceable clause-by-clause assessment, calibration evidence, and documented execution, none of which is claimed here.

## Evidence and Outcome

The retained photograph shows a completed, integrated apparatus with labeled gas controls, heater controllers, a reactor/heating section, and the analyzer installed alongside it. The project archive also contains the intended operating and analysis workflow. This evidence supports a completed bench-scale hardware build.

The site does not contain raw analyzer exports, calibration certificates, sample chain of custody, an uncertainty calculation, repeated runs, or an independent comparison. No quantitative catalyst or apparatus performance result is therefore reported.

The defensible outcome is that the gas, thermal, reactor, control, and measurement subsystems were physically integrated into a laboratory apparatus suitable for further commissioning and documented testing.

## Limitations and Next Validation Steps

- Formal VGB-R 302 or EPRI compliance is not claimed.
- Calibration certificates and instrument traceability are not included in the archive.
- No raw run data, blank tests, repeatability series, or uncertainty budget is available here.
- Gas-mixing uniformity, residence time, thermal gradients, and sampling delay remain to be quantified.
- Catalyst composition, condition, preparation, and chain of custody would need to be recorded for each result.
- Gas safety, ventilation, interlocks, leak response, and operating procedures require site-specific professional approval.

A credible commissioning package would include a piping and instrumentation diagram, leak-test records, calibrated flow and temperature checks, analyzer zero/span records, blank runs, repeated reference-sample tests, and versioned operating procedures. Results could then be reported with their test conditions and uncertainty.

## What I Learned

This apparatus made clear that environmental test engineering depends on the whole measurement chain. Gas preparation, temperature control, reactor geometry, sampling, analyzer behavior, and shutdown discipline all affect whether a number is meaningful. The project also reinforced a documentation principle: a completed machine is strong evidence of engineering work, while a performance claim still requires traceable test data.
