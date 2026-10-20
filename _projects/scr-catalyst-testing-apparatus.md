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

My contribution focused on apparatus-level engineering: translating the test concept into an integrated arrangement of gas handling, heating, sample exposure, controls, and measurement. The case study covers that integration and the shared hardware build.

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

## Integrating Gas Delivery, Reactor, and Measurement

The apparatus was designed around a controlled comparison: expose a catalyst sample to a defined gas stream and thermal condition, then measure how the outlet differs from the inlet. Gas delivery, sample sealing, temperature, and analyzer response all contribute to the result.

The apparatus design connects separate gas channels, flow control, a mixing section, the heated sample region, and downstream analysis. Each channel has a chemical role: a carrier stream, oxygen, the pollutant species, and the reducing-agent supply. A channel's flow rate alone does not specify the final concentration.

For a diluted cylinder mixture, the inlet species fraction depends on both the cylinder concentration and its contribution to total flow:

`c_in = Σ(c_i Q_i) / ΣQ_i`

All flows must use a consistent reference basis. Treating a listed NO-channel flow as pure NO while reporting a few hundred ppm downstream would create a serious inconsistency. The cylinder mixture and calibration are therefore part of the experimental specification.

## Sample Mounting and Thermal Zones

The reactor had to direct the gas through the catalyst channels. Leakage around the sample would let part of the flow bypass the active material, changing the apparent conversion. Sample dimensions, mounting, sealing, and differential-pressure connections were consequently part of the measurement design.

The thermal architecture used separate heating regions to manage inlet conditioning and the sample region. Independent temperature readings help reveal an axial gradient that one controller display could conceal. Heater output, reactor-wall temperature, gas temperature, and catalyst temperature are related but not interchangeable measurements.

| Subsystem | What it controls or observes | Why it affects the result |
| --- | --- | --- |
| Gas channels and flow control | Inlet composition and throughput | Changes reaction conditions and contact time |
| Sample mounting | Intended flow through the catalyst | Bypass can distort apparent activity |
| Heating zones | Thermal condition along the flow path | Temperature variation changes conversion |
| Pressure measurements | Flow resistance across the sample/path | Helps identify blockage or changed flow conditions |
| Sampling and analyzer | Outlet composition over time | Delay and conditioning affect the reported signal |

This integration work was the substance of the apparatus project: arranging the instruments and process hardware so an analyzer value could be interpreted as a result under known conditions.

## Experimental Sequence and Data Record

The documented test workflow included sample identification, apparatus checks, thermal stabilization, an inlet baseline, a reaction interval, and a controlled end to the test. The engineering purpose of these stages was to keep transient startup behavior separate from the comparison interval.

The data record needs the sample identity and dimensions, gas-mixture information, flow settings and measured values, temperature channels, pressure observations, analyzer state, and timestamps. A steady-looking outlet trace is not enough if the inlet concentration or total flow changed during the same period.

Analyzer zero/span checks and the sampling path also matter. A delay between the reactor and analyzer shifts the apparent response in time. Condensation or adsorption in a sample line can change the composition that reaches the instrument. These effects belong to the measurement system, rather than being attributed automatically to the catalyst.

The project is presented as apparatus engineering and experimental design. The public page is not an operating procedure for toxic gases; exact gas-handling settings and protective arrangements belong to the approved laboratory procedure.

## Interpreting Catalyst Performance

For inlet and outlet NOx concentrations on the same basis, removal efficiency is:

`η_NOx = (C_in − C_out) / C_in`

The comparison needs a declared NOx definition, consistent wet/dry basis, and any oxygen-reference correction applied consistently. A low outlet reading can reflect effective conversion, a changed inlet, dilution, or a measurement problem. The apparatus record should make those alternatives distinguishable.

Flow and catalyst geometry determine exposure. A common volumetric comparison is gas hourly space velocity, `GHSV = Q / V_catalyst`, with an explicitly declared flow reference and catalyst-volume definition. An apparent first-order index can be written `k_app = −ln(C_out/C_in) × GHSV` under its model assumptions.

An area-normalized activity index uses its own declared area convention. Catalyst face area and internal active surface area are different quantities; switching between them changes the meaning of the result. Such an index is not automatically an intrinsic material constant independent of transport and operating conditions.

The earlier documentation also considered ammonia slip and sulfur conversion. Those quantities require suitable species measurements or an explicitly qualified balance. A decrease in NOx does not directly measure ammonia slip, and a small SO₂ difference alone does not establish SO₃ production when measurement uncertainty or other losses are comparable.

## Repeatability and Uncertainty

A repeatability study would hold the declared sample and conditions fixed across runs, then compare the selected steady-state intervals. Reproducibility requires stating what changed: operator, day, calibration, sample installation, or instrument.

For the simple efficiency relation and independent inlet/outlet uncertainties, first-order propagation gives:

`u_η² ≈ (C_out u_in / C_in²)² + (u_out / C_in)²`

Correlated measurements require covariance terms. Near a high removal fraction, the small outlet value can be particularly sensitive to the analyzer's detection range and uncertainty. Reporting many decimal places does not improve the underlying measurement.

The completed apparatus and its design document the integrated build and intended test methodology. A quantitative catalyst comparison additionally needs the original calibrated run data and uncertainty analysis.


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
