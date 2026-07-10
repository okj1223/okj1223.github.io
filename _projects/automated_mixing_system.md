---
layout: project
title: "Automated Mixing System for Industrial Chemical Processing"
permalink: /projects/automated-mixing-system/
date: 2024-10-02
description: "A prototype-focused industrial mixing project that combines embedded control, process sensing, batch sequencing, and supervisory monitoring."
card_description: "A prototype-focused mixing system built around process sensing, explicit batch states, embedded control, and supervisory monitoring."
share-description: "A prototype-focused industrial mixing project covering embedded control, process sensing, batch sequencing, and supervisory monitoring."
subtitle: "Turning a manual mixing workflow into a measurable, state-based control system."
thumbnail-img: /project/automated-mixing-system/automated-mixing-system.png
card_media_fit: cover
filter_categories:
  - control
  - mechanical
category_label: "Industrial Automation - Process Control"
impacts:
  - value: "Layered"
    label: "Control Design"
  - value: "Closed-loop"
    label: "Process Feedback"
  - value: "State-based"
    label: "Batch Sequence"
tech_tags:
  - label: "Arduino"
    style: "prog"
  - label: "Raspberry Pi"
    style: "prog"
  - label: "PID Control"
    style: "eng"
  - label: "Python"
    style: "prog"
  - label: "C++"
    style: "prog"
  - label: "MQTT"
    style: "comm"
  - label: "Process Sensing"
    style: "sensor"
quick_summary_note: "A concise case study of the control architecture and physical mixing prototype, scoped to demonstrated integration rather than production KPIs."
quick_summary:
  - label: "Context"
    value: "A manual industrial mixing sequence that depended on operator timing and observation"
  - label: "My Role"
    value: "Workflow analysis, control architecture, I/O planning, and prototype integration"
  - label: "System"
    value: "Embedded control, supervisory computing, sensors, valves, mixer, pump, and logging"
  - label: "Evidence"
    value: "Physical mixing hardware and an end-to-end system design"
  - label: "Status"
    value: "Prototype work; production performance and certification are not claimed"
---

## Context

The starting point was a manual batch-mixing workflow. An operator had to watch temperature and liquid level, decide when to move to the next step, run the mixer for the intended interval, and then manage transfer out of the vessel. That approach can work, but process quality depends heavily on attention, timing, and consistent execution.

The engineering goal was to convert that sequence into an explicit control problem:

- measure the process instead of relying only on visual judgment,
- define clear transitions between batch stages,
- separate direct equipment control from monitoring and data handling,
- and make abnormal conditions visible before continuing the sequence.

This page presents the resulting prototype and system design. It does not present the work as a certified production installation.

<figure markdown="0">
  <img class="project-image"
       src="{{ '/project/automated-mixing-system/automated-mixing-system.png' | relative_url }}"
       alt="Liquid being agitated in the physical mixing vessel"
       loading="lazy">
  <figcaption>Physical mixing trial in the project vessel. This is the surviving public hardware evidence for the project.</figcaption>
</figure>

## My Role

I worked across the process and control boundary. My contribution covered:

- breaking the manual workflow into discrete operating states,
- mapping the measurements and actuators required at each state,
- defining a layered Arduino and Raspberry Pi control architecture,
- planning temperature, level, flow, and pressure feedback,
- structuring heater, valve, mixer, and transfer-pump commands,
- and documenting commissioning and fault-handling requirements.

The useful part of this work was not a single controller or algorithm. It was translating an operator procedure into a system whose behavior could be inspected, tested, and revised.

## System

The design separates responsibilities between two computing layers.

The embedded controller handles equipment-facing work: reading process sensors, applying local control logic, sequencing relays and valves, and moving the system to a safe state when required. The supervisory computer handles slower, less deterministic work such as operator status, batch records, trends, and network communication.

The intended signal path is:

1. temperature, level, flow, and pressure sensors describe the current process state;
2. the embedded controller validates those inputs and applies local control;
3. heater, inlet valve, mixer, and transfer-pump commands advance the batch;
4. the supervisory layer records state and exposes operating information;
5. alarms or invalid conditions stop progression and require review.

The batch itself is represented as an explicit state sequence: idle, preparation or heating, filling, operator-confirmed material addition, mixing, transfer, and completion. A state transition occurs only when its entry conditions are met. This keeps timing logic, sensor checks, and operator actions from becoming an unstructured collection of relay commands.

## Key Decisions

### Separate direct control from supervision

Equipment I/O and local fault response should not depend on a web interface or network connection. Keeping those functions on the embedded controller reduces coupling, while the Raspberry Pi remains available for logging, visualization, and remote status.

### Use feedback for process completion

Elapsed time alone is a weak proxy for temperature, fill level, or completed transfer. The design therefore treats measured process conditions as transition inputs. Timeouts remain useful, but primarily as fault detectors when an expected condition is not reached.

### Make the sequence observable

Each batch state has a defined purpose, expected inputs, allowed outputs, and exit condition. This makes troubleshooting more direct: an operator can identify where the process stopped and which condition prevented the next transition.

### Keep safety functions independent

Software limits can improve operation, but they are not a substitute for correctly rated protection. Emergency stopping, electrical protection, actuator defaults, and any safety-critical interlocks must be implemented with hardware appropriate to the actual industrial risk assessment.

## Evidence and Outcome

The project produced a physical mixing setup and an end-to-end automation design linking process measurements, state sequencing, equipment commands, and supervisory logging. The available image confirms operation of the mixer in a liquid-filled vessel. The design record also establishes a coherent division of responsibility between embedded and supervisory control.

That is the outcome claimed here. Cycle time, accuracy, availability, energy use, and process capability are not reported because the public project record does not include the raw test logs, calibration records, or production history required to substantiate them.

## Limitations

The public evidence is narrower than the original technical write-up. It does not currently include a complete wiring package, reproducible firmware repository, sensor calibration dataset, repeated batch results, or third-party safety review. The project should therefore be read as prototype and systems-design work, not proof of production readiness.

Before deployment, the next engineering steps would be:

- verify sensor selection and chemical compatibility against the real process,
- validate control behavior with traceable calibration and repeated batch data,
- complete electrical and process hazard reviews,
- use appropriately rated industrial and safety hardware,
- test loss of power, sensor faults, stuck actuators, and communication loss,
- and document maintenance, manual recovery, and operator procedures.

The central lesson was that automation begins with a precise description of the process. Once states, measurements, transitions, and failure responses are explicit, hardware and software decisions become much easier to evaluate.
