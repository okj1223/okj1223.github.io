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
quick_summary_note: "Batch sequencing, sensor interfaces, temperature control, equipment integration, and physical mixing."
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
  <figcaption>Physical mixing trial in the project vessel.</figcaption>
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

## Batch Sequence and State Transitions

The central control artifact was the batch sequence. The documented sequence includes initialization, filling, heating, mixing, cooling, transfer, completion, and fault handling. Each state combines an equipment command with a condition for leaving that state.

<div class="project-owned-pipeline" markdown="0">
  <div class="project-flow" role="list">
    <div role="listitem"><strong>Prepare</strong><small>Check the selected batch and sensor state.</small></div>
    <div role="listitem"><strong>Fill</strong><small>Observe level and confirm the intended quantity.</small></div>
    <div role="listitem"><strong>Condition and mix</strong><small>Control temperature and execute the mixing interval.</small></div>
    <div role="listitem"><strong>Transfer</strong><small>Route the batch and monitor completion.</small></div>
    <div role="listitem"><strong>Complete or fault</strong><small>Record the outcome and require an explicit next action.</small></div>
  </div>
</div>

The transition condition matters as much as the state name. “Heater on for a fixed duration” and “temperature reached and held” describe different processes. Likewise, an elapsed transfer time does not necessarily mean the vessel emptied. Measurements, timeouts, and equipment feedback provide different evidence and should not be silently substituted for one another.

| Phase | Main feedback | Control-design question |
| --- | --- | --- |
| Filling | Level or accumulated flow | Has the intended amount actually entered? |
| Heating | Temperature and sensor validity | Is the process approaching its target without an invalid reading? |
| Mixing | Elapsed mixing interval and equipment state | Did mixing occur under the intended process conditions? |
| Transfer | Level, flow, and pump state | Is material leaving through the selected route? |
| Fault handling | Cause and latched state | What must be checked before a controlled restart? |

## Sensor Interfaces and Signal Conditioning

The documented sensor plan combined temperature, level, pressure, and flow measurements. These channels have different interfaces and failure modes.

Temperature measurement was described around a PT100 interface. Level measurement required a conversion from sensor distance into vessel fill height and then volume. For a vessel with a conical lower section, a single constant “litres per centimetre” conversion is inappropriate across the entire height.

The pressure channel required the correct sensor transfer function and pressure convention. Flow pulses required a calibration factor and counting interval. These conversions belong close to acquisition so the higher-level controller receives quantities with declared units.

The integration work also considered filtering, range checks, missing readings, and wiring around switching equipment. A filtered value can still be invalid. The controller must retain both the measurement and its status rather than replacing every questionable reading with a plausible constant.

## Temperature Control and Saturation

The design used feedback control for temperature, with a heater command limited by the available actuator. In parallel PID notation:

`u = Kp e + Ki ∫e dt + Kd de/dt`

The process responds slowly compared with sensor sampling, and thermal delay can cause the integral term to keep accumulating before the temperature visibly changes. Anti-windup handles this mismatch when the heater command is saturated. It is also necessary to define what happens to controller state when a batch changes phase or a sensor becomes invalid.

The control design considered step-response modeling and PID tuning. The meaningful engineering sequence is to identify the measured response, choose an initial controller, and inspect overshoot, settling, and actuator saturation during the relevant process state. A set of coefficients without their units, sample interval, and actuator convention is not a reproducible controller.

A supervisory temperature command and a power-control stage also need a clear boundary. Calling a system “cascade control” requires a genuine inner feedback loop; a PWM output alone is an actuator command, not evidence of a separately measured power loop.

## Arduino, Raspberry Pi, and Message Boundaries

The layered architecture assigned immediate sensor and equipment handling to the Arduino, while the Raspberry Pi handled supervision, logs, and the user interface. This separated the process sequence from network or browser delays.

The documented serial protocol carried status and commands between the two controllers. Message framing, length, type, and an integrity check help detect incomplete or corrupted frames. The receiving side also needs a timeout and a defined response to stale commands.

Supervisory records should connect a batch identifier with state transitions, setpoints, measurements, and faults. That makes it possible to answer whether a temperature excursion happened during heating, after transfer began, or while communication was interrupted.

The MQTT and web-interface discussion extended the monitoring architecture. Remote visibility can improve operation, but it should not make a network connection a prerequisite for the local controller to recognize an invalid sensor or stop an active sequence.

## Transfer Hydraulics, Piping, and Electrical Integration

The pump-selection work began with the intended batch volume and transfer time: `Q_required = V_batch / t_transfer`. The required head then includes elevation, line losses, fittings, and the receiving condition. The operating point is where the selected pump characteristic meets the system requirement.

That process is more useful than repeating a catalog maximum flow and maximum head as though both occur simultaneously. Wetted-material compatibility, liquid temperature, suction conditions, and service access also affect selection. The old page's exact pump identity and ratings are not established by the retained vessel photograph.

The electrical design discussion covered actuator relays, power supplies, sensor interfaces, grounding, and the separation of process control from protective functions. The piping and instrumentation view connected each valve and sensor to its role in filling, mixing, draining, or transferring.

## Validation as a Sequence of Observable Checks

The engineering validation plan moved from individual channels to complete batches: verify sensor conversion, check actuator correspondence, exercise one state transition at a time, and then observe the combined sequence. Invalid readings, interrupted communication, and an incomplete transition belong in that plan alongside a nominal batch.

For process performance, the important outputs are final batch quantity, temperature history, phase duration, and repeatability under declared starting conditions. These quantities connect the control design to a repeatable commissioning plan. Capability, availability, and energy improvements need measured batch histories before they can be reported as results.


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
