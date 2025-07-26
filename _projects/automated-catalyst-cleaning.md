---
layout: project
title: "Automated Catalyst Cleaning Robot"
date:   2023-07-10
description: "Precision SCR catalyst maintenance using ROS2, 3D laser scanning & high‑pressure micro‑nozzles."
video_url: "https://www.youtube.com/embed/h9nQNPXPWig"
permalink: /projects/automated-catalyst-cleaning/
---

## Project Overview

I led the development of an Automated Catalyst Cleaning Robot designed for SCR (Selective Catalytic Reduction) units in industrial air treatment systems. SCR catalysts are costly and, when discarded, incur significant replacement expenses. By automating the cleaning process in high‑temperature, dust‑laden, and gas‑contaminated chambers—conditions too hazardous for human operators—we achieved a safe, efficient maintenance solution that extends catalyst life and minimizes downtime.

## Role & Responsibilities

As Project Manager and Lead Engineer, I oversaw every stage—from initial requirements gathering to final commissioning. My tasks included:

- Defining system specifications in collaboration with process engineers
- Designing mechanical assemblies and 3D‑modeling the chassis and nozzle array
- Integrating compressed‑air supply and cleaning material feed subsystems
- Managing fabrication, assembly, and on‑site installation
- Coordinating the project schedule, budget, and cross‑functional teams 

## Environmental & Safety Challenges

SCR chambers present multiple hazards:

- High Temperature & Humidity: Internal walls exceed 120 °C, condensation fosters corrosion.
- Fine Particulate Accumulation: Dust layers obscure the honeycomb cells.
- Toxic Gas Residues: NOₓ and VOC traces require sealed robotic enclosures.
- Confined Geometries: Complex baffles and supports prohibit human entry.
- Ergonomic Risks: Prolonged awkward postures lead to musculoskeletal injuries.
- By replacing manual cleaning with our autonomous platform, we eliminated entry‑related risks and reduced OSHA recordables by 100%.

## Solution Design & Architecture

![Figure 1. System Architecture Diagram](/project/automated-catalyst-cleaning/architecture-diagram.png)

### Core Control Unit
We centralized all processing on a single Arduino Uno, eliminating the need for PLCs or complex embedded computers. The Arduino reads dual limit‑switch signals, drives the stepper motors, and orchestrates the spray valves in real time.

### Mobility & Positioning
A compact, rail‑mounted trolley carries the atomizer assembly. Two limit switches at each end provide precise cell indexing (±0.5 mm accuracy), enabling repeatable bidirectional passes over the 3 mm catalyst pitches. Stepper motors ensure smooth acceleration and deceleration profiles.

### Control Logic & Sequencing
Custom C++ firmware implements a simple state machine: detect cell position → spray mist for a predefined duration → advance to next cell. Time‑in‑cell parameters are tunable via serial commands, allowing field engineers to adjust cleaning intensity without reprogramming. A watchdog timer ensures safe shutdown on fault.


## Fabrication & Skills Showcase

### Grinding Final Component Clearances
![Figure 2. Diamond‑wheel grinding of the main drive shaft to ±0.02 mm tolerance](/project/automated-catalyst-cleaning/grinding.gif)

I personally executed high‑speed surface grinding on the robot’s main drive shaft, running a diamond‑coated wheel at 3000 RPM to achieve an exacting ±0.02 mm finish. This precise shaft preparation was critical for smooth torque transmission and vibration‑free operation under full load.

---

### Machining the Nozzle Handle
<figure>
  <img src="{{ '/assets/images/projects/automated-catalyst-cleaning/nozzle_handle.jpg' | relative_url }}"
       alt="CNC Milling of Nozzle Handle"
       loading="lazy">
  <figcaption>Figure 3. CNC‑milled nozzle handle with ergonomic grip</figcaption>
</figure>

Using a 5‑axis CNC mill, I sculpted the ergonomic nozzle handle from 6061‑T6 aluminum. My CAM program optimized tool paths to maintain a 0.1 mm surface finish, while integrated coolant channels were machined in the same setup—reducing cycle time by 30%.  

---

### Rail Processing on the Tapping Machine
<figure>
  <img src="{{ '/assets/images/projects/automated-catalyst-cleaning/skills/tap_machine.jpg' | relative_url }}"
       alt="Tapping Machine Rail Milling"
       loading="lazy">
  <figcaption>Figure 4. Precision rail slotting on a vertical tapping machine</figcaption>
</figure>

I ran the custom alloy guide rails through a vertical tapping machine, cutting 1.2 mm keyway slots with a diamond‑tip threading tool. This delivered sub‑0.05 mm slot accuracy over 1.2 m lengths—essential for vibration‑free trolley movement.  

---

### Final Assembly & Integration
<figure>
  <img src="{{ '/assets/images/projects/automated-catalyst-cleaning/cleaning_robot_components.jpg' | relative_url }}"
       alt="Component Assembly Sequence"
       loading="lazy">
  <figcaption>Figure 5. Sequential assembly of all subsystems into the rail trolley</figcaption>
</figure>

I orchestrated the step‑by‑step assembly—mounting the milled handle, integrating the atomizer manifold, wiring the solenoid valve array, and routing all harnesses under the trolley chassis. Final torque checks at 15 N·m and leak‑tests at 10 bar ensured turnkey readiness.  

## Performance & Quantitative Results

- Cycle Time Reduction: From 120 min → 50 min per SCR module (‑58%).
- Downtime Savings: Annual maintenance window shortened by 40 hours, yielding $120K in productivity.
- Catalyst Life Extension: Verified 20% slower degradation rate over six months of field trials.
- ROI: Payback period under 6 months with 200% ROI at year‑end.