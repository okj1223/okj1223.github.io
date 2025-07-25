---
layout: project
title: "Automated Catalyst Cleaning Robot"
date:   2023-07-10
description: "Precision SCR catalyst maintenance using ROS2, 3D laser scanning & high‑pressure micro‑nozzles."
video_id: "영상ID1"
permalink: /projects/automated-catalyst-cleaning/
---

## Overview

At Geesco’s R&D center, I designed and built an autonomous robot for precision cleaning of SCR catalysts.  
By integrating a **3 mm micro‑nozzle manifold** with a **real‑time 3D laser scanner**, the system maps catalyst surfaces and selectively removes particulate buildup—improving emissions control efficiency.

Control logic was implemented in ROS2 (Galactic) with Python nodes orchestrating navigation, scanning, and cleaning sequences.  
The platform navigates within a confined test cell, avoids obstacles, and manages nozzle pressure dynamically.

## Key Features

- High‑precision micro‑nozzle array (⌀3 mm) for localized cleaning  
- 3D laser scanning for real‑time surface topology mapping  
- ROS2‑based task orchestration with Python scripting  
- Autonomous path planning within complex catalyst geometries  
- Dynamic pressure control to prevent over‑etching  

## Technology Stack

- **ROS2 Galactic** & Python 3.9  
- 3D LiDAR Scanner (Ouster OS1‑16)  
- High‑pressure pump & custom nozzle manifold  
- Ubuntu 20.04 LTS, Docker container for deployment  
- RViz & MoveIt! for simulation and calibration  