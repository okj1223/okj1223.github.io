---
layout: project
title: "Autonomous Detection Arm Robot"
permalink: /projects/autonomous-detection-arm/
date: 2025-08-12
description: "Precision SCR catalyst maintenance using ROS2, 3D laser scanning & high‑pressure micro‑nozzles."
video_url: "https://www.youtube.com/embed/영상ID2"
---



<!-- Hero -->
<section class="project-hero">
  <div class="container">
    <h1 class="project-title">Autonomous Detection Arm Robot</h1>
    <p class="project-subtitle">
      AI‑powered 6‑DOF robotic arm for automated inspection, defect classification, and handling in industrial environments
    </p>
  </div>
</section>

<!-- Overview -->
<section class="project-overview">
  <div class="container two-column">
    <div class="column description">
      <h2>Overview</h2>
      <p>
        This project integrates a six‑degree‑of‑freedom robotic manipulator with advanced computer vision to autonomously inspect components on a production line.
        Using YOLOv8 for real‑time object detection and classification, the system identifies defects, categorizes parts, and executes pick‑and‑place operations.
      </p>
      <p>
        The entire solution is orchestrated in ROS2 (Humble) with Python nodes handling perception, motion planning via MoveIt!, and safety‑critical interlocks.
        Collision avoidance and dynamic path re‑planning ensure safe and reliable operation in cluttered industrial cells.
      </p>
    </div>
    <div class="column media">
      <h2>Demo Video</h2>
      <div class="video-wrapper">
        <iframe
          src="https://www.youtube.com/embed/영상ID4"
          title="Autonomous Detection Arm Robot Demo"
          frameborder="0"
          allowfullscreen>
        </iframe>
      </div>
    </div>
  </div>
</section>

<!-- Key Features -->
<section class="project-features">
  <div class="container">
    <h2>Key Features</h2>
    <ul class="features-list">
      <li>YOLOv8‑based defect detection and classification in under 50 ms per frame</li>
      <li>6‑DOF UR5 robotic arm integration with real‑time ROS2 control</li>
      <li>Automated pick‑and‑place sequencing driven by classification output</li>
      <li>Dynamic collision avoidance and safety interlock mechanisms</li>
      <li>MoveIt! motion planning with custom grasping strategies</li>
    </ul>
  </div>
</section>

<!-- Technology Stack -->
<section class="project-techstack">
  <div class="container">
    <h2>Technology Stack</h2>
    <ul class="tech-list">
      <li>ROS2 Humble & Python 3.10</li>
      <li>YOLOv8 & OpenCV for computer vision</li>
      <li>6‑DOF UR5 Robotic Arm</li>
      <li>MoveIt! & RViz for path planning and simulation</li>
      <li>Ubuntu 22.04 LTS, Docker containers for deployment</li>
    </ul>
  </div>
</section>

<!-- Back to Projects -->
<section class="project-back">
  <div class="container">
    <a href="/projects/" class="button">&larr; Back to Projects</a>
  </div>
</section>
