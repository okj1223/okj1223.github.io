---
layout: page
title: "Projects"
permalink: /projects/
---

<section class="project-showcase-section">
  <h2 class="project-title">🛠️ All Projects</h2>
  <div class="project-showcase-container">
    <div class="project-showcase-grid">
      {%- assign project_dirs = site.projects
        | sort: "date"
        | reverse -%}

      {%- for p in project_dirs -%}
      <a href="{{ p.url | relative_url }}" class="project-link">
        <div class="project-item">
          <div class="project-description">
            <h3>{{ p.title }}</h3>
            {% if p.description %}
              <p>{{ p.description }}</p>
            {% endif %}
          </div>

          {% if p.video_url %}
          <div class="project-video">
            <div class="video-wrapper">
              <iframe 
                src="{{ p.video_url }}"
                title="{{ p.title }} Demo"
                frameborder="0"
                allowfullscreen>
              </iframe>
            </div>
          </div>
          {% endif %}
        </div>
      </a>
      {%- endfor -%}
  </div>
</section>
