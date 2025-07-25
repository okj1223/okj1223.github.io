---
layout: page
title: "Projects"
permalink: /projects/
---

<!-- ── Banner ── -->
<div id="banner" style="background-image: url('/assets/img/banner_projects.jpg')">
  <div class="banner-overlay">
    <h1 class="banner-title">Projects</h1>
    <p class="banner-subtitle">Engineering motion, crafting intelligence</p>
  </div>
</div>

<!-- ── Projects Showcase ── -->
<section class="project-showcase-section">
  <div class="project-showcase-container">
    <h2 class="project-title">🛠️ All Projects</h2>
    <div class="project-showcase-grid">

      {%- comment -%}
      site.pages 중에서 /projects/ 경로를 갖고,
      자체 index.md(/projects/)는 제외하고,
      date 필드가 있는 것만 골라냅니다.
      {%- endcomment -%}
      {%- assign projs = site.pages
           | where_exp: "p","p.url contains '/projects/' and p.url != '/projects/'"
           | where_exp: "p","p.date"
           | sort: "date" | reverse -%}

      {%- for proj in projs -%}
      <a href="{{ proj.url | relative_url }}" class="project-link">
        <div class="project-item">
          <div class="project-description">
            <h3>{{ proj.title }}</h3>
            {%- if proj.date -%}
            <time class="project-date" datetime="{{ proj.date | date: "%Y-%m-%d" }}">
              {{ proj.date | date: "%Y.%m.%d" }}
            </time>
            {%- endif -%}
            {%- if proj.description -%}
            <p>{{ proj.description }}</p>
            {%- endif -%}
          </div>
          {%- if proj.video_id -%}
          <div class="project-video">
            <iframe
              src="https://www.youtube.com/embed/{{ proj.video_id }}"
              title="{{ proj.title }}"
              frameborder="0" allowfullscreen>
            </iframe>
          </div>
          {%- endif -%}
        </div>
      </a>
      {%- endfor -%}

    </div>
  </div>
</section>
