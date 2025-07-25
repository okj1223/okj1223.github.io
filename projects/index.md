---
layout: page
title: "Projects"
permalink: /projects/
---

<section class="projects-index">
  <div class="container">
    <h1 class="page-title">🛠️ All Projects</h1>
    <ul class="projects-list">
      {% assign project_dirs = site.pages | where_exp: "p", "p.url contains '/projects/'" | where_exp: "p", "p.url != '/projects/'" | sort: "date" | reverse %}
      {% for p in project_dirs %}
        <li><a href="{{ p.url | relative_url }}">{{ p.title }}</a></li>
      {% endfor %}
    </ul>
  </div>
</section>

