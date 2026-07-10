# Kyungjun Oh Robotics Portfolio

Jekyll portfolio for robotics systems engineering work across robot learning,
autonomous platforms, industrial automation, and field deployment.

## Local Development

```bash
bundle install
bundle exec jekyll serve
```

The site is available at `http://127.0.0.1:4000/` by default.

## Content Model

- `_data/home.yml`: homepage focus areas, skills, credentials, and career
  history.
- `_projects/*.md`: project case studies rendered by `_layouts/project.html`.
  Set `featured: true` and a unique `featured_order` from 1 to 3 to surface a
  project on the homepage.
- `side-builds/*.md`: smaller personal tools and experiments.
- `_includes/project-card.html` and `assets/css/project-card.css`: the shared
  project-card component used by both the homepage and Projects listing.
- `assets/data/searchcorpus.json`: Liquid-generated search index for projects
  and public pages.

Project pages should distinguish measured evidence from design targets,
simulations, prototypes, and future work. Company work must also state the
author's direct role and avoid disclosing internal implementation details.

## Verification

```bash
ruby scripts/validate_project_cards.rb
bundle exec jekyll build
bundle exec jekyll doctor
```

Before publishing, verify desktop and mobile layouts, internal links, project
media, search results, and external demo links.
