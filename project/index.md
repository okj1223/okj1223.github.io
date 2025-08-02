---
layout: page
title: "Projects"
permalink: /projects/
---

<section class="project-showcase-section">
  <h2 class="project-title">🛠️ All Projects</h2>
  <div class="project-showcase-container">
    <div class="project-showcase-grid">

      <!-- Industrial Safety Robot System -->
      <a href="/projects/crack-ppe-detection/" class="project-link">
        <div class="project-item">
          <div class="project-description">
            <h3>Industrial Safety Robot System</h3>
            <p>In the intersection of mathematics and machinery, we find not just efficiency, but the possibility of preserving human life itself.</p>
          </div>
          <div class="project-video">
            <div class="video-wrapper">
              <iframe 
                src="https://www.youtube.com/embed/gsk4GZmsjrw"
                title="Industrial Safety Robot System Demo"
                frameborder="0"
                allowfullscreen>
              </iframe>
            </div>
          </div>
        </div>
      </a>

      <!-- Automated Catalyst Cleaning Robot -->
      <a href="/projects/automated-catalyst-cleaning/" class="project-link">
        <div class="project-item">
          <div class="project-description">
            <h3>Automated Catalyst Cleaning Robot</h3>
            <p>Designed and fabricated an in-situ SCR catalyst cleaning system using Arduino-based electrical control, fluid dynamic nozzles, 3D modeling, and welded structural assembly.</p>
          </div>
          <div class="project-video">
            <div class="video-wrapper">
              <iframe 
                src="https://www.youtube.com/embed/7pVAK6bW15U"
                title="Automated Catalyst Cleaning Robot Demo"
                frameborder="0"
                allowfullscreen>
              </iframe>
            </div>
          </div>
        </div>
      </a>

    </div>
  </div>
</section>

<style>
/* 프로젝트 쇼케이스 스타일 */
.project-showcase-section {
  max-width: 1200px;
  margin: 0 auto;
  padding: 2rem 1rem;
}

.project-title {
  text-align: center;
  font-size: 2.5rem;
  margin-bottom: 3rem;
  color: #2c3e50;
  font-weight: 700;
}

.project-showcase-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(500px, 1fr));
  gap: 2rem;
  margin-bottom: 2rem;
}

.project-link {
  text-decoration: none;
  color: inherit;
  display: block;
  height: 100%;
}

.project-item {
  background: #ffffff;
  border-radius: 15px;
  overflow: hidden;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.project-item:hover {
  transform: translateY(-10px);
  box-shadow: 0 20px 40px rgba(0, 0, 0, 0.15);
}

.project-description {
  padding: 1.5rem;
  flex-grow: 1;
}

.project-description h3 {
  font-size: 1.5rem;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 1rem;
  line-height: 1.3;
}

.project-description p {
  color: #5d737e;
  line-height: 1.6;
  font-size: 0.95rem;
}

.project-video {
  padding: 0 1.5rem 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%; /* 16:9 aspect ratio */
  height: 0;
  overflow: hidden;
  border-radius: 8px;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  border: none;
}

/* 반응형 디자인 */
@media (max-width: 768px) {
  .project-showcase-grid {
    grid-template-columns: 1fr;
    gap: 1.5rem;
  }
  
  .project-title {
    font-size: 2rem;
    margin-bottom: 2rem;
  }
  
  .project-description {
    padding: 1.25rem;
  }
  
  .project-description h3 {
    font-size: 1.3rem;
  }
  
  .project-video {
    padding: 0 1.25rem 1.25rem;
  }
}

@media (max-width: 480px) {
  .project-showcase-section {
    padding: 1rem 0.5rem;
  }
  
  .project-title {
    font-size: 1.8rem;
  }
  
  .project-description {
    padding: 1rem;
  }
  
  .project-video {
    padding: 0 1rem 1rem;
  }
}

/* 로딩 애니메이션 */
.project-item {
  animation: fadeInUp 0.6s ease-out;
}

.project-item:nth-child(2) {
  animation-delay: 0.2s;
}

@keyframes fadeInUp {
  from {
    opacity: 0;
    transform: translateY(30px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* 접근성 개선 */
.project-link:focus {
  outline: 2px solid #007bff;
  outline-offset: 2px;
}

.project-link:focus .project-item {
  transform: translateY(-5px);
}
</style>