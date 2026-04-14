document.addEventListener('DOMContentLoaded', function () {
  var hasEcoPanels = document.querySelector('.eco-system-left, .eco-system-right, .progress-node, .env-monitor-hud');

  if (!hasEcoPanels) {
    return;
  }

  var progressLine = document.getElementById('scroll-progress-line');
  var progressNodes = Array.from(document.querySelectorAll('.progress-node'));
  var thresholdMap = {
    banner: 5,
    projects: 20,
    skills: 50,
    career: 80
  };
  var sectionSelectors = {
    banner: ['#banner', '.drone-banner-environment', '.enviro-banner', '.banner-section'],
    projects: ['.home-projects-section'],
    skills: ['.skills-section'],
    career: ['.certifications', '.career-bio-section']
  };
  var scrollTicking = false;
  var hasHud = document.getElementById('co2-val') || document.getElementById('pwr-val') || document.getElementById('eff-val');

  function applyBannerImage() {
    var banner = document.getElementById('banner');
    var bannerImage = banner && banner.dataset ? banner.dataset.bannerImage : null;

    if (banner && bannerImage) {
      banner.style.backgroundImage = "url('" + bannerImage + "')";
    }
  }

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
  }

  function getSectionElement(sectionName) {
    var selectors = sectionSelectors[sectionName] || [];
    var index;
    var element;

    for (index = 0; index < selectors.length; index++) {
      element = document.querySelector(selectors[index]);
      if (element) {
        return element;
      }
    }

    return null;
  }

  function calculateSectionPositions() {
    var positions = {};

    Object.keys(sectionSelectors).forEach(function (sectionName) {
      var element = getSectionElement(sectionName);

      if (element) {
        positions[sectionName] = Math.max(element.offsetTop - 100, 0);
      }
    });

    return positions;
  }

  function updateScrollProgress() {
    var maxScroll = document.documentElement.scrollHeight - window.innerHeight;
    var scrollPercent = maxScroll > 0 ? (window.scrollY / maxScroll) * 100 : 0;

    if (progressLine) {
      progressLine.style.height = Math.min(scrollPercent, 100) + '%';
    }

    progressNodes.forEach(function (node, index) {
      var sectionName = node.dataset.section;
      var threshold = thresholdMap[sectionName];

      if (typeof threshold !== 'number') {
        threshold = (index + 1) * 25;
      }

      node.classList.toggle('active', scrollPercent >= threshold);
    });
  }

  function updateEnvironmentalData() {
    var co2Val = document.getElementById('co2-val');
    var pwrVal = document.getElementById('pwr-val');
    var effVal = document.getElementById('eff-val');
    var co2Bar = document.getElementById('co2-bar');
    var pwrBar = document.getElementById('pwr-bar');
    var effBar = document.getElementById('eff-bar');
    var co2;
    var pwr;
    var eff;

    if (co2Val) {
      co2 = 340 + Math.floor(Math.random() * 15);
      co2Val.textContent = String(co2);
      if (co2Bar) {
        co2Bar.style.width = clamp((co2 / 400) * 100, 0, 100) + '%';
      }
    }

    if (pwrVal) {
      pwr = 85 + Math.floor(Math.random() * 10);
      pwrVal.textContent = String(pwr);
      if (pwrBar) {
        pwrBar.style.width = clamp(pwr, 0, 100) + '%';
      }
    }

    if (effVal) {
      eff = 92 + Math.floor(Math.random() * 6);
      effVal.textContent = String(eff);
      if (effBar) {
        effBar.style.width = clamp(eff, 0, 100) + '%';
      }
    }
  }

  function addTechIconTooltips() {
    document.querySelectorAll('.tech-icon-item').forEach(function (item) {
      var techName = item.getAttribute('data-tech');

      item.style.pointerEvents = 'auto';

      item.addEventListener('mouseenter', function () {
        var tooltip;

        if (!techName || item.querySelector('.tech-tooltip')) {
          return;
        }

        tooltip = document.createElement('div');
        tooltip.className = 'tech-tooltip';
        tooltip.textContent = techName;
        tooltip.style.cssText = [
          'position: absolute',
          'background: rgba(0,0,0,0.9)',
          'color: white',
          'padding: 4px 8px',
          'border-radius: 4px',
          'font-size: 0.7rem',
          'pointer-events: none',
          'z-index: 10000',
          'right: 45px',
          'top: 50%',
          'transform: translateY(-50%)',
          'white-space: nowrap',
          "font-family: 'Courier New', monospace",
          'opacity: 0',
          'transition: opacity 0.3s ease'
        ].join(';');
        item.appendChild(tooltip);

        setTimeout(function () {
          tooltip.style.opacity = '1';
        }, 10);
      });

      item.addEventListener('mouseleave', function () {
        var tooltip = item.querySelector('.tech-tooltip');

        if (!tooltip) {
          return;
        }

        tooltip.style.opacity = '0';
        setTimeout(function () {
          if (tooltip.parentNode) {
            tooltip.parentNode.removeChild(tooltip);
          }
        }, 300);
      });
    });
  }

  function addSmartProgressNodeClicks() {
    progressNodes.forEach(function (node, index) {
      node.style.pointerEvents = 'auto';
      node.addEventListener('click', function () {
        var positions = calculateSectionPositions();
        var sectionName = node.dataset.section;
        var fallbackPosition;

        if (Object.prototype.hasOwnProperty.call(positions, sectionName)) {
          window.scrollTo({ top: positions[sectionName], behavior: 'smooth' });
          return;
        }

        fallbackPosition = index === 0 ? 0 : window.innerHeight * (index * 1.2);
        window.scrollTo({ top: fallbackPosition, behavior: 'smooth' });
      });
    });
  }

  function addFusionInteraction() {
    document.querySelectorAll('.tech-sphere').forEach(function (sphere) {
      sphere.style.pointerEvents = 'auto';

      sphere.addEventListener('mouseenter', function () {
        sphere.style.transform = 'translateX(-50%) scale(1.3)';
        sphere.style.zIndex = '1001';
        sphere.style.transition = 'all 0.3s ease';
      });

      sphere.addEventListener('mouseleave', function () {
        sphere.style.transform = 'translateX(-50%) scale(1)';
        sphere.style.zIndex = 'auto';
      });
    });
  }

  function enhanceDataStream() {
    document.querySelectorAll('.data-packet').forEach(function (packet) {
      var delay = Math.random() * 3;
      var scale = 0.8 + Math.random() * 0.4;

      packet.style.animationDelay = delay + 's';
      packet.style.transform = 'scaleY(' + scale + ')';
    });
  }

  function addGridInteraction() {
    var gridPattern = document.querySelector('.bg-grid-pattern');

    if (!gridPattern) {
      return;
    }

    document.addEventListener('mousemove', function (event) {
      var mouseX = event.clientX / window.innerWidth;
      var mouseY = event.clientY / window.innerHeight;
      var opacity = 0.02 + (mouseX * mouseY * 0.03);

      gridPattern.style.opacity = Math.min(opacity, 0.08);
    });
  }

  function initFadeIn() {
    var leftPanel = document.querySelector('.eco-system-left');
    var rightPanel = document.querySelector('.eco-system-right');

    if (leftPanel) {
      leftPanel.style.opacity = '0';
      leftPanel.style.transform = 'translateY(-50%) translateX(-20px)';
      leftPanel.style.transition = 'all 0.8s ease';

      setTimeout(function () {
        leftPanel.style.opacity = '1';
        leftPanel.style.transform = 'translateY(-50%) translateX(0)';
      }, 300);
    }

    if (rightPanel) {
      rightPanel.style.opacity = '0';
      rightPanel.style.transform = 'translateY(-50%) translateX(20px)';
      rightPanel.style.transition = 'all 0.8s ease';

      setTimeout(function () {
        rightPanel.style.opacity = '1';
        rightPanel.style.transform = 'translateY(-50%) translateX(0)';
      }, 500);
    }
  }

  function requestScrollTick() {
    if (scrollTicking) {
      return;
    }

    scrollTicking = true;
    window.requestAnimationFrame(function () {
      updateScrollProgress();
      scrollTicking = false;
    });
  }

  function safeInit() {
    try {
      applyBannerImage();
      updateScrollProgress();
      updateEnvironmentalData();
      addTechIconTooltips();
      addSmartProgressNodeClicks();
      addFusionInteraction();
      enhanceDataStream();
      addGridInteraction();
      initFadeIn();
    } catch (error) {
      console.warn('Eco panel initialization warning:', error);
    }
  }

  if (progressLine || progressNodes.length > 0) {
    window.addEventListener('scroll', requestScrollTick);
    window.addEventListener('resize', requestScrollTick);
  }

  if (hasHud) {
    window.setInterval(function () {
      try {
        updateEnvironmentalData();
      } catch (error) {
        console.warn('Eco panel HUD update warning:', error);
      }
    }, 4000);
  }

  document.addEventListener('visibilitychange', function () {
    document.querySelectorAll('.orbital-ring, .data-packet').forEach(function (element) {
      element.style.animationPlayState = document.hidden ? 'paused' : 'running';
    });
  });

  safeInit();
});
