document.addEventListener('DOMContentLoaded', function () {
  var navbar = document.querySelector('.navbar-custom');
  var navLinks = Array.from(document.querySelectorAll('.navbar-nav .nav-link'));
  var brandWordmark = document.querySelector('.navbar-brand-wordmark');
  var searchLink = document.getElementById('nav-search-link');
  var searchIcon = document.getElementById('nav-search-icon');
  var navbarToggler = document.querySelector('.navbar-toggler');
  var navbarCollapse = document.querySelector('.navbar-collapse');
  var backToTopButton = document.getElementById('backToTop');
  var demoModal = document.getElementById('siteVideoModal');
  var demoModalFrame = document.getElementById('siteVideoModalFrame');
  var demoModalTitle = document.getElementById('siteVideoModalTitle');
  var demoModalClose = demoModal ? demoModal.querySelector('.site-video-modal-close') : null;
  var activeDemoTrigger = null;
  var demoModalHiddenSiblings = [];
  var navbarResizeTimeout = null;

  function closeMobileMenu() {
    if (!navbarCollapse || !navbarToggler) {
      return;
    }

    navbarCollapse.classList.remove('show');
    navbarToggler.setAttribute('aria-expanded', 'false');
    document.body.style.overflow = '';
  }

  function setActiveNavLink() {
    var currentPath = window.location.pathname;

    navLinks.forEach(function (link) {
      var linkPath = new URL(link.href).pathname;
      var isActive = linkPath === currentPath ||
        (currentPath === '/' && linkPath === '/') ||
        (currentPath.startsWith('/projects') && linkPath.startsWith('/projects')) ||
        (currentPath.startsWith('/aboutme') && linkPath.startsWith('/aboutme'));

      link.classList.toggle('active', isActive);
      if (isActive) {
        link.setAttribute('aria-current', 'page');
      } else {
        link.removeAttribute('aria-current');
      }
    });
  }

  function smoothScroll(target, duration) {
    var targetElement = document.querySelector(target);
    var startPosition = window.pageYOffset;
    var navbarOffset = navbar ? navbar.offsetHeight + 20 : 20;
    var targetPosition;
    var distance;
    var startTime = null;

    if (!targetElement) {
      return;
    }

    targetPosition = targetElement.offsetTop - navbarOffset;
    distance = targetPosition - startPosition;

    function ease(timeElapsed, start, change, totalDuration) {
      var normalized = timeElapsed / (totalDuration / 2);

      if (normalized < 1) {
        return change / 2 * normalized * normalized + start;
      }

      normalized -= 1;
      return -change / 2 * (normalized * (normalized - 2) - 1) + start;
    }

    function animation(currentTime) {
      var timeElapsed;
      var position;

      if (startTime === null) {
        startTime = currentTime;
      }

      timeElapsed = currentTime - startTime;
      position = ease(timeElapsed, startPosition, distance, duration);
      window.scrollTo(0, position);

      if (timeElapsed < duration) {
        window.requestAnimationFrame(animation);
      }
    }

    window.requestAnimationFrame(animation);
  }

  if (brandWordmark) {
    brandWordmark.addEventListener('mouseenter', function () {
      brandWordmark.style.transform = 'translateY(-1px)';
      brandWordmark.style.transition = 'all 0.24s ease';
      brandWordmark.style.filter = 'drop-shadow(0 10px 22px rgba(22, 16, 12, 0.18))';
    });

    brandWordmark.addEventListener('mouseleave', function () {
      brandWordmark.style.transform = '';
      brandWordmark.style.filter = '';
    });
  }

  if (navbar) {
    setActiveNavLink();
    window.addEventListener('resize', function () {
      window.clearTimeout(navbarResizeTimeout);
      navbarResizeTimeout = window.setTimeout(function () {
        if (window.innerWidth > 1199) {
          closeMobileMenu();
        }
      }, 250);
    });

    if (window.innerWidth > 1199) {
      document.querySelectorAll('.nav-item.dropdown').forEach(function (dropdown) {
        var timeout;

        dropdown.addEventListener('mouseenter', function () {
          var dropdownMenu = dropdown.querySelector('.dropdown-menu');
          var dropdownToggle = dropdown.querySelector('.dropdown-toggle');

          if (!dropdownMenu || !dropdownToggle) {
            return;
          }

          window.clearTimeout(timeout);
          dropdownToggle.setAttribute('aria-expanded', 'true');
          dropdownMenu.classList.add('show');
        });

        dropdown.addEventListener('mouseleave', function () {
          var dropdownMenu = dropdown.querySelector('.dropdown-menu');
          var dropdownToggle = dropdown.querySelector('.dropdown-toggle');

          if (!dropdownMenu || !dropdownToggle) {
            return;
          }

          timeout = window.setTimeout(function () {
            dropdownToggle.setAttribute('aria-expanded', 'false');
            dropdownMenu.classList.remove('show');
          }, 300);
        });
      });
    }
  }

  document.querySelectorAll('a[href^="#"]').forEach(function (link) {
    link.addEventListener('click', function (event) {
      var target = link.getAttribute('href');
      var targetElement;

      if (target === '#') {
        return;
      }

      targetElement = document.querySelector(target);
      if (!targetElement) {
        return;
      }

      event.preventDefault();

      if (link.classList.contains('skip-link')) {
        targetElement.focus({ preventScroll: true });
        window.scrollTo(0, Math.max(targetElement.offsetTop - (navbar ? navbar.offsetHeight : 0), 0));
        return;
      }

      smoothScroll(target, 800);
      closeMobileMenu();
    });
  });

  if (searchLink) {
    searchLink.addEventListener('click', function (event) {
      event.preventDefault();

      if (searchIcon) {
        searchIcon.style.transform = 'rotate(360deg)';
        searchIcon.style.transition = 'transform 0.5s ease';
        setTimeout(function () {
          searchIcon.style.transform = 'rotate(0deg)';
        }, 500);
      }

      closeMobileMenu();

      if (window.openSearch) {
        window.openSearch();
      }
    });
  }

  if (navbar && navbarToggler && navbarCollapse) {
    navbarToggler.addEventListener('click', function (event) {
      var isExpanded;

      event.preventDefault();
      event.stopPropagation();

      isExpanded = navbarToggler.getAttribute('aria-expanded') === 'true';

      if (isExpanded) {
        closeMobileMenu();
        return;
      }

      navbarCollapse.classList.add('show');
      navbarToggler.setAttribute('aria-expanded', 'true');
      document.body.style.overflow = 'hidden';
    });

    document.addEventListener('click', function (event) {
      if (!navbar.contains(event.target) && navbarCollapse.classList.contains('show')) {
        closeMobileMenu();
      }
    });
  }

  if (backToTopButton) {
    window.addEventListener('scroll', function () {
      backToTopButton.classList.toggle('visible', window.pageYOffset > 300);
    });

    backToTopButton.addEventListener('click', function () {
      window.scrollTo({ top: 0, behavior: 'smooth' });
    });
  }

  function buildDemoEmbedUrl(source) {
    var url;

    if (!source) {
      return '';
    }

    try {
      url = new URL(source, window.location.origin);
    } catch (error) {
      return source;
    }

    if (url.hostname.indexOf('youtube.com') !== -1 || url.hostname.indexOf('youtu.be') !== -1) {
      url.searchParams.set('autoplay', '1');
      url.searchParams.set('rel', '0');
      url.searchParams.set('modestbranding', '1');
    }

    return url.toString();
  }

  function closeDemoModal() {
    if (!demoModal || demoModal.hidden) {
      return;
    }

    demoModal.hidden = true;
    demoModal.setAttribute('aria-hidden', 'true');
    document.body.classList.remove('demo-modal-open');

    demoModalHiddenSiblings.forEach(function (item) {
      if (!item.hadInert) {
        item.element.removeAttribute('inert');
      }
    });
    demoModalHiddenSiblings = [];

    if (demoModalFrame) {
      demoModalFrame.src = '';
      demoModalFrame.title = 'Project demo video';
    }

    if (activeDemoTrigger && typeof activeDemoTrigger.focus === 'function') {
      activeDemoTrigger.focus();
    }

    activeDemoTrigger = null;
  }

  function openDemoModal(trigger) {
    var embedSource;
    var resolvedTitle;

    if (!demoModal || !demoModalFrame || !trigger) {
      return;
    }

    embedSource = trigger.getAttribute('data-demo-embed');
    if (!embedSource) {
      return;
    }

    resolvedTitle = trigger.getAttribute('data-demo-title') ||
      trigger.getAttribute('aria-label') ||
      'Project demo video';

    activeDemoTrigger = trigger;

    demoModalHiddenSiblings = Array.from(document.body.children)
      .filter(function (element) {
        return element !== demoModal && element.tagName !== 'SCRIPT';
      })
      .map(function (element) {
        var hadInert = element.hasAttribute('inert');
        element.setAttribute('inert', '');
        return { element: element, hadInert: hadInert };
      });

    if (demoModalTitle) {
      demoModalTitle.textContent = resolvedTitle;
    }

    demoModalFrame.title = resolvedTitle;
    demoModalFrame.src = buildDemoEmbedUrl(embedSource);
    demoModal.hidden = false;
    demoModal.setAttribute('aria-hidden', 'false');
    document.body.classList.add('demo-modal-open');

    if (demoModalClose) {
      demoModalClose.focus();
    }
  }

  if (demoModal) {
    document.addEventListener('click', function (event) {
      var trigger = event.target.closest('[data-demo-embed]');
      var closeTrigger = event.target.closest('[data-close-demo-modal]');

      if (closeTrigger) {
        event.preventDefault();
        closeDemoModal();
        return;
      }

      if (!trigger) {
        return;
      }

      if (event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) {
        return;
      }

      event.preventDefault();
      openDemoModal(trigger);
    });

    document.addEventListener('keydown', function (event) {
      if (event.key === 'Escape') {
        closeDemoModal();
        if (navbarToggler && navbarToggler.getAttribute('aria-expanded') === 'true') {
          closeMobileMenu();
          navbarToggler.focus();
        }
        return;
      }

      if (event.key === 'Tab' && demoModal && !demoModal.hidden) {
        var focusable = Array.from(demoModal.querySelectorAll('button, a[href], iframe, [tabindex]:not([tabindex="-1"])'))
          .filter(function (element) { return !element.disabled; });

        if (focusable.length === 0) {
          event.preventDefault();
          return;
        }

        var first = focusable[0];
        var last = focusable[focusable.length - 1];

        if (event.shiftKey && document.activeElement === first) {
          event.preventDefault();
          last.focus();
        } else if (!event.shiftKey && document.activeElement === last) {
          event.preventDefault();
          first.focus();
        }
      }
    });
  }
});

window.addEventListener('beforeunload', function () {
  var navbar = document.querySelector('.navbar-custom');

  if (navbar) {
    navbar.style.opacity = '0.5';
    navbar.style.transform = 'translateY(-10px)';
  }
});
