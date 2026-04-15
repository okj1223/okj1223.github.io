(function() {
  'use strict';

  var THREE_CDN = 'https://cdn.jsdelivr.net/npm/three@0.150.0/build/three.min.js';
  var OCEAN_SCRIPT_SELECTOR = 'script[data-ocean-3d]';
  var THREE_SCRIPT_SELECTOR = 'script[data-three-runtime]';

  function loadScript(src, selector, attributes) {
    if (document.querySelector(selector)) {
      return Promise.resolve();
    }

    return new Promise(function(resolve, reject) {
      var script = document.createElement('script');
      script.src = src;
      script.async = true;

      if (attributes) {
        Object.keys(attributes).forEach(function(key) {
          script.setAttribute(key, attributes[key]);
        });
      }

      script.onload = resolve;
      script.onerror = reject;
      document.body.appendChild(script);
    });
  }

  function supportsWebGL() {
    try {
      var canvas = document.createElement('canvas');
      return Boolean(
        window.WebGLRenderingContext &&
        (canvas.getContext('webgl') || canvas.getContext('experimental-webgl'))
      );
    } catch (error) {
      return false;
    }
  }

  function shouldEnableOceanScene() {
    var connection = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
    var prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    var narrowViewport = window.matchMedia('(max-width: 991px)').matches;
    var saveData = Boolean(connection && connection.saveData);
    var effectiveType = connection && connection.effectiveType ? connection.effectiveType : '';
    var slowConnection = /(^|[^a-z])(slow-2g|2g)([^a-z]|$)/i.test(effectiveType);
    var lowMemory = typeof navigator.deviceMemory === 'number' && navigator.deviceMemory < 4;

    return !prefersReducedMotion &&
      !narrowViewport &&
      !saveData &&
      !slowConnection &&
      !lowMemory &&
      supportsWebGL();
  }

  function initFooterOceanLoader() {
    var section = document.querySelector('.ocean-waves-section');
    var oceanScriptSrc = section && section.getAttribute('data-ocean-script-src');

    if (!section) return;

    function setState(state) {
      section.setAttribute('data-ocean-state', state);
    }

    function loadOceanScene() {
      if (section.getAttribute('data-ocean-state') === 'loading' || section.getAttribute('data-ocean-state') === 'ready') {
        return;
      }

      setState('loading');

      loadScript(THREE_CDN, THREE_SCRIPT_SELECTOR, { 'data-three-runtime': 'true' })
        .then(function() {
          return loadScript(oceanScriptSrc || '/assets/js/ocean-waves-3d.js', OCEAN_SCRIPT_SELECTOR, { 'data-ocean-3d': 'true' });
        })
        .then(function() {
          setState('ready');
        })
        .catch(function() {
          setState('fallback');
        });
    }

    if (!shouldEnableOceanScene()) {
      setState('static');
      return;
    }

    setState('idle');

    if (!('IntersectionObserver' in window)) {
      loadOceanScene();
      return;
    }

    var observer = new IntersectionObserver(function(entries) {
      var visible = entries.some(function(entry) {
        return entry.isIntersecting;
      });

      if (!visible) return;

      observer.disconnect();
      loadOceanScene();
    }, { rootMargin: '300px 0px' });

    observer.observe(section);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initFooterOceanLoader);
  } else {
    initFooterOceanLoader();
  }
})();
