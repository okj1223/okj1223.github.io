document.addEventListener('DOMContentLoaded', function() {
  var skip = document.getElementById('skip-button');
  var video = document.getElementById('intro-video');
  var wrapper = document.querySelector('.intro-wrapper');

  if (skip) {
    skip.addEventListener('click', function() {
      var dest = this.getAttribute('data-href');
      if (dest) location.href = dest;
    });
  }

  if (!video || !wrapper) return;

  var connection = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
  var prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var smallViewport = window.matchMedia('(max-width: 767px)').matches;
  var saveData = Boolean(connection && connection.saveData);
  var effectiveType = connection && connection.effectiveType ? connection.effectiveType : '';
  var slowConnection = /(^|[^a-z])(slow-2g|2g)([^a-z]|$)/i.test(effectiveType);
  var shouldEnhanceVideo = !prefersReducedMotion && !smallViewport && !saveData && !slowConnection;

  function markStaticMode() {
    wrapper.classList.add('video-disabled');
  }

  function loadIntroVideo() {
    var sources = video.querySelectorAll('source[data-src]');

    if (!sources.length) return;

    Array.prototype.forEach.call(sources, function(source) {
      if (source.getAttribute('src')) return;
      source.src = source.getAttribute('data-src');
    });

    video.load();
  }

  function startVideoPlayback() {
    wrapper.classList.add('video-ready');

    var playAttempt = video.play();
    if (playAttempt && typeof playAttempt.catch === 'function') {
      playAttempt.catch(markStaticMode);
    }
  }

  if (!shouldEnhanceVideo) {
    markStaticMode();
    return;
  }

  wrapper.classList.add('video-enhanced');
  video.addEventListener('canplay', startVideoPlayback, { once: true });
  video.addEventListener('error', markStaticMode, { once: true });

  if (typeof window.requestIdleCallback === 'function') {
    window.requestIdleCallback(loadIntroVideo, { timeout: 1200 });
  } else {
    window.setTimeout(loadIntroVideo, 250);
  }

  document.addEventListener('visibilitychange', function() {
    if (document.hidden) {
      video.pause();
      return;
    }

    if (wrapper.classList.contains('video-ready')) {
      startVideoPlayback();
    }
  });
});
