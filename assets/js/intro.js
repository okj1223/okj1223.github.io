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
  var saveData = Boolean(connection && connection.saveData);
  var shouldEnhanceVideo = !prefersReducedMotion && !saveData;
  var hasStarted = false;

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
    if (hasStarted) {
      return;
    }

    hasStarted = true;
    wrapper.classList.add('video-ready');
    video.muted = true;
    video.defaultMuted = true;
    video.setAttribute('muted', '');
    video.setAttribute('playsinline', '');

    var playAttempt = video.play();
    if (playAttempt && typeof playAttempt.catch === 'function') {
      playAttempt.catch(function() {
        hasStarted = false;
        markStaticMode();
      });
    }
  }

  if (!shouldEnhanceVideo) {
    markStaticMode();
    return;
  }

  wrapper.classList.add('video-enhanced');
  video.addEventListener('canplay', startVideoPlayback, { once: true });
  video.addEventListener('loadeddata', startVideoPlayback, { once: true });
  video.addEventListener('error', markStaticMode, { once: true });

  loadIntroVideo();

  document.addEventListener('visibilitychange', function() {
    if (document.hidden) {
      video.pause();
      return;
    }

    if (wrapper.classList.contains('video-ready')) {
      hasStarted = false;
      startVideoPlayback();
    }
  });
});
