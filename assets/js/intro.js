document.addEventListener('DOMContentLoaded', () => {
  const v = document.getElementById('intro-video');
  v.addEventListener('ended', () => {
    document.getElementById('intro-header').remove();
    document.getElementById('main-content').style.visibility = 'visible';
  });
});
