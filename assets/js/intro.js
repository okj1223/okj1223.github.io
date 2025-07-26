document.addEventListener('DOMContentLoaded', () => {
  const video = document.getElementById('intro-video');
  const intro = document.getElementById('intro-header');
  const main = document.getElementById('main-content');
  const skipBtn = document.getElementById('skip-button');

  // 영상 끝날 때도 계속 루프이므로, JS 루프 핸들러는 필요 없지만 
  // 스킵 버튼 클릭 시 인트로 제거
  skipBtn.addEventListener('click', () => {
    video.pause();
    intro.remove();
    main.style.visibility = 'visible';
  });
});
