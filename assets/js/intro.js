document.addEventListener('DOMContentLoaded', () => {
  const video     = document.getElementById('intro-video');
  const intro     = document.getElementById('intro-header');
  const main      = document.getElementById('main-content');
  const skipBtn   = document.getElementById('skip-button');

  // 버튼 클릭하면 바로 메인 콘텐츠 노출
  skipBtn.addEventListener('click', () => {
    video.pause();                   // 재생 중지
    intro.style.display = 'none';    // 인트로 숨기기
    main.style.visibility = 'visible'; // 메인 노출
  });
});
