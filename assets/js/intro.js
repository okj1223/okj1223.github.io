document.addEventListener('DOMContentLoaded', () => {
  const skipBtn = document.getElementById('skip-button');

  skipBtn.addEventListener('click', () => {
    // 실제 홈 페이지로 이동
    window.location.href = '{{ "/home.html" | relative_url }}';
  });
});
