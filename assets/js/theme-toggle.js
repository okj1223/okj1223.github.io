const toggle = document.getElementById('theme-toggle');
const body = document.body;

// 이전 선택 테마 읽어 적용
if (localStorage.getItem('theme') === 'dark') {
  body.classList.add('dark-mode');
}

toggle.addEventListener('click', () => {
  body.classList.toggle('dark-mode');
  const mode = body.classList.contains('dark-mode') ? 'dark' : 'light';
  localStorage.setItem('theme', mode);
});
