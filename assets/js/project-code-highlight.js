document.addEventListener('DOMContentLoaded', () => {
  if (!window.hljs) return;

  document.querySelectorAll('.project-content pre code').forEach((block) => {
    if (block.classList.contains('language-text')) {
      block.classList.remove('language-text');
      block.classList.add('language-plaintext');
    }

    if (!block.dataset.highlighted) {
      window.hljs.highlightElement(block);
    }
  });
});
