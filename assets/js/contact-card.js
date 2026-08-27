(function () {
  'use strict';

  const shareButton = document.querySelector('[data-share-card]');
  const shareLabel = document.querySelector('[data-share-label]');
  const shareStatus = document.querySelector('[data-share-status]');

  if (!shareButton || !shareLabel || !shareStatus) {
    return;
  }

  const cardUrl = 'https://okj1223.github.io/card/';
  let resetTimer;

  function showStatus(message) {
    window.clearTimeout(resetTimer);
    shareLabel.textContent = message;
    shareStatus.textContent = message;
    resetTimer = window.setTimeout(function () {
      shareLabel.textContent = 'Share card';
      shareStatus.textContent = '';
    }, 2200);
  }

  shareButton.addEventListener('click', async function () {
    try {
      if (navigator.share) {
        await navigator.share({
          title: 'Kyungjun Oh · NTU',
          text: 'M.S. Student in Mechanical Engineering at National Taiwan University · ASR Lab',
          url: cardUrl
        });
        showStatus('Shared');
        return;
      }

      await navigator.clipboard.writeText(cardUrl);
      showStatus('Link copied');
    } catch (error) {
      if (error && error.name === 'AbortError') {
        return;
      }

      showStatus('Copy the URL below');
    }
  });
})();
