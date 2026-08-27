(function () {
  'use strict';

  const shareButton = document.querySelector('[data-share-card]');
  const shareLabel = document.querySelector('[data-share-label]');
  const shareStatus = document.querySelector('[data-share-status]');
  const saveImageLink = document.querySelector('[data-save-card-image]');
  const saveImageLabel = document.querySelector('[data-save-image-label]');

  if (!shareStatus) {
    return;
  }

  const cardUrl = 'https://okj1223.github.io/card/';
  const imageFileName = 'kyungjun-oh-contact-card.png';
  let resetTimer;

  function showStatus(message) {
    window.clearTimeout(resetTimer);
    shareLabel.textContent = message;
    shareStatus.textContent = message;
    resetTimer = window.setTimeout(function () {
      shareLabel.textContent = 'Share link';
      shareStatus.textContent = '';
    }, 2200);
  }

  if (shareButton && shareLabel) {
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
  }

  if (saveImageLink && saveImageLabel) {
    saveImageLink.addEventListener('click', async function (event) {
      event.preventDefault();
      const imageUrl = saveImageLink.href;
      const defaultLabel = 'Save card image';

      try {
        saveImageLabel.textContent = 'Preparing image…';
        shareStatus.textContent = 'Choose “Save Image” in the share sheet.';

        const response = await fetch(imageUrl, { cache: 'force-cache' });
        if (!response.ok) {
          throw new Error('Image request failed');
        }

        const imageBlob = await response.blob();
        const imageFile = new File([imageBlob], imageFileName, { type: 'image/png' });
        const canShareImage = navigator.share &&
          navigator.canShare &&
          navigator.canShare({ files: [imageFile] });

        if (!canShareImage) {
          window.location.assign(imageUrl);
          return;
        }

        await navigator.share({
          files: [imageFile],
          title: 'Kyungjun Oh · Contact Card'
        });
        shareStatus.textContent = 'Image share completed.';
      } catch (error) {
        if (!error || error.name !== 'AbortError') {
          window.location.assign(imageUrl);
          return;
        }
      } finally {
        saveImageLabel.textContent = defaultLabel;
        window.clearTimeout(resetTimer);
        resetTimer = window.setTimeout(function () {
          shareStatus.textContent = '';
        }, 3200);
      }
    });
  }
})();
