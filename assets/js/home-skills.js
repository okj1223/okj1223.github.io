document.addEventListener('DOMContentLoaded', function () {
  var filterButtons = Array.from(document.querySelectorAll('.filter-btn'));
  var skillItems = Array.from(document.querySelectorAll('.unified-skills-grid .skill-item'));

  if (filterButtons.length === 0 || skillItems.length === 0) {
    return;
  }

  function showSkillItem(item) {
    clearTimeout(item._skillFilterTimer);
    item.style.setProperty('display', 'flex', 'important');
    item.style.opacity = '0';
    item.style.transform = 'translateY(20px)';

    item._skillFilterTimer = setTimeout(function () {
      item.style.transition = 'all 0.3s ease';
      item.style.opacity = '1';
      item.style.transform = 'translateY(0)';
    }, 50);
  }

  function hideSkillItem(item) {
    clearTimeout(item._skillFilterTimer);
    item.style.transition = 'all 0.3s ease';
    item.style.opacity = '0';
    item.style.transform = 'translateY(20px)';

    item._skillFilterTimer = setTimeout(function () {
      item.style.setProperty('display', 'none', 'important');
    }, 300);
  }

  filterButtons.forEach(function (button) {
    button.addEventListener('click', function () {
      var filter = button.getAttribute('data-filter');

      filterButtons.forEach(function (btn) {
        var isActive = btn === button;
        btn.classList.toggle('active', isActive);
        btn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
      });

      skillItems.forEach(function (item) {
        var categories = (item.getAttribute('data-categories') || item.getAttribute('data-category') || '')
          .split('||')
          .filter(Boolean);

        if (filter === 'all' || categories.indexOf(filter) !== -1) {
          showSkillItem(item);
          return;
        }

        hideSkillItem(item);
      });
    });
  });
});
