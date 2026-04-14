document.addEventListener('DOMContentLoaded', function () {
  var filterButtons = Array.from(document.querySelectorAll('.filter-btn'));
  var skillItems = Array.from(document.querySelectorAll('.unified-skills-grid .skill-item'));

  if (filterButtons.length === 0 || skillItems.length === 0) {
    return;
  }

  function showSkillItem(item) {
    item.style.display = 'flex';
    item.style.opacity = '0';
    item.style.transform = 'translateY(20px)';

    setTimeout(function () {
      item.style.transition = 'all 0.3s ease';
      item.style.opacity = '1';
      item.style.transform = 'translateY(0)';
    }, 50);
  }

  function hideSkillItem(item) {
    item.style.transition = 'all 0.3s ease';
    item.style.opacity = '0';
    item.style.transform = 'translateY(20px)';

    setTimeout(function () {
      item.style.display = 'none';
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
        var category = item.getAttribute('data-category');

        if (filter === 'all' || category === filter) {
          showSkillItem(item);
          return;
        }

        hideSkillItem(item);
      });
    });
  });
});
