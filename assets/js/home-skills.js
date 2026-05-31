document.addEventListener('DOMContentLoaded', function () {
  var skillsSection = document.querySelector('.skills-section');
  var filterButtons = Array.from(document.querySelectorAll('.filter-btn'));
  var skillItems = Array.from(document.querySelectorAll('.unified-skills-grid .skill-item'));

  if (filterButtons.length === 0 || skillItems.length === 0) {
    return;
  }

  function getCategories(item) {
    return (item.getAttribute('data-categories') || item.getAttribute('data-category') || '')
      .split('||')
      .filter(Boolean);
  }

  function updateFilterState(filter) {
    var isFiltered = filter !== 'all';

    if (skillsSection) {
      skillsSection.classList.toggle('is-filtered', isFiltered);
      skillsSection.setAttribute('data-active-filter', filter);
    }

    skillItems.forEach(function (item) {
      var isMatch = !isFiltered || getCategories(item).indexOf(filter) !== -1;
      item.classList.toggle('is-selected', isFiltered && isMatch);
      item.classList.toggle('is-dimmed', isFiltered && !isMatch);
      item.setAttribute('aria-hidden', 'false');
    });
  }

  filterButtons.forEach(function (button) {
    button.addEventListener('click', function () {
      var filter = button.getAttribute('data-filter');

      filterButtons.forEach(function (btn) {
        var isActive = btn === button;
        btn.classList.toggle('active', isActive);
        btn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
      });

      updateFilterState(filter);
    });
  });

  updateFilterState('all');
});
