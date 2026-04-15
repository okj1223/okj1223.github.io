document.addEventListener('DOMContentLoaded', function () {
  var defaultFilter = 'all';
  var currentFilter = defaultFilter;
  var currentPage = 1;
  var itemsPerPage = 5;

  var filterBar = document.querySelector('.project-filter-bar');
  var paginationSummary = document.querySelector('.projects-pagination-summary');
  var filterBtns = Array.from(document.querySelectorAll('.proj-filter-btn'));
  var articles = Array.from(document.querySelectorAll('.home-project-item'));
  var pagination = document.querySelector('.pagination-controls');
  var pageInfo = document.querySelector('.pagination-info');
  var emptyState = document.getElementById('projectsEmptyState');
  var currentPageSpan = document.getElementById('currentPage');
  var totalPagesSpan = document.getElementById('totalPages');
  var filteredProjectsSpan = document.getElementById('filteredProjectsCount');
  var allProjectsSpan = document.getElementById('allProjectsCount');
  var prevBtn = document.getElementById('prevBtn');
  var nextBtn = document.getElementById('nextBtn');
  var pageNumbers = document.querySelector('.page-numbers');
  var allowedFilters = filterBtns
    .map(function (btn) { return btn.dataset.filter; })
    .filter(Boolean);

  if (filterBar) filterBar.hidden = false;
  if (paginationSummary) paginationSummary.hidden = false;
  if (pagination) pagination.hidden = false;

  function getArticleCategories(article) {
    return (article.dataset.category || '')
      .trim()
      .split(/\s+/)
      .filter(Boolean);
  }

  function isValidFilter(filter) {
    return allowedFilters.indexOf(filter) !== -1;
  }

  function parsePositiveInteger(value, fallback) {
    var parsed = parseInt(value, 10);
    return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
  }

  function readStateFromUrl() {
    var params = new URLSearchParams(window.location.search);
    var filter = params.get('filter') || defaultFilter;
    var page = parsePositiveInteger(params.get('page'), 1);

    if (!isValidFilter(filter)) {
      filter = defaultFilter;
    }

    return {
      filter: filter,
      page: page
    };
  }

  function writeStateToUrl(mode) {
    if (!window.history || !window.history.replaceState) return;

    var url = new URL(window.location.href);

    if (currentFilter === defaultFilter) {
      url.searchParams.delete('filter');
    } else {
      url.searchParams.set('filter', currentFilter);
    }

    if (currentPage <= 1) {
      url.searchParams.delete('page');
    } else {
      url.searchParams.set('page', String(currentPage));
    }

    var nextUrl = url.pathname + (url.search ? url.search : '') + url.hash;
    var currentUrl = window.location.pathname + window.location.search + window.location.hash;

    if (nextUrl === currentUrl) return;

    if (mode === 'push' && window.history.pushState) {
      window.history.pushState({ filter: currentFilter, page: currentPage }, '', nextUrl);
      return;
    }

    window.history.replaceState({ filter: currentFilter, page: currentPage }, '', nextUrl);
  }

  function matchesFilter(article) {
    if (currentFilter === defaultFilter) {
      return true;
    }

    return getArticleCategories(article).indexOf(currentFilter) !== -1;
  }

  function getFilteredArticles() {
    return articles.filter(matchesFilter);
  }

  function getTotalPages(filteredCount) {
    return Math.max(1, Math.ceil(filteredCount / itemsPerPage));
  }

  function setArticleVisibility(article, shouldShow) {
    article.hidden = !shouldShow;

    if (shouldShow) {
      article.style.removeProperty('display');
      return;
    }

    article.style.setProperty('display', 'none', 'important');
  }

  function setActiveFilterButton() {
    filterBtns.forEach(function (btn) {
      var isActive = btn.dataset.filter === currentFilter;
      btn.classList.toggle('active', isActive);
      btn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
    });
  }

  function updateFilterButtonCounts() {
    var counts = { all: articles.length };

    articles.forEach(function (article) {
      getArticleCategories(article).forEach(function (category) {
        counts[category] = (counts[category] || 0) + 1;
      });
    });

    filterBtns.forEach(function (btn) {
      var baseLabel = btn.dataset.baseLabel || btn.textContent.trim();
      var filter = btn.dataset.filter;
      var count = counts[filter] || 0;

      btn.dataset.baseLabel = baseLabel;
      btn.innerHTML = '<span class="proj-filter-label">' + baseLabel + '</span><span class="proj-filter-count">' + count + '</span>';
      btn.setAttribute('aria-label', baseLabel + ' (' + count + ' projects)');
    });
  }

  function buildPageButtons(totalPages) {
    if (!pageNumbers) return;

    pageNumbers.innerHTML = '';

    for (var page = 1; page <= totalPages; page++) {
      var button = document.createElement('button');
      button.className = 'page-number-btn' + (page === currentPage ? ' active' : '');
      button.dataset.page = String(page);
      button.textContent = String(page);
      pageNumbers.appendChild(button);
    }
  }

  function render(options) {
    var settings = options || {};
    var filteredArticles = getFilteredArticles();
    var totalFiltered = filteredArticles.length;
    var totalPages = getTotalPages(totalFiltered);
    var startIndex;
    var endIndex;
    var visibleArticles;
    var visibleSet;

    currentPage = Math.min(Math.max(currentPage, 1), totalPages);

    startIndex = (currentPage - 1) * itemsPerPage;
    endIndex = startIndex + itemsPerPage;
    visibleArticles = filteredArticles.slice(startIndex, endIndex);
    visibleSet = new Set(visibleArticles);

    articles.forEach(function (article) {
      setArticleVisibility(article, visibleSet.has(article));
    });

    setActiveFilterButton();

    if (emptyState) {
      emptyState.hidden = totalFiltered !== 0;
    }

    if (pageInfo) pageInfo.style.display = totalFiltered === 0 ? 'none' : '';
    if (pagination) pagination.style.display = totalPages > 1 ? '' : 'none';

    if (currentPageSpan) currentPageSpan.textContent = totalFiltered === 0 ? '0' : String(currentPage);
    if (totalPagesSpan) totalPagesSpan.textContent = String(totalPages);
    if (filteredProjectsSpan) filteredProjectsSpan.textContent = String(totalFiltered);
    if (allProjectsSpan) allProjectsSpan.textContent = String(articles.length);
    if (prevBtn) prevBtn.disabled = currentPage === 1 || totalFiltered === 0;
    if (nextBtn) nextBtn.disabled = currentPage === totalPages || totalFiltered === 0;

    buildPageButtons(totalPages);

    if (settings.urlMode) {
      writeStateToUrl(settings.urlMode);
    }
  }

  function updateState(nextFilter, nextPage, options) {
    var settings = options || {};

    currentFilter = isValidFilter(nextFilter) ? nextFilter : defaultFilter;
    currentPage = parsePositiveInteger(nextPage, 1);

    render({ urlMode: settings.urlMode || 'push' });

    if (settings.scroll) {
      scrollToGrid();
    }
  }

  function applyStateFromUrl() {
    var state = readStateFromUrl();
    currentFilter = state.filter;
    currentPage = state.page;
    render({ urlMode: 'replace' });
  }

  filterBtns.forEach(function (btn) {
    btn.addEventListener('click', function () {
      updateState(this.dataset.filter, 1, { urlMode: 'push' });
    });
  });

  if (prevBtn) {
    prevBtn.addEventListener('click', function () {
      if (currentPage === 1) return;
      updateState(currentFilter, currentPage - 1, { urlMode: 'push', scroll: true });
    });
  }

  if (nextBtn) {
    nextBtn.addEventListener('click', function () {
      var totalPages = getTotalPages(getFilteredArticles().length);
      if (currentPage >= totalPages) return;
      updateState(currentFilter, currentPage + 1, { urlMode: 'push', scroll: true });
    });
  }

  if (pageNumbers) {
    pageNumbers.addEventListener('click', function (event) {
      var target = event.target.closest('.page-number-btn');
      if (!target) return;

      updateState(currentFilter, target.dataset.page, { urlMode: 'push', scroll: true });
    });
  }

  document.addEventListener('keydown', function (event) {
    var totalPages = getTotalPages(getFilteredArticles().length);
    var targetTag = event.target && event.target.tagName;

    if (targetTag === 'INPUT' || targetTag === 'TEXTAREA' || targetTag === 'SELECT') {
      return;
    }

    if (event.key === 'ArrowLeft' && currentPage > 1) {
      updateState(currentFilter, currentPage - 1, { urlMode: 'push', scroll: true });
    }

    if (event.key === 'ArrowRight' && currentPage < totalPages) {
      updateState(currentFilter, currentPage + 1, { urlMode: 'push', scroll: true });
    }
  });

  window.addEventListener('popstate', function () {
    applyStateFromUrl();
  });

  function scrollToGrid() {
    var grid = document.getElementById('projectsGrid');
    if (!grid) return;
    window.scrollTo({ top: grid.offsetTop - 100, behavior: 'smooth' });
  }

  var bannerCount = document.querySelector('.stat-number');
  if (bannerCount && bannerCount.textContent.indexOf('+') !== -1) {
    bannerCount.textContent = articles.length + '+';
  }

  updateFilterButtonCounts();
  applyStateFromUrl();
});
