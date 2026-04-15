// Dean Attali / Beautiful Jekyll 2023

let BeautifulJekyllJS = {

  bigImgEl : null,
  numImgs : null,
  searchInitialized : false,
  searchLoadingPromise : null,

  init : function() {
    setTimeout(BeautifulJekyllJS.initNavbar, 10);

    // Shorten the navbar after scrolling a little bit down
    $(window).scroll(function() {
        if ($(".navbar").offset().top > 50) {
            $(".navbar").addClass("top-nav-short");
        } else {
            $(".navbar").removeClass("top-nav-short");
        }
    });

    // On mobile, hide the avatar when expanding the navbar menu
    $('#main-navbar').on('show.bs.collapse', function () {
      $(".navbar").addClass("top-nav-expanded");
    });
    $('#main-navbar').on('hidden.bs.collapse', function () {
      $(".navbar").removeClass("top-nav-expanded");
    });

    // show the big header image
    BeautifulJekyllJS.initImgs();

    BeautifulJekyllJS.initSearch();
  },

  initNavbar : function() {
    // Set the navbar-dark/light class based on its background color
    const rgb = $('.navbar').css("background-color").replace(/[^\d,]/g,'').split(",");
    const brightness = Math.round(( // http://www.w3.org/TR/AERT#color-contrast
      parseInt(rgb[0]) * 299 +
      parseInt(rgb[1]) * 587 +
      parseInt(rgb[2]) * 114
    ) / 1000);
    if (brightness <= 125) {
      $(".navbar").removeClass("navbar-light").addClass("navbar-dark");
    } else {
      $(".navbar").removeClass("navbar-dark").addClass("navbar-light");
    }
  },

  initImgs : function() {
    // If the page was large images to randomly select from, choose an image
    if ($("#header-big-imgs").length > 0) {
      BeautifulJekyllJS.bigImgEl = $("#header-big-imgs");
      BeautifulJekyllJS.numImgs = BeautifulJekyllJS.bigImgEl.attr("data-num-img");

      // 2fc73a3a967e97599c9763d05e564189
      // set an initial image
      const imgInfo = BeautifulJekyllJS.getImgInfo();
      const src = imgInfo.src;
      const desc = imgInfo.desc;
      BeautifulJekyllJS.setImg(src, desc);

      // For better UX, prefetch the next image so that it will already be loaded when we want to show it
      const getNextImg = function() {
        const imgInfo = BeautifulJekyllJS.getImgInfo();
        const src = imgInfo.src;
        const desc = imgInfo.desc;

        const prefetchImg = new Image();
        prefetchImg.src = src;
        // if I want to do something once the image is ready: `prefetchImg.onload = function(){}`

        setTimeout(function(){
          const img = $("<div></div>").addClass("big-img-transition").css("background-image", 'url(' + src + ')');
          $(".intro-header.big-img").prepend(img);
          setTimeout(function(){ img.css("opacity", "1"); }, 50);

          // after the animation of fading in the new image is done, prefetch the next one
          //img.one("transitioned webkitTransitionEnd oTransitionEnd MSTransitionEnd", function(){
          setTimeout(function() {
            BeautifulJekyllJS.setImg(src, desc);
            img.remove();
            getNextImg();
          }, 1000);
          //});
        }, 6000);
      };

      // If there are multiple images, cycle through them
      if (BeautifulJekyllJS.numImgs > 1) {
        getNextImg();
      }
    }
  },

  getImgInfo : function() {
    const randNum = Math.floor((Math.random() * BeautifulJekyllJS.numImgs) + 1);
    const src = BeautifulJekyllJS.bigImgEl.attr("data-img-src-" + randNum);
    const desc = BeautifulJekyllJS.bigImgEl.attr("data-img-desc-" + randNum);

    return {
      src : src,
      desc : desc
    }
  },

  setImg : function(src, desc) {
    $(".intro-header.big-img").css("background-image", 'url(' + src + ')');
    if (typeof desc !== typeof undefined && desc !== false) {
      $(".img-desc").text(desc).show();
    } else {
      $(".img-desc").hide();
    }
  },

  loadScript : function(src, dataAttr) {
    if (!src) {
      return Promise.reject(new Error("Missing script source"));
    }

    const selector = dataAttr ? 'script[' + dataAttr + ']' : 'script[src="' + src + '"]';
    const existing = document.querySelector(selector);

    if (existing) {
      if (existing.dataset.loaded === "true") {
        return Promise.resolve();
      }

      return new Promise(function(resolve, reject) {
        existing.addEventListener("load", function() {
          existing.dataset.loaded = "true";
          resolve();
        }, { once : true });
        existing.addEventListener("error", reject, { once : true });
      });
    }

    return new Promise(function(resolve, reject) {
      const script = document.createElement("script");
      script.src = src;
      script.async = true;
      if (dataAttr) {
        script.setAttribute(dataAttr, "true");
      }
      script.onload = function() {
        script.dataset.loaded = "true";
        resolve();
      };
      script.onerror = reject;
      document.body.appendChild(script);
    });
  },

  initializeSearch : function(overlay, searchInput, resultsContainer) {
    if (BeautifulJekyllJS.searchInitialized || !window.SimpleJekyllSearch) {
      return;
    }

    window.SimpleJekyllSearch({
      searchInput : searchInput,
      resultsContainer : resultsContainer,
      json : overlay.dataset.searchJson
    });

    BeautifulJekyllJS.searchInitialized = true;
  },

  ensureSearchReady : function(overlay, searchInput, resultsContainer) {
    const originalPlaceholder = searchInput.dataset.originalPlaceholder || searchInput.getAttribute("placeholder") || "Search";
    searchInput.dataset.originalPlaceholder = originalPlaceholder;

    if (BeautifulJekyllJS.searchInitialized) {
      searchInput.disabled = false;
      searchInput.placeholder = originalPlaceholder;
      searchInput.removeAttribute("aria-busy");
      return Promise.resolve();
    }

    if (BeautifulJekyllJS.searchLoadingPromise) {
      return BeautifulJekyllJS.searchLoadingPromise;
    }

    searchInput.disabled = true;
    searchInput.setAttribute("aria-busy", "true");
    searchInput.placeholder = "Loading search...";
    resultsContainer.innerHTML = "<li class=\"search-result-state\">Loading search index...</li>";

    const scriptSrc = overlay.dataset.searchScript;

    BeautifulJekyllJS.searchLoadingPromise = BeautifulJekyllJS.loadScript(scriptSrc, "data-search-script")
      .then(function() {
        BeautifulJekyllJS.initializeSearch(overlay, searchInput, resultsContainer);
        searchInput.disabled = false;
        searchInput.placeholder = originalPlaceholder;
        searchInput.removeAttribute("aria-busy");
        resultsContainer.innerHTML = "";
      })
      .catch(function() {
        searchInput.disabled = true;
        searchInput.placeholder = "Search unavailable";
        resultsContainer.innerHTML = "<li class=\"search-result-state\">Search is temporarily unavailable.</li>";
      })
      .then(function() {
        BeautifulJekyllJS.searchLoadingPromise = null;
      });

    return BeautifulJekyllJS.searchLoadingPromise;
  },

  openSearchOverlay : function() {
    const overlay = document.getElementById("beautifuljekyll-search-overlay");
    const searchInput = document.getElementById("nav-search-input");
    const resultsContainer = document.getElementById("search-results-container");

    if (!overlay || !searchInput || !resultsContainer) {
      return;
    }

    $("#beautifuljekyll-search-overlay").show();
    $("body").addClass("overflow-hidden");

    BeautifulJekyllJS.ensureSearchReady(overlay, searchInput, resultsContainer).then(function() {
      if (searchInput.disabled) {
        return;
      }

      searchInput.focus();
      searchInput.select();
    });
  },

  closeSearchOverlay : function() {
    $("#beautifuljekyll-search-overlay").hide();
    $("body").removeClass("overflow-hidden");
  },

  initSearch : function() {
    if (!document.getElementById("beautifuljekyll-search-overlay") || !document.getElementById("nav-search-link")) {
      return;
    }

    $("#nav-search-link").click(function(e) {
      e.preventDefault();
      BeautifulJekyllJS.openSearchOverlay();
    });
    $("#nav-search-exit").click(function(e) {
      e.preventDefault();
      BeautifulJekyllJS.closeSearchOverlay();
    });
    $(document).on('keyup', function(e) {
      if (e.key == "Escape") {
        BeautifulJekyllJS.closeSearchOverlay();
      }
    });
  }
};

// 2fc73a3a967e97599c9763d05e564189

document.addEventListener('DOMContentLoaded', BeautifulJekyllJS.init);
