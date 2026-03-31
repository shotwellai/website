(function () {
  // ─── Hamburger menu toggle ───
  var toggle = document.getElementById('nav-toggle');
  var links = document.getElementById('nav-links');

  toggle.addEventListener('click', function () {
    toggle.classList.toggle('open');
    links.classList.toggle('open');
  });

  links.querySelectorAll('a').forEach(function (link) {
    link.addEventListener('click', function () {
      toggle.classList.remove('open');
      links.classList.remove('open');
    });
  });

  // ─── Nav scroll effect ───
  var nav = document.getElementById('nav');
  var scrolled = false;

  function checkScroll() {
    var shouldBeScrolled = window.scrollY > 40;
    if (shouldBeScrolled !== scrolled) {
      scrolled = shouldBeScrolled;
      nav.classList.toggle('scrolled', scrolled);
    }
  }

  window.addEventListener('scroll', checkScroll, { passive: true });
  checkScroll();

  // ─── Scroll-triggered reveals ───
  var revealEls = document.querySelectorAll('.reveal, .reveal-group');

  if ('IntersectionObserver' in window) {
    var observer = new IntersectionObserver(
      function (entries) {
        entries.forEach(function (entry) {
          if (entry.isIntersecting) {
            entry.target.classList.add('visible');
            observer.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.12, rootMargin: '0px 0px -40px 0px' }
    );

    revealEls.forEach(function (el) {
      observer.observe(el);
    });
  } else {
    revealEls.forEach(function (el) {
      el.classList.add('visible');
    });
  }

  // ─── Staggered hero reveals ───
  var heroReveals = document.querySelectorAll('.hero .reveal');
  heroReveals.forEach(function (el, i) {
    el.style.transitionDelay = (i * 0.12) + 's';
  });

  // Trigger hero reveals immediately after short delay
  setTimeout(function () {
    heroReveals.forEach(function (el) {
      el.classList.add('visible');
    });
  }, 100);
})();
