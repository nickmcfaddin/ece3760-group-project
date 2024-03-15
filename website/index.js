
// Set the default target page to HOME
window.onload = function() {
  if (!window.location.hash || window.location.hash === '#') {
    window.location.hash = '#home';
  }
};
