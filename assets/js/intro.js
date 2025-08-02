document.addEventListener('DOMContentLoaded', function() {
  var skip = document.getElementById('skip-button');
  if (!skip) return;
  skip.addEventListener('click', function() {
    var dest = this.getAttribute('data-href');
    if (dest) location.href = dest;
  });
});
