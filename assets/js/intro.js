document.addEventListener('DOMContentLoaded', function() {
  var skip = document.getElementById('skip-button');
  skip.addEventListener('click', function() {
    location.href = this.getAttribute('data-href');
  });
});
