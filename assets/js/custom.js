document.addEventListener('DOMContentLoaded', () => {
  document.querySelectorAll('.skills-bars .progress').forEach(el => {
    el.style.width = el.getAttribute('data-percent');
  });
});


// Radar chart
const ctx = document.getElementById('skillRadar').getContext('2d');
new Chart(ctx, {
  type: 'radar',
  data: {
    labels: ['Python','ROS2','Arduino','CAD','ML'],
    datasets: [{
      label: 'Proficiency',
      data: [85,70,65,80,60],
      backgroundColor: 'rgba(0,102,204,0.2)',
      borderColor: '#0066cc',
    }]
  },
  options: {
    scale: { ticks: { beginAtZero: true } }
  }
});
