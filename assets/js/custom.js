// Radar chart
const ctx = document.getElementById('skillRadar').getContext('2d');
new Chart(ctx, {
  type: 'radar',
  data: {
    labels: ['Python','ROS2','Arduino','CAD','ML'],
    datasets: [{
      label: 'Proficiency',
      data: [85,80,75,85,75],
      backgroundColor: 'rgba(0,102,204,0.2)',
      borderColor: '#0066cc',
    }]
  },
  options: {
    scale: { ticks: { beginAtZero: true } }
  }
});
