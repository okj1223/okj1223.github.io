// Radar chart with fixed 0–100 scale
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
    scale: {
      ticks: {
        beginAtZero: true,
        min: 0,        // 최소값
        max: 100,      // 최대값
        stepSize: 20   // 눈금 간격
      },
      pointLabels: {
        fontSize: 14,
        fontColor: '#333'
      },
      gridLines: {
        color: '#ccc'
      }
    },
    legend: {
      position: 'bottom'
    }
  }
});
