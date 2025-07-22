// Radar chart with full 0–100 grid lines
const ctx = document.getElementById('skillRadar').getContext('2d');
new Chart(ctx, {
  type: 'radar',
  data: {
    labels: ['Python','ROS2','Arduino','CAD','ML'],
    datasets: [{
      label: 'Proficiency',
      data: [90, 80, 70, 85, 75],
      backgroundColor: 'rgba(0,102,204,0.2)',
      borderColor: '#0066cc',
      pointBackgroundColor: '#0066cc',
      pointRadius: 4
    }]
  },
  options: {
    scales: {
      r: {                        // v3에서는 'r' 축으로 설정
        min: 0,
        max: 100,
        beginAtZero: true,
        ticks: {
          stepSize: 20,
          backdropColor: 'rgba(255,255,255,0)',  // 눈금 라벨 배경 제거
          color: '#666'
        },
        grid: {
          color: '#ddd'
        },
        pointLabels: {
          font: {
            size: 14
          },
          color: '#333'
        }
      }
    },
    plugins: {
      legend: {
        position: 'bottom'
      },
      tooltip: {
        callbacks: {
          label: ctx => ` ${ctx.dataset.label}: ${ctx.parsed.r}`
        }
      }
    },
    maintainAspectRatio: false     // 컨테이너에 맞게 크기 조정
  }
});
