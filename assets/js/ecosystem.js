// Living Technology Ecosystem JavaScript
document.addEventListener('DOMContentLoaded', function() {
  
  // 스크롤 진행률 업데이트
  function updateScrollProgress() {
    const progressLine = document.getElementById('scroll-progress-line');
    const progressNodes = document.querySelectorAll('.progress-node');
    
    if (progressLine) {
      const scrollPercent = (window.scrollY / (document.documentElement.scrollHeight - window.innerHeight)) * 100;
      progressLine.style.height = Math.min(scrollPercent, 100) + '%';
      
      // 노드 활성화
      progressNodes.forEach((node, index) => {
        const threshold = (index + 1) * 25;
        if (scrollPercent >= threshold) {
          node.classList.add('active');
        } else {
          node.classList.remove('active');
        }
      });
    }
  }
  
  // 환경 데이터 실시간 업데이트
  function updateEnvironmentalData() {
    const co2Val = document.getElementById('co2-val');
    const pwrVal = document.getElementById('pwr-val');
    const effVal = document.getElementById('eff-val');
    const co2Bar = document.getElementById('co2-bar');
    const pwrBar = document.getElementById('pwr-bar');
    const effBar = document.getElementById('eff-bar');
    
    if (co2Val) {
      const co2 = 340 + Math.floor(Math.random() * 15);
      co2Val.textContent = co2;
      if (co2Bar) {
        co2Bar.style.width = (co2 / 400 * 100) + '%';
      }
    }
    
    if (pwrVal) {
      const pwr = 85 + Math.floor(Math.random() * 10);
      pwrVal.textContent = pwr;
      if (pwrBar) {
        pwrBar.style.width = pwr + '%';
      }
    }
    
    if (effVal) {
      const eff = 92 + Math.floor(Math.random() * 6);
      effVal.textContent = eff;
      if (effBar) {
        effBar.style.width = eff + '%';
      }
    }
  }
  
  // 기술 아이콘 호버 효과
  function addTechIconTooltips() {
    const techItems = document.querySelectorAll('.tech-icon-item');
    
    techItems.forEach(item => {
      const techName = item.getAttribute('data-tech');
      
      // 포인터 이벤트 활성화
      item.style.pointerEvents = 'auto';
      
      item.addEventListener('mouseenter', function(e) {
        const tooltip = document.createElement('div');
        tooltip.className = 'tech-tooltip';
        tooltip.textContent = techName;
        tooltip.style.cssText = `
          position: absolute;
          background: rgba(0,0,0,0.9);
          color: white;
          padding: 4px 8px;
          border-radius: 4px;
          font-size: 0.7rem;
          pointer-events: none;
          z-index: 10000;
          right: 45px;
          top: 50%;
          transform: translateY(-50%);
          white-space: nowrap;
          font-family: 'Courier New', monospace;
          opacity: 0;
          transition: opacity 0.3s ease;
        `;
        this.appendChild(tooltip);
        
        // 페이드인 효과
        setTimeout(() => {
          tooltip.style.opacity = '1';
        }, 10);
      });
      
      item.addEventListener('mouseleave', function() {
        const tooltip = this.querySelector('.tech-tooltip');
        if (tooltip) {
          tooltip.style.opacity = '0';
          setTimeout(() => {
            tooltip.remove();
          }, 300);
        }
      });
    });
  }
  
  // 스무스 스크롤 for progress nodes
  function addProgressNodeClicks() {
    const progressNodes = document.querySelectorAll('.progress-node');
    const sections = ['banner', 'projects', 'skills', 'career'];
    
    progressNodes.forEach((node, index) => {
      node.style.pointerEvents = 'auto';
      node.addEventListener('click', function() {
        // 섹션별 대략적인 스크롤 위치 계산
        let scrollPosition;
        
        switch(index) {
          case 0: // banner
            scrollPosition = 0;
            break;
          case 1: // projects
            scrollPosition = window.innerHeight * 0.8;
            break;
          case 2: // skills
            scrollPosition = window.innerHeight * 1.8;
            break;
          case 3: // career
            scrollPosition = window.innerHeight * 2.5;
            break;
          default:
            scrollPosition = window.innerHeight * index;
        }
        
        window.scrollTo({ 
          top: scrollPosition, 
          behavior: 'smooth' 
        });
      });
    });
  }
  
  // 기술 융합 시각화 인터랙션
  function addFusionInteraction() {
    const techSpheres = document.querySelectorAll('.tech-sphere');
    
    techSpheres.forEach(sphere => {
      sphere.style.pointerEvents = 'auto';
      
      sphere.addEventListener('mouseenter', function() {
        this.style.transform = 'translateX(-50%) scale(1.3)';
        this.style.zIndex = '1001';
        this.style.transition = 'all 0.3s ease';
      });
      
      sphere.addEventListener('mouseleave', function() {
        this.style.transform = 'translateX(-50%) scale(1)';
        this.style.zIndex = 'auto';
      });
    });
  }
  
  // 데이터 패킷 애니메이션 향상
  function enhanceDataStream() {
    const dataPackets = document.querySelectorAll('.data-packet');
    
    dataPackets.forEach((packet, index) => {
      // 랜덤한 애니메이션 지연 추가
      const delay = Math.random() * 3;
      packet.style.animationDelay = delay + 's';
      
      // 랜덤한 크기 변화
      const scale = 0.8 + Math.random() * 0.4;
      packet.style.transform = `scaleY(${scale})`;
    });
  }
  
  // 배경 그리드 인터랙션 (선택적)
  function addGridInteraction() {
    const gridPattern = document.querySelector('.bg-grid-pattern');
    
    if (gridPattern) {
      document.addEventListener('mousemove', function(e) {
        const mouseX = e.clientX / window.innerWidth;
        const mouseY = e.clientY / window.innerHeight;
        
        // 마우스 위치에 따른 그리드 투명도 변화
        const opacity = 0.02 + (mouseX * mouseY * 0.03);
        gridPattern.style.opacity = Math.min(opacity, 0.08);
      });
    }
  }
  
  // 성능 최적화된 스크롤 이벤트
  let ticking = false;
  function requestTick() {
    if (!ticking) {
      requestAnimationFrame(updateScrollProgress);
      ticking = true;
    }
  }
  
  function handleScroll() {
    requestTick();
    ticking = false;
  }
  
  // 초기 페이드인 애니메이션
  function initFadeIn() {
    const leftPanel = document.querySelector('.eco-system-left');
    const rightPanel = document.querySelector('.eco-system-right');
    
    if (leftPanel) {
      leftPanel.style.opacity = '0';
      leftPanel.style.transform = 'translateY(-50%) translateX(-20px)';
      leftPanel.style.transition = 'all 0.8s ease';
      
      setTimeout(() => {
        leftPanel.style.opacity = '1';
        leftPanel.style.transform = 'translateY(-50%) translateX(0)';
      }, 300);
    }
    
    if (rightPanel) {
      rightPanel.style.opacity = '0';
      rightPanel.style.transform = 'translateY(-50%) translateX(20px)';
      rightPanel.style.transition = 'all 0.8s ease';
      
      setTimeout(() => {
        rightPanel.style.opacity = '1';
        rightPanel.style.transform = 'translateY(-50%) translateX(0)';
      }, 500);
    }
  }
  
  // 에러 핸들링이 포함된 초기화
  function safeInit() {
    try {
      updateScrollProgress();
      updateEnvironmentalData();
      addTechIconTooltips();
      addProgressNodeClicks();
      addFusionInteraction();
      enhanceDataStream();
      addGridInteraction();
      initFadeIn();
    } catch (error) {
      console.warn('Ecosystem initialization warning:', error);
    }
  }
  
  // 이벤트 리스너 등록
  window.addEventListener('scroll', handleScroll);
  
  // 데이터 업데이트 인터벌
  setInterval(() => {
    try {
      updateEnvironmentalData();
    } catch (error) {
      console.warn('Data update warning:', error);
    }
  }, 4000);
  
  // 창 크기 변경 시 업데이트
  window.addEventListener('resize', function() {
    try {
      updateScrollProgress();
    } catch (error) {
      console.warn('Resize handler warning:', error);
    }
  });
  
  // 초기화 실행
  safeInit();
  
  // 페이지 가시성 변경 시 애니메이션 최적화
  document.addEventListener('visibilitychange', function() {
    if (document.hidden) {
      // 페이지가 숨겨졌을 때 애니메이션 일시 정지
      document.querySelectorAll('.orbital-ring, .data-packet').forEach(el => {
        el.style.animationPlayState = 'paused';
      });
    } else {
      // 페이지가 다시 보일 때 애니메이션 재시작
      document.querySelectorAll('.orbital-ring, .data-packet').forEach(el => {
        el.style.animationPlayState = 'running';
      });
    }
  });
});