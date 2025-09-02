// 3D 오션 웨이브 시스템
class OceanWaves {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    if (!this.container) {
      console.error('컨테이너를 찾을 수 없음:', containerId);
      return;
    }
    
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / 200, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    this.time = 0;
    this.waves = [];
    
    this.init();
    this.createWaves();
    this.setupLighting();
    this.animate();
  }
  
  init() {
    // 컨테이너 크기 가져오기
    const containerWidth = this.container.offsetWidth || window.innerWidth;
    const containerHeight = this.container.offsetHeight || 250;
    
    // 렌더러 설정
    this.renderer.setSize(containerWidth, containerHeight);
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    
    // 컨테이너에 추가
    this.container.appendChild(this.renderer.domElement);
    
    // 카메라 설정 (컨테이너 비율에 맞게) - 더 가까이
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.camera.position.set(0, 20, 30);
    this.camera.lookAt(0, 0, 0);
  }
  
  createWaves() {
    // 진짜 바다 파도 만들기 (페이지 2배 크기 고려)
    for (let i = 0; i < 6; i++) {
      // 작은 테스트 파도
      const waveGeometry = new THREE.PlaneGeometry(20, 10, 16, 8);
      const waveMaterial = new THREE.MeshBasicMaterial({
        color: 0xff0000, // 일단 빨간색으로 디버깅
        transparent: false,
        opacity: 1.0,
        side: THREE.DoubleSide,
        wireframe: false
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        i * 5,  // 화면 중앙에서 시작
        0,      // 같은 높이
        0
      );
      
      wave.userData = {
        speed: 0.8 + i * 0.15,     // 다른 속도로 흐르기
        waveOffset: i * Math.PI / 3,
        amplitude: 4 - i * 0.3,    // 앞쪽 파도가 더 높게
        frequency: 0.02 + i * 0.005 // 주파수도 다르게
      };
      
      this.scene.add(wave);
      this.waves.push(wave);
    }
  }
  
  setupLighting() {
    // 주변광
    const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
    this.scene.add(ambientLight);
    
    // 방향광
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 5);
    directionalLight.castShadow = true;
    this.scene.add(directionalLight);
  }
  
  animate() {
    this.time += 0.01; // 시간 증가
    
    // 진짜 바다 파도 애니메이션
    this.waves.forEach((wave, waveIndex) => {
      // 오른쪽에서 왼쪽으로 흐르는 움직임
      wave.position.x -= wave.userData.speed;
      
      // 화면 왼쪽 끝을 벗어나면 오른쪽으로 되돌리기 
      if (wave.position.x < -100) {
        wave.position.x = 200;
      }
      
      // 사인파로 파도 출렁이는 효과 만들기
      const positions = wave.geometry.attributes.position;
      for (let i = 0; i < positions.count; i++) {
        const x = positions.getX(i);
        const z = positions.getZ(i);
        
        // 여러 사인파를 겹쳐서 자연스러운 파도 만들기
        let waveHeight = 0;
        
        // 주 파도 (큰 파도)
        waveHeight += Math.sin((x + wave.position.x) * wave.userData.frequency + this.time * 2 + wave.userData.waveOffset) * wave.userData.amplitude;
        
        // 작은 파도들 (디테일)
        waveHeight += Math.sin((x * 0.05 + z * 0.03) + this.time * 3) * (wave.userData.amplitude * 0.3);
        waveHeight += Math.cos((z * 0.08) + this.time * 1.5) * (wave.userData.amplitude * 0.2);
        
        // 측면 파도 (Z축 방향)
        waveHeight += Math.sin((z * 0.04) + this.time * 2.5 + wave.userData.waveOffset) * (wave.userData.amplitude * 0.4);
        
        positions.setY(i, waveHeight);
      }
      
      positions.needsUpdate = true;
    });
    
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(() => this.animate());
  }
  
  // 리사이즈 핸들러
  onWindowResize() {
    const containerWidth = this.container.offsetWidth || window.innerWidth;
    const containerHeight = this.container.offsetHeight || 250;
    
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(containerWidth, containerHeight);
  }
}

// 초기화 함수
function initOceanWaves() {
  console.log('🌊 초기화 시작!');
  
  // THREE.js 확인
  if (typeof THREE === 'undefined') {
    console.error('❌ THREE.js가 로드되지 않음');
    // 컨테이너에 에러 메시지 표시
    const container = document.getElementById('ocean-wave-container');
    if (container) {
      container.innerHTML = '<div style="color: white; text-align: center; padding: 50px;">THREE.js 로드 실패</div>';
    }
    return;
  }
  console.log('✅ THREE.js 로드됨');
  
  // 컨테이너 확인
  const container = document.getElementById('ocean-wave-container');
  if (!container) {
    console.error('❌ ocean-wave-container 요소를 찾을 수 없음');
    return;
  }
  console.log('✅ 컨테이너 찾음:', container.offsetWidth, 'x', container.offsetHeight);
  
  try {
    // 간단한 테스트 캔버스 먼저 만들어보기
    const canvas = document.createElement('canvas');
    canvas.width = container.offsetWidth;
    canvas.height = container.offsetHeight;
    canvas.style.background = 'blue';
    canvas.style.position = 'absolute';
    canvas.style.top = '0';
    canvas.style.left = '0';
    container.appendChild(canvas);
    console.log('✅ 테스트 캔버스 추가됨');
    
    const oceanWaves = new OceanWaves('ocean-wave-container');
    
    // 리사이즈 이벤트 리스너
    window.addEventListener('resize', () => {
      oceanWaves.onWindowResize();
    });
  } catch (error) {
    console.error('❌ OceanWaves 생성 중 오류:', error);
    // 컨테이너에 에러 메시지 표시
    container.innerHTML = '<div style="color: white; text-align: center; padding: 50px;">3D 바다 생성 실패: ' + error.message + '</div>';
  }
}

// DOM 로드 후 초기화
document.addEventListener('DOMContentLoaded', initOceanWaves);