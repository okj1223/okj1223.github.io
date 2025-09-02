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
    // 여러 개의 파도를 만들어서 오른쪽에서 왼쪽으로 흐르게 하기
    for (let i = 0; i < 5; i++) {
      const waveGeometry = new THREE.PlaneGeometry(60, 30, 64, 32);
      const waveMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(0.0, 0.4 + i * 0.1, 0.8 + i * 0.05), // 점점 밝은 파랑
        transparent: true,
        opacity: 0.7,
        side: THREE.DoubleSide,
        wireframe: false
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        100 + i * 80,  // 오른쪽 멀리서 시작
        0,
        0
      );
      
      wave.userData = {
        speed: 1.0 + i * 0.2,      // 다른 속도로 흐르기
        waveOffset: i * Math.PI / 4,
        originalX: wave.position.x
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
    this.time += 0.016; // 시간 증가
    
    // 파도들을 오른쪽에서 왼쪽으로 흐르게 하기
    this.waves.forEach((wave, index) => {
      // 왼쪽으로 이동
      wave.position.x -= wave.userData.speed;
      
      // 화면 왼쪽 끝을 벗어나면 오른쪽으로 되돌리기
      if (wave.position.x < -150) {
        wave.position.x = 400;
      }
      
      // 파도 높낮이 애니메이션
      const positions = wave.geometry.attributes.position;
      for (let i = 0; i < positions.count; i++) {
        const x = positions.getX(i);
        const z = positions.getZ(i);
        
        // 파도 높이 계산
        const waveHeight = 
          Math.sin((x + wave.position.x) * 0.02 + this.time + wave.userData.waveOffset) * 4 +
          Math.cos((z) * 0.03 + this.time * 1.5) * 2;
          
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
  // THREE.js 확인
  if (typeof THREE === 'undefined') {
    console.error('THREE.js가 로드되지 않음');
    return;
  }
  
  // 컨테이너 확인
  const container = document.getElementById('ocean-wave-container');
  if (!container) {
    console.error('ocean-wave-container 요소를 찾을 수 없음');
    return;
  }
  
  try {
    const oceanWaves = new OceanWaves('ocean-wave-container');
    
    // 리사이즈 이벤트 리스너
    window.addEventListener('resize', () => {
      oceanWaves.onWindowResize();
    });
  } catch (error) {
    console.error('OceanWaves 생성 중 오류:', error);
  }
}

// DOM 로드 후 초기화
document.addEventListener('DOMContentLoaded', initOceanWaves);