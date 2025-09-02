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
      const waveGeometry = new THREE.PlaneGeometry(20, 10, 16, 8);
      const waveMaterial = new THREE.MeshBasicMaterial({
        color: i === 0 ? 0xff0000 : (i === 1 ? 0x00ff00 : (i === 2 ? 0x0000ff : (i === 3 ? 0xffff00 : 0xff00ff))), // 빨강,초록,파랑,노랑,보라
        transparent: false, // 일단 투명도 끄기
        opacity: 1.0,
        side: THREE.DoubleSide,
        wireframe: false
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        i * 15,  // 더 멀리 떨어뜨리기
        i * 2,   // Y축도 다르게
        i * 3    // Z축도 다르게  
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
      // 각자 다른 속도로 왼쪽으로 이동
      wave.position.x -= wave.userData.speed * 0.3;
      
      // 화면 왼쪽 끝을 벗어나면 오른쪽으로 되돌리기
      if (wave.position.x < -80) {
        wave.position.x = 80;
      }
      
      console.log(`파도 ${index} 위치: x=${wave.position.x.toFixed(1)}, y=${wave.position.y}, z=${wave.position.z}`);
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