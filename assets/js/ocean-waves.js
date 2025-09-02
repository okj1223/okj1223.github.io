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
    this.camera.position.set(0, 20, 30); // 더 가까이
    this.camera.lookAt(0, 0, 0);
    console.log('카메라 위치:', this.camera.position); // 디버깅용
  }
  
  createWaves() {
    // 메인 바다 표면 생성 (디버깅용 빨간색) - 작게
    const oceanGeometry = new THREE.PlaneGeometry(50, 25, 32, 16);
    const oceanMaterial = new THREE.MeshBasicMaterial({
      color: 0xff0000, // 디버깅용 빨간색
      transparent: false,
      opacity: 1.0,
      side: THREE.DoubleSide,
      wireframe: false
    });
    
    this.ocean = new THREE.Mesh(oceanGeometry, oceanMaterial);
    this.ocean.rotation.x = -Math.PI / 2;
    this.ocean.position.set(0, 0, 0);
    this.scene.add(this.ocean);
    this.waves.push(this.ocean);
    
    // 여러 레이어의 파도 추가 (디버깅용 확실한 색상)
    for (let i = 0; i < 3; i++) {
      const layerGeometry = new THREE.PlaneGeometry(40 - i * 5, 20 - i * 2, 32, 16);
      const layerMaterial = new THREE.MeshBasicMaterial({
        color: i === 0 ? 0x00ff00 : (i === 1 ? 0x0000ff : 0xffff00), // 초록, 파랑, 노랑
        transparent: false,
        opacity: 1.0,
        side: THREE.DoubleSide,
        wireframe: false
      });
      
      const layer = new THREE.Mesh(layerGeometry, layerMaterial);
      layer.rotation.x = -Math.PI / 2;
      layer.position.set(0, -1 - i * 0.5, 0);
      
      layer.userData = {
        speed: 0.3 + i * 0.1,
        waveOffset: i * Math.PI / 3,
        amplitude: 3 - i * 0.5
      };
      
      this.scene.add(layer);
      this.waves.push(layer);
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
    
    // 단순한 회전으로 일단 보이는지 확인
    this.waves.forEach((wave, index) => {
      wave.rotation.z = Math.sin(this.time + index) * 0.1;
      console.log(`파도 ${index} 회전:`, wave.rotation.z);
    });
    
    this.renderer.render(this.scene, this.camera);
    console.log('렌더링 중...');
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