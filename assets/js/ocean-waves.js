// 3D 오션 웨이브 시스템 (오른쪽→왼쪽 흐름)
class OceanWaves {
  constructor(containerId) {
    console.log('🏗️ OceanWaves 생성자 시작:', containerId);
    
    this.container = document.getElementById(containerId);
    if (!this.container) {
      console.error('❌ 컨테이너를 찾을 수 없음:', containerId);
      return;
    }
    console.log('📦 컨테이너 크기:', this.container.offsetWidth, 'x', this.container.offsetHeight);
    
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / 200, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    this.time = 0;
    this.waves = [];
    
    console.log('⚙️ Three.js 객체들 생성됨');
    
    this.init();
    this.createWaves();
    this.setupLighting();
    this.animate();
    
    console.log('✨ OceanWaves 초기화 완료');
  }
  
  init() {
    console.log('🎬 init() 시작');
    
    // 컨테이너 크기 가져오기
    const containerWidth = this.container.offsetWidth || window.innerWidth;
    const containerHeight = this.container.offsetHeight || 250;
    console.log('📏 렌더러 크기 설정:', containerWidth, 'x', containerHeight);
    
    // 렌더러 설정
    this.renderer.setSize(containerWidth, containerHeight);
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    
    // 컨테이너에 추가
    this.container.appendChild(this.renderer.domElement);
    console.log('🖼️ 캔버스를 컨테이너에 추가함');
    
    // 카메라 설정 (컨테이너 비율에 맞게)
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.camera.position.set(0, 15, 25);
    this.camera.lookAt(0, 0, 0);
    console.log('📹 카메라 설정 완료');
  }
  
  createWaves() {
    console.log('🌊 createWaves() 시작');
    
    // 여러 파도들을 만들어서 오른쪽에서 왼쪽으로 흐르게 하기
    for (let i = 0; i < 3; i++) {
      console.log(`🌊 파도 ${i + 1} 생성 중...`);
      
      const waveGeometry = new THREE.PlaneGeometry(300, 150, 128, 64);
      const waveMaterial = new THREE.MeshLambertMaterial({
        color: i === 0 ? 0x0066cc : (i === 1 ? 0x1e88e5 : 0x42a5f5),
        transparent: true,
        opacity: 0.7 + (i * 0.1),
        side: THREE.DoubleSide,
        wireframe: false
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        300 + (i * 200),  // 오른쪽 밖에서 시작
        -2 + (i * 1),     // 더 뚜렷한 높이 차이
        0
      );
      
      // 파도 정보 저장
      wave.userData = {
        speed: 1.0 + (i * 0.2),      // 더 빠른 속도
        originalX: wave.position.x,   
        waveOffset: i * Math.PI * 0.7 
      };
      
      this.scene.add(wave);
      this.waves.push(wave);
      console.log(`✅ 파도 ${i + 1} 추가됨 - 위치: (${wave.position.x}, ${wave.position.y}, ${wave.position.z})`);
    }
    console.log(`🌊 총 ${this.waves.length}개 파도 생성 완료`);
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
    
    // 모든 파도들을 애니메이션
    this.waves.forEach((wave) => {
      // 오른쪽에서 왼쪽으로 이동
      wave.position.x -= wave.userData.speed;
      
      // 화면 왼쪽 끝을 벗어나면 오른쪽으로 되돌리기
      if (wave.position.x < -300) {
        wave.position.x = 400;
      }
      
      // 파도 높낮이 애니메이션 (더 역동적으로)
      const positions = wave.geometry.attributes.position;
      for (let i = 0; i < positions.count; i++) {
        const x = positions.getX(i);
        const z = positions.getZ(i);
        
        // 여러 파도가 겹치는 효과 (더 크고 뚜렷하게)
        const waveHeight = 
          Math.sin((x + wave.position.x) * 0.015 + this.time + wave.userData.waveOffset) * 8 +
          Math.cos((z + wave.position.z) * 0.02 + this.time * 1.5) * 5 +
          Math.sin((x + z) * 0.012 + this.time * 1.2) * 3;
          
        positions.setY(i, waveHeight);
      }
      positions.needsUpdate = true;
      
      // 파도 전체적으로 약간 회전 (더 생동감 있게)
      wave.rotation.z = Math.sin(this.time * 0.3 + wave.userData.waveOffset) * 0.05;
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
  console.log('🌊 initOceanWaves 함수 실행됨');
  
  // THREE.js 확인
  if (typeof THREE === 'undefined') {
    console.error('❌ THREE.js가 로드되지 않음');
    return;
  }
  console.log('✅ THREE.js 로드됨');
  
  // 컨테이너 확인
  const container = document.getElementById('ocean-wave-container');
  if (!container) {
    console.error('❌ ocean-wave-container 요소를 찾을 수 없음');
    return;
  }
  console.log('✅ 컨테이너 찾음:', container);
  
  try {
    const oceanWaves = new OceanWaves('ocean-wave-container');
    console.log('✅ OceanWaves 인스턴스 생성됨');
    
    // 리사이즈 이벤트 리스너
    window.addEventListener('resize', () => {
      oceanWaves.onWindowResize();
    });
  } catch (error) {
    console.error('❌ OceanWaves 생성 중 오류:', error);
  }
}

// DOM 로드 후 초기화
document.addEventListener('DOMContentLoaded', () => {
  console.log('🚀 DOM 로드 완료');
  initOceanWaves();
});