// 3D 오션 웨이브 시스템 (오른쪽→왼쪽 흐름)
class OceanWaves {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
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
    // 렌더러 설정
    this.renderer.setSize(window.innerWidth, 200);
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    
    // 컨테이너에 추가
    this.container.appendChild(this.renderer.domElement);
    
    // 카메라 위치
    this.camera.position.set(0, 10, 20);
    this.camera.lookAt(0, 0, 0);
  }
  
  createWaves() {
    // 여러 파도들을 만들어서 오른쪽에서 왼쪽으로 흐르게 하기
    for (let i = 0; i < 5; i++) {
      const waveGeometry = new THREE.PlaneGeometry(200, 100, 64, 32);
      const waveMaterial = new THREE.MeshLambertMaterial({
        color: i % 2 === 0 ? 0x0066cc : 0x1e88e5,
        transparent: true,
        opacity: 0.4 + (i * 0.1),
        side: THREE.DoubleSide,
        wireframe: false
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        200 + (i * 150),  // 오른쪽 밖에서 시작
        -5 + (i * 0.5),   // 약간씩 다른 높이
        0
      );
      
      // 파도 정보 저장
      wave.userData = {
        speed: 0.5 + (i * 0.1),      // 다른 속도로 흐르기
        originalX: wave.position.x,   // 원래 위치 기억
        waveOffset: i * Math.PI * 0.5 // 파도 위상 차이
      };
      
      this.scene.add(wave);
      this.waves.push(wave);
    }
    
    // 추가 배경 파도 (더 큰 파도)
    const bigWaveGeometry = new THREE.PlaneGeometry(500, 200, 128, 64);
    const bigWaveMaterial = new THREE.MeshLambertMaterial({
      color: 0x004499,
      transparent: true,
      opacity: 0.3,
      side: THREE.DoubleSide
    });
    
    this.bigWave = new THREE.Mesh(bigWaveGeometry, bigWaveMaterial);
    this.bigWave.rotation.x = -Math.PI / 2;
    this.bigWave.position.set(0, -8, 0);
    this.bigWave.userData = { speed: 0.2, originalX: 0, waveOffset: 0 };
    this.scene.add(this.bigWave);
    this.waves.push(this.bigWave);
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
        
        // 여러 파도가 겹치는 효과
        const waveHeight = 
          Math.sin((x + wave.position.x) * 0.01 + this.time + wave.userData.waveOffset) * 3 +
          Math.cos((z + wave.position.z) * 0.015 + this.time * 1.2) * 2 +
          Math.sin((x + z) * 0.008 + this.time * 0.8) * 1.5;
          
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
    this.camera.aspect = window.innerWidth / 200;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, 200);
  }
}

// 초기화 함수
function initOceanWaves() {
  if (typeof THREE !== 'undefined') {
    const oceanWaves = new OceanWaves('ocean-wave-container');
    
    // 리사이즈 이벤트 리스너
    window.addEventListener('resize', () => {
      oceanWaves.onWindowResize();
    });
  }
}

// DOM 로드 후 초기화
document.addEventListener('DOMContentLoaded', initOceanWaves);