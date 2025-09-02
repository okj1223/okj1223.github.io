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
    
    // 카메라 설정 (컨테이너 비율에 맞게)
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.camera.position.set(0, 30, 50);
    this.camera.lookAt(0, 0, 0);
  }
  
  createWaves() {
    // 메인 바다 표면 생성
    const oceanGeometry = new THREE.PlaneGeometry(200, 100, 128, 64);
    const oceanMaterial = new THREE.MeshPhongMaterial({
      color: 0x006994,
      transparent: true,
      opacity: 0.8,
      side: THREE.DoubleSide,
      shininess: 100,
      specular: 0x004466
    });
    
    this.ocean = new THREE.Mesh(oceanGeometry, oceanMaterial);
    this.ocean.rotation.x = -Math.PI / 2;
    this.ocean.position.set(0, 0, 0);
    this.scene.add(this.ocean);
    this.waves.push(this.ocean);
    
    // 여러 레이어의 파도 추가 (깊이감을 위해)
    for (let i = 0; i < 3; i++) {
      const layerGeometry = new THREE.PlaneGeometry(180 - i * 20, 90 - i * 10, 64, 32);
      const layerMaterial = new THREE.MeshPhongMaterial({
        color: new THREE.Color(0.0 + i * 0.05, 0.4 + i * 0.05, 0.6 + i * 0.1),
        transparent: true,
        opacity: 0.6 - i * 0.15,
        side: THREE.DoubleSide
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
    
    // 모든 파도들을 자연스럽게 애니메이션
    this.waves.forEach((wave, waveIndex) => {
      const positions = wave.geometry.attributes.position;
      
      for (let i = 0; i < positions.count; i++) {
        const x = positions.getX(i);
        const z = positions.getZ(i);
        
        // 자연스러운 파도 높이 계산
        let waveHeight = 0;
        
        // 여러 주파수의 파도를 겹쳐서 자연스럽게
        waveHeight += Math.sin(x * 0.05 + this.time * 2) * 2; // 큰 파도
        waveHeight += Math.sin(x * 0.1 + z * 0.05 + this.time * 3) * 1.5; // 중간 파도  
        waveHeight += Math.cos(x * 0.15 + z * 0.1 + this.time * 4) * 0.8; // 작은 파도
        waveHeight += Math.sin(z * 0.08 + this.time * 1.5) * 1.2; // 측면 파도
        
        // 레이어별로 다른 진폭 적용
        if (wave.userData && wave.userData.amplitude) {
          waveHeight *= wave.userData.amplitude / 3;
          waveHeight += Math.sin(x * 0.02 + this.time + wave.userData.waveOffset) * wave.userData.amplitude;
        }
        
        positions.setY(i, waveHeight);
      }
      
      positions.needsUpdate = true;
      
      // 전체적으로 약간의 움직임 추가 (오른쪽에서 왼쪽으로)
      if (wave.userData && wave.userData.speed) {
        wave.material.map && (wave.material.map.offset.x -= wave.userData.speed * 0.001);
      }
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