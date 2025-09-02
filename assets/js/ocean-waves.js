// 3D 오션 웨이브 시스템
class OceanWaves {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / 200, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    
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
    // 첫 번째 파도 레이어
    const wave1Geometry = new THREE.PlaneGeometry(100, 50, 100, 50);
    const wave1Material = new THREE.MeshLambertMaterial({
      color: 0x0078ff,
      transparent: true,
      opacity: 0.8,
      side: THREE.DoubleSide
    });
    this.wave1 = new THREE.Mesh(wave1Geometry, wave1Material);
    this.wave1.rotation.x = -Math.PI / 2;
    this.wave1.position.y = -2;
    this.scene.add(this.wave1);
    
    // 두 번째 파도 레이어
    const wave2Geometry = new THREE.PlaneGeometry(80, 40, 80, 40);
    const wave2Material = new THREE.MeshLambertMaterial({
      color: 0x1e88e5,
      transparent: true,
      opacity: 0.6,
      side: THREE.DoubleSide
    });
    this.wave2 = new THREE.Mesh(wave2Geometry, wave2Material);
    this.wave2.rotation.x = -Math.PI / 2;
    this.wave2.position.y = -1;
    this.scene.add(this.wave2);
    
    // 세 번째 파도 레이어
    const wave3Geometry = new THREE.PlaneGeometry(120, 60, 120, 60);
    const wave3Material = new THREE.MeshLambertMaterial({
      color: 0x42a5f5,
      transparent: true,
      opacity: 0.4,
      side: THREE.DoubleSide
    });
    this.wave3 = new THREE.Mesh(wave3Geometry, wave3Material);
    this.wave3.rotation.x = -Math.PI / 2;
    this.wave3.position.y = -3;
    this.scene.add(this.wave3);
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
    const time = Date.now() * 0.001;
    
    // 첫 번째 파도 애니메이션
    if (this.wave1) {
      const positions1 = this.wave1.geometry.attributes.position;
      for (let i = 0; i < positions1.count; i++) {
        const x = positions1.getX(i);
        const z = positions1.getZ(i);
        positions1.setY(i, Math.sin(x * 0.1 + time) * 2 + Math.cos(z * 0.1 + time) * 1);
      }
      positions1.needsUpdate = true;
      this.wave1.rotation.z = Math.sin(time * 0.5) * 0.1;
    }
    
    // 두 번째 파도 애니메이션
    if (this.wave2) {
      const positions2 = this.wave2.geometry.attributes.position;
      for (let i = 0; i < positions2.count; i++) {
        const x = positions2.getX(i);
        const z = positions2.getZ(i);
        positions2.setY(i, Math.sin(x * 0.15 + time * 1.2) * 1.5 + Math.cos(z * 0.15 + time * 1.2) * 0.8);
      }
      positions2.needsUpdate = true;
      this.wave2.rotation.z = Math.cos(time * 0.7) * 0.08;
    }
    
    // 세 번째 파도 애니메이션
    if (this.wave3) {
      const positions3 = this.wave3.geometry.attributes.position;
      for (let i = 0; i < positions3.count; i++) {
        const x = positions3.getX(i);
        const z = positions3.getZ(i);
        positions3.setY(i, Math.sin(x * 0.08 + time * 0.8) * 2.5 + Math.cos(z * 0.08 + time * 0.8) * 1.2);
      }
      positions3.needsUpdate = true;
      this.wave3.rotation.z = Math.sin(time * 0.3) * 0.12;
    }
    
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