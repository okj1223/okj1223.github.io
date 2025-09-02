// 3D 오션 웨이브 시스템
class OceanWaves {
  constructor(containerId) {
    console.log('🏗️ OceanWaves 생성자 시작');
    
    this.container = document.getElementById(containerId);
    if (!this.container) {
      console.error('컨테이너를 찾을 수 없음:', containerId);
      return;
    }
    console.log('📦 컨테이너 확인됨');
    
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / 200, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    this.time = 0;
    this.waves = [];
    console.log('⚙️ Three.js 객체들 생성됨');
    
    this.init();
    console.log('🎬 init() 완료');
    
    this.createWaves();
    console.log('🌊 createWaves() 완료');
    
    this.setupLighting();
    console.log('💡 setupLighting() 완료');
    
    this.animate();
    console.log('🎥 animate() 시작');
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
    this.camera.position.set(0, 15, 25);
    this.camera.lookAt(0, 0, 0);
  }
  
  createWaves() {
    // 진짜 바다 파도 만들기 (페이지 2배 크기 고려)
    for (let i = 0; i < 6; i++) {
      // 작은 테스트 파도
      const waveGeometry = new THREE.PlaneGeometry(40, 20, 32, 16);
      const waveMaterial = new THREE.MeshPhongMaterial({
        color: new THREE.Color().setHSL(0.58 + i * 0.02, 0.7, 0.5), // HSL로 명확한 파란색
        transparent: true,
        opacity: 0.9 - i * 0.1, // 뒤로 갈수록 투명하게
        side: THREE.DoubleSide,
        wireframe: false,
        shininess: 100,
        specular: new THREE.Color(0x87ceeb), // 하늘색 반사광
        emissive: new THREE.Color(0x002244), // 약간의 자체 발광
        emissiveIntensity: 0.1
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        (i - 3) * 8,  // 화면 중앙에서 퍼져나가기
        -i * 0.5,     // 레이어링 효과
        -i * 2        // 깊이감
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
    // 주변광 - 바다의 푸른빛
    const ambientLight = new THREE.AmbientLight(0x4d7ea8, 0.4);
    this.scene.add(ambientLight);
    
    // 메인 태양광
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
    directionalLight.position.set(30, 50, 20);
    directionalLight.castShadow = true;
    directionalLight.shadow.camera.left = -50;
    directionalLight.shadow.camera.right = 50;
    directionalLight.shadow.camera.top = 50;
    directionalLight.shadow.camera.bottom = -50;
    this.scene.add(directionalLight);
    
    // 보조 조명 - 파도 하이라이트
    const pointLight = new THREE.PointLight(0x87ceeb, 0.5, 100);
    pointLight.position.set(0, 20, 0);
    this.scene.add(pointLight);
  }
  
  animate() {
    this.time += 0.02; // 시간 증가 (조금 더 빠르게)
    
    // 진짜 바다 파도 애니메이션
    this.waves.forEach((wave, waveIndex) => {
      // 파도가 앞뒤로 움직이는 효과 (밀려왔다 빠지는 느낌)
      wave.position.x = wave.position.x + Math.sin(this.time + waveIndex) * 0.1;
      wave.position.z = wave.position.z + Math.cos(this.time * 0.8 + waveIndex) * 0.05;
      
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
    console.log('🚀 OceanWaves 생성 시작...');
    const oceanWaves = new OceanWaves('ocean-wave-container');
    console.log('✅ OceanWaves 생성 완료!');
    
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