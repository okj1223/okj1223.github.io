// Ocean Waves V2 - BLUE VERSION
class OceanWavesV2 {
  constructor(containerId) {
    console.log('🌊 OceanWavesV2 Starting - BLUE VERSION');
    
    this.container = document.getElementById(containerId);
    if (!this.container) {
      console.error('Container not found:', containerId);
      return;
    }
    
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / 200, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    this.time = 0;
    this.waves = [];
    
    this.init();
    this.createBlueOceanWaves();
    this.setupLighting();
    this.animate();
  }
  
  init() {
    const containerWidth = this.container.offsetWidth || window.innerWidth;
    const containerHeight = this.container.offsetHeight || 250;
    
    this.renderer.setSize(containerWidth, containerHeight);
    this.renderer.setClearColor(0x000000, 0);
    this.container.appendChild(this.renderer.domElement);
    
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.camera.position.set(0, 15, 25);
    this.camera.lookAt(0, 0, 0);
  }
  
  createBlueOceanWaves() {
    console.log('Creating BLUE ocean waves...');
    
    // BLUE COLORS ONLY!
    const oceanBlues = [
      '#0066CC', '#0077DD', '#0088EE', '#0099FF', '#00AAFF', '#00BBFF'
    ];
    
    for (let i = 0; i < 6; i++) {
      const waveGeometry = new THREE.PlaneGeometry(40, 20, 32, 16);
      
      // USE BLUE COLOR - NO RED!
      const blueColor = oceanBlues[i];
      console.log(`Wave ${i} using BLUE color: ${blueColor}`);
      
      const waveMaterial = new THREE.MeshStandardMaterial({
        color: blueColor,
        transparent: true,
        opacity: 0.85 - i * 0.1,
        side: THREE.DoubleSide,
        roughness: 0.3,
        metalness: 0.1
      });
      
      const wave = new THREE.Mesh(waveGeometry, waveMaterial);
      wave.rotation.x = -Math.PI / 2;
      wave.position.set(
        (i - 3) * 8,
        -i * 0.5,
        -i * 2
      );
      
      wave.userData = {
        speed: 0.8 + i * 0.15,
        waveOffset: i * Math.PI / 3,
        amplitude: 4 - i * 0.3,
        frequency: 0.02 + i * 0.005
      };
      
      this.scene.add(wave);
      this.waves.push(wave);
      
      // Verify it's really blue
      console.log(`Wave ${i} final color:`, waveMaterial.color.getHexString());
    }
    
    console.log('✅ All waves created with BLUE colors');
  }
  
  setupLighting() {
    const ambientLight = new THREE.AmbientLight(0x4d7ea8, 0.5);
    this.scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
    directionalLight.position.set(30, 50, 20);
    this.scene.add(directionalLight);
    
    const pointLight = new THREE.PointLight(0x87ceeb, 0.5, 100);
    pointLight.position.set(0, 20, 0);
    this.scene.add(pointLight);
  }
  
  animate() {
    this.time += 0.02;
    
    this.waves.forEach((wave, waveIndex) => {
      wave.position.x = wave.position.x + Math.sin(this.time + waveIndex) * 0.1;
      wave.position.z = wave.position.z + Math.cos(this.time * 0.8 + waveIndex) * 0.05;
      
      const positions = wave.geometry.attributes.position;
      for (let i = 0; i < positions.count; i++) {
        const x = positions.getX(i);
        const z = positions.getZ(i);
        
        let waveHeight = 0;
        waveHeight += Math.sin((x + wave.position.x) * wave.userData.frequency + this.time * 2 + wave.userData.waveOffset) * wave.userData.amplitude;
        waveHeight += Math.sin((x * 0.05 + z * 0.03) + this.time * 3) * (wave.userData.amplitude * 0.3);
        waveHeight += Math.cos((z * 0.08) + this.time * 1.5) * (wave.userData.amplitude * 0.2);
        waveHeight += Math.sin((z * 0.04) + this.time * 2.5 + wave.userData.waveOffset) * (wave.userData.amplitude * 0.4);
        
        positions.setY(i, waveHeight);
      }
      
      positions.needsUpdate = true;
    });
    
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(() => this.animate());
  }
  
  onWindowResize() {
    const containerWidth = this.container.offsetWidth || window.innerWidth;
    const containerHeight = this.container.offsetHeight || 250;
    
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(containerWidth, containerHeight);
  }
}

// Initialize V2
function initOceanWavesV2() {
  console.log('🌊 Initializing OceanWaves V2 - BLUE VERSION!');
  
  if (typeof THREE === 'undefined') {
    console.error('THREE.js not loaded');
    return;
  }
  
  const container = document.getElementById('ocean-wave-container');
  if (!container) {
    console.error('ocean-wave-container not found');
    return;
  }
  
  try {
    const oceanWaves = new OceanWavesV2('ocean-wave-container');
    console.log('✅ OceanWavesV2 created successfully!');
    
    window.addEventListener('resize', () => {
      oceanWaves.onWindowResize();
    });
  } catch (error) {
    console.error('Error creating OceanWavesV2:', error);
  }
}

// DOM loaded
document.addEventListener('DOMContentLoaded', initOceanWavesV2);