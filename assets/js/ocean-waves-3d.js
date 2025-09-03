// 3D Ocean Waves Implementation
// Based on WellRipple3D component

(function() {
  'use strict';

  // Configuration
  const config = {
    WORLD_X: 18,
    WORLD_Z: 10,
    WATER_Y: 0,
    waterAlpha: 0.85,
    waveSpeed: 2,
    waveHeight: 0.5,
    waveFrequency: 0.5
  };

  let scene, renderer, camera, waterMesh;
  let raycaster = new THREE.Raycaster();
  let clickPoint = new THREE.Vector3();
  let waterPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  
  // Wave parameters
  let waveData = [];
  
  // Animation state
  let animationId;
  let clock = new THREE.Clock();

  function init() {
    const container = document.getElementById('ocean-waves-container');
    if (!container) return;

    const width = container.clientWidth || window.innerWidth;
    const height = 400; // Fixed height for footer

    // Scene setup
    scene = new THREE.Scene();

    // Renderer setup
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(width, height);
    renderer.setClearColor(0x000000, 0);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(renderer.domElement);

    // Camera setup
    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 50);
    camera.position.set(0, 12, 15);
    camera.lookAt(0, 0, 0);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
    dirLight.position.set(5, 10, 5);
    scene.add(dirLight);

    // Create water surface
    createWaterSurface();

    // Create splash column pool
    createColumnPool();

    // Start auto-ripple effect
    startAutoRipple();

    // Event listeners
    container.addEventListener('click', onContainerClick);
    window.addEventListener('resize', onWindowResize);

    // Start animation
    animate();
  }

  function createWaterSurface() {
    const segments = 200;
    const geometry = new THREE.PlaneGeometry(config.WORLD_X, config.WORLD_Z, segments, segments);
    
    // Create water material with better ocean color
    const material = new THREE.MeshPhongMaterial({
      color: 0x006994,  // Deep ocean blue
      transparent: true,
      opacity: config.waterAlpha,
      side: THREE.DoubleSide,
      shininess: 100,
      specular: 0x4488ff,
      flatShading: false,
      vertexColors: false
    });

    waterMesh = new THREE.Mesh(geometry, material);
    waterMesh.rotation.x = -Math.PI / 2;
    waterMesh.position.y = config.WATER_Y;
    scene.add(waterMesh);

    // Store initial vertices for wave animation
    waterMesh.userData.originalVertices = [];
    const positions = geometry.attributes.position.array;
    for (let i = 0; i < positions.length; i += 3) {
      waterMesh.userData.originalVertices.push({
        x: positions[i],
        y: positions[i + 1],
        z: positions[i + 2]
      });
    }

    // Initialize wave data
    for (let i = 0; i < 5; i++) {
      waveData.push({
        speed: 1 + Math.random() * 2,
        amplitude: 0.2 + Math.random() * 0.3,
        frequency: 0.5 + Math.random() * 0.5,
        offset: Math.random() * Math.PI * 2
      });
    }
  }

  function createColumnPool() {
    // Removed - no longer using columns for wave effect
  }

  function spawnRipple(position) {
    // Create ripple effect on water mesh when clicked
    createRippleOnWater(position);
  }

  function createRippleOnWater(center) {
    if (!waterMesh || !waterMesh.geometry) return;

    const geometry = waterMesh.geometry;
    const positions = geometry.attributes.position.array;
    const originalVerts = waterMesh.userData.originalVertices;
    
    const rippleData = {
      center: center.clone(),
      startTime: clock.getElapsedTime(),
      amplitude: 0.3,
      speed: 2,
      decay: 1
    };

    // Store ripple data for animation
    if (!waterMesh.userData.ripples) {
      waterMesh.userData.ripples = [];
    }
    waterMesh.userData.ripples.push(rippleData);

    // Clean old ripples
    waterMesh.userData.ripples = waterMesh.userData.ripples.filter(r => {
      const age = clock.getElapsedTime() - r.startTime;
      return age < 3; // Keep ripples for 3 seconds
    });
  }

  function updateWaterSurface() {
    if (!waterMesh) return;

    const geometry = waterMesh.geometry;
    const positions = geometry.attributes.position.array;
    const originalVerts = waterMesh.userData.originalVertices;
    const currentTime = clock.getElapsedTime();

    // Apply wave effect from right to left
    for (let i = 0; i < originalVerts.length; i++) {
      const vert = originalVerts[i];
      let height = 0;

      // Multiple wave layers for more realistic ocean effect
      waveData.forEach((wave, index) => {
        // Primary wave moving from right to left
        const waveX = vert.x - currentTime * wave.speed;
        const waveZ = vert.y * 0.5; // Less variation in Z direction
        
        // Create wave pattern
        const mainWave = Math.sin(waveX * wave.frequency + wave.offset) * wave.amplitude;
        const secondaryWave = Math.sin(waveX * wave.frequency * 2 + waveZ * 0.3) * wave.amplitude * 0.3;
        
        height += mainWave + secondaryWave;
      });

      // Add small random perturbation for natural look
      height += Math.sin(currentTime * 3 + vert.x * 10) * 0.02;
      
      // Apply height to vertex
      positions[i * 3 + 2] = height;
    }

    // Apply click ripples if any
    if (waterMesh.userData.ripples) {
      waterMesh.userData.ripples.forEach(ripple => {
        const age = currentTime - ripple.startTime;
        const radius = age * ripple.speed;
        const amplitude = ripple.amplitude * Math.exp(-age * ripple.decay);

        for (let i = 0; i < originalVerts.length; i++) {
          const vert = originalVerts[i];
          const dx = vert.x - ripple.center.x;
          const dz = vert.y - ripple.center.z;
          const distance = Math.sqrt(dx * dx + dz * dz);

          if (distance < radius + 1) {
            const wave = Math.sin((radius - distance) * 2) * amplitude;
            const falloff = Math.max(0, 1 - distance / (radius + 1));
            positions[i * 3 + 2] += wave * falloff;
          }
        }
      });
    }

    geometry.attributes.position.needsUpdate = true;
    geometry.computeVertexNormals();
  }

  function updateColumns(deltaTime) {
    // Removed - no longer using columns
  }

  function startAutoRipple() {
    // Waves are now continuous, no need for auto-ripple
  }

  function onContainerClick(event) {
    const container = document.getElementById('ocean-waves-container');
    if (!container) return;

    const rect = container.getBoundingClientRect();
    const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(new THREE.Vector2(x, y), camera);
    
    if (raycaster.ray.intersectPlane(waterPlane, clickPoint)) {
      clickPoint.x = THREE.MathUtils.clamp(clickPoint.x, -config.WORLD_X/2, config.WORLD_X/2);
      clickPoint.z = THREE.MathUtils.clamp(clickPoint.z, -config.WORLD_Z/2, config.WORLD_Z/2);
      clickPoint.y = config.WATER_Y;
      spawnRipple(clickPoint.clone());
    }
  }

  function onWindowResize() {
    const container = document.getElementById('ocean-waves-container');
    if (!container) return;

    const width = container.clientWidth;
    const height = 400;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }

  function animate() {
    animationId = requestAnimationFrame(animate);

    const deltaTime = clock.getDelta();

    // Update water surface waves
    updateWaterSurface();

    // Removed column update

    // Render
    renderer.render(scene, camera);
  }

  function cleanup() {
    if (animationId) {
      cancelAnimationFrame(animationId);
    }

    const container = document.getElementById('ocean-waves-container');
    if (container) {
      container.removeEventListener('click', onContainerClick);
      if (renderer && renderer.domElement) {
        container.removeChild(renderer.domElement);
      }
    }

    window.removeEventListener('resize', onWindowResize);

    // Dispose Three.js resources
    if (scene) {
      scene.traverse((child) => {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
          if (Array.isArray(child.material)) {
            child.material.forEach(m => m.dispose());
          } else {
            child.material.dispose();
          }
        }
      });
    }

    if (renderer) {
      renderer.dispose();
    }
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

  // Cleanup on page unload
  window.addEventListener('beforeunload', cleanup);

})();