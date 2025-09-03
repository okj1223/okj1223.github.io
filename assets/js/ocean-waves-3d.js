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

    // Lights - enhanced for better water reflection
    const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(5, 10, 5);
    scene.add(dirLight);

    // Add point light for water highlights
    const pointLight = new THREE.PointLight(0x88ccff, 0.5);
    pointLight.position.set(0, 5, 0);
    scene.add(pointLight);

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
    // Create water base (static water body)
    const baseGeometry = new THREE.BoxGeometry(config.WORLD_X, 2, config.WORLD_Z);
    const baseMaterial = new THREE.MeshPhongMaterial({
      color: 0x004466,  // Darker ocean blue for depth
      transparent: true,
      opacity: 0.9,
      shininess: 80
    });
    
    const waterBase = new THREE.Mesh(baseGeometry, baseMaterial);
    waterBase.position.y = -1;
    scene.add(waterBase);

    // Create wave surface on top
    const segments = 80;
    const geometry = new THREE.PlaneGeometry(config.WORLD_X, config.WORLD_Z, segments, segments);
    
    // Create water material with better ocean color
    const material = new THREE.MeshPhongMaterial({
      color: 0x0077aa,  // Ocean blue
      transparent: true,
      opacity: 0.95,
      side: THREE.DoubleSide,
      shininess: 150,
      specular: 0x88ccff,
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

    // Initialize wave data - multiple waves with random directions
    for (let i = 0; i < 5; i++) {
      waveData.push({
        speedX: (Math.random() - 0.5) * 3,  // Random X direction speed
        speedZ: (Math.random() - 0.5) * 2,  // Random Z direction speed
        amplitude: 0.3 + Math.random() * 0.4,
        frequencyX: 0.2 + Math.random() * 0.3,
        frequencyZ: 0.2 + Math.random() * 0.3,
        offsetX: Math.random() * Math.PI * 2,
        offsetZ: Math.random() * Math.PI * 2,
        directionAngle: Math.random() * Math.PI * 2  // Random wave direction
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

    // Apply wave effects with random XYZ movement
    for (let i = 0; i < originalVerts.length; i++) {
      const vert = originalVerts[i];
      let height = 0;
      let offsetX = 0;
      let offsetZ = 0;

      // Multiple wave layers with different directions
      waveData.forEach((wave, index) => {
        // Wave movement in X and Z directions
        const waveX = vert.x * wave.frequencyX + currentTime * wave.speedX + wave.offsetX;
        const waveZ = vert.y * wave.frequencyZ + currentTime * wave.speedZ + wave.offsetZ;
        
        // Combine waves from different directions
        const wavePattern = Math.sin(waveX) * Math.cos(waveZ) * wave.amplitude;
        
        // Add diagonal wave movement
        const diagonalWave = Math.sin(
          vert.x * Math.cos(wave.directionAngle) + 
          vert.y * Math.sin(wave.directionAngle) + 
          currentTime * 2
        ) * wave.amplitude * 0.5;
        
        height += wavePattern + diagonalWave;
        
        // Add small horizontal movement for more dynamic effect
        offsetX += Math.sin(waveZ) * 0.02;
        offsetZ += Math.cos(waveX) * 0.02;
      });
      
      // Add turbulence for more random movement
      const turbulence = Math.sin(currentTime * 4 + vert.x * 5) * 
                        Math.cos(currentTime * 3 + vert.y * 5) * 0.05;
      height += turbulence;
      
      // Apply position changes
      const idx = i * 3;
      positions[idx] = originalVerts[i].x + offsetX;     // X position
      positions[idx + 1] = originalVerts[i].y + offsetZ; // Z position (Y in plane)
      positions[idx + 2] = height;                       // Y position (height)
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