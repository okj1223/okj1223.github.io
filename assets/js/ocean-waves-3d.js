// 3D Ocean Waves Implementation
// Based on WellRipple3D component

(function() {
  'use strict';

  // Configuration
  const config = {
    WORLD_X: 36,  // 2배로 증가 (18 -> 36)
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

    // Renderer setup with enhanced properties
    renderer = new THREE.WebGLRenderer({ 
      antialias: true, 
      alpha: true,
      powerPreference: "high-performance"
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(width, height);
    renderer.setClearColor(0x000000, 0);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.2;
    container.appendChild(renderer.domElement);

    // Camera setup - closer for better wave visibility
    camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 50);
    camera.position.set(0, 8, 12);
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
    // Create unified water body using BoxGeometry with animated vertices
    const segments = 120;
    const geometry = new THREE.BoxGeometry(config.WORLD_X, 3, config.WORLD_Z, segments, 20, segments);
    
    // Create unified water material
    const material = new THREE.MeshPhongMaterial({
      color: 0x001f3f,  // Same as background
      transparent: true,
      opacity: 0.92,
      side: THREE.DoubleSide,
      shininess: 200,
      specular: 0xaaccff,
      emissive: 0x001122,
      emissiveIntensity: 0.1,
      flatShading: false,
      vertexColors: false
    });

    waterMesh = new THREE.Mesh(geometry, material);
    waterMesh.position.y = config.WATER_Y - 1.5; // Center the box at water level
    scene.add(waterMesh);

    // Store initial vertices for wave animation (only animate top face)
    waterMesh.userData.originalVertices = [];
    const positions = geometry.attributes.position.array;
    
    // Identify and store only top surface vertices for animation
    for (let i = 0; i < positions.length; i += 3) {
      const x = positions[i];
      const y = positions[i + 1];
      const z = positions[i + 2];
      
      waterMesh.userData.originalVertices.push({
        x: x,
        y: y,
        z: z,
        isTopSurface: Math.abs(y - 1.5) < 0.1 // Top face vertices
      });
    }

    // Initialize wave data - thicker, more voluminous waves
    for (let i = 0; i < 8; i++) {
      waveData.push({
        speedX: (Math.random() - 0.5) * 4,  
        speedZ: (Math.random() - 0.5) * 3,  
        amplitude: 0.2 + Math.random() * 0.4,  // Bigger amplitudes for thickness
        frequencyX: 0.08 + Math.random() * 0.3,  // Lower frequency for broader waves
        frequencyZ: 0.08 + Math.random() * 0.3,
        offsetX: Math.random() * Math.PI * 2,
        offsetZ: Math.random() * Math.PI * 2,
        directionAngle: Math.random() * Math.PI * 2,
        phase: Math.random() * Math.PI * 2,
        thickness: 0.5 + Math.random() * 0.5  // Add thickness parameter
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

    // Apply wave effects only to top surface vertices
    for (let i = 0; i < originalVerts.length; i++) {
      const vert = originalVerts[i];
      
      // Skip if not a top surface vertex
      if (!vert.isTopSurface) {
        continue;
      }
      
      let height = 0;
      let offsetX = 0;
      let offsetZ = 0;

      // Multiple wave layers with thickness effect
      waveData.forEach((wave, index) => {
        // Wave movement with phase offset for more randomness
        const waveX = vert.x * wave.frequencyX + currentTime * wave.speedX + wave.offsetX + wave.phase;
        const waveZ = vert.z * wave.frequencyZ + currentTime * wave.speedZ + wave.offsetZ;
        
        // Create thicker waves using multiple sine waves
        let wavePattern;
        if (index % 3 === 0) {
          // Thick wave with gaussian-like profile
          const base = Math.sin(waveX) * Math.cos(waveZ);
          const thick1 = Math.sin(waveX * 1.5) * Math.cos(waveZ * 1.5) * 0.3;
          const thick2 = Math.sin(waveX * 2) * Math.cos(waveZ * 2) * 0.1;
          wavePattern = (base + thick1 + thick2) * wave.amplitude * wave.thickness;
        } else if (index % 3 === 1) {
          // Rolling thick wave
          const base = Math.sin(waveX + waveZ);
          const thick = Math.sin((waveX + waveZ) * 1.3) * 0.4;
          wavePattern = (base + thick) * wave.amplitude * wave.thickness;
        } else {
          // Broad wave crest
          const base = (Math.sin(waveX) + Math.sin(waveZ)) * 0.5;
          const thick = Math.pow(Math.max(0, base), 0.7); // Power function for broader crests
          wavePattern = thick * wave.amplitude * wave.thickness;
        }
        
        // Add volume with smoother transitions
        const volumeWave = Math.sin(
          vert.x * Math.cos(wave.directionAngle) * 0.5 + 
          vert.z * Math.sin(wave.directionAngle) * 0.5 + 
          currentTime * (1.2 + index * 0.2)
        ) * wave.amplitude * wave.thickness * 0.4;
        
        height += wavePattern + volumeWave;
        
        // Add varying horizontal movement
        offsetX += Math.sin(waveZ + wave.phase) * 0.008;
        offsetZ += Math.cos(waveX - wave.phase) * 0.008;
      });
      
      // Add random noise for natural irregularity
      const noise = (Math.sin(vert.x * 7.3 + currentTime * 2.1) * 
                    Math.cos(vert.z * 5.7 - currentTime * 1.8) +
                    Math.sin(vert.x * 13.1 - vert.z * 11.3 + currentTime * 3.3)) * 0.02;
      height += noise;
      
      // Apply position changes only to top surface
      const idx = i * 3;
      positions[idx] = vert.x + offsetX;     // X position
      positions[idx + 1] = vert.y + height; // Y position (height) - keep original Y and add wave height
      positions[idx + 2] = vert.z + offsetZ; // Z position
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