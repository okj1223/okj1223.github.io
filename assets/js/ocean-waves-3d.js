// 3D Ocean Waves Implementation
// Based on WellRipple3D component

(function() {
  'use strict';

  // Configuration
  const config = {
    WORLD_X: 36,  // 2배로 증가 (18 -> 36)
    WORLD_Z: 10,
    WATER_Y: 0,
    waterAlpha: 0.75, // More transparent
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
  
  // Floating objects
  let floatingObjects = [];
  let objectPool = [];
  
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

    // Renderer setup optimized for performance
    renderer = new THREE.WebGLRenderer({ 
      antialias: window.devicePixelRatio <= 1, // Only use antialiasing on low DPI displays
      alpha: true,
      powerPreference: "high-performance"
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5)); // Limit pixel ratio
    renderer.setSize(width, height);
    renderer.setClearColor(0x000000, 0);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    // Remove tone mapping for better performance
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

    // Create floating objects
    createFloatingObjects();

    // Start object spawning
    startObjectSpawning();

    // Event listeners
    container.addEventListener('click', onContainerClick);
    window.addEventListener('resize', onWindowResize);

    // Start animation
    animate();
  }

  function createWaterSurface() {
    // Create unified water body using BoxGeometry with optimized segments
    const segments = 64; // Reduced from 120 to 64 for better performance
    const geometry = new THREE.BoxGeometry(config.WORLD_X, 3, config.WORLD_Z, segments, 12, segments);
    
    // Create unified water material with more transparency
    const material = new THREE.MeshPhongMaterial({
      color: 0x004466,  // Slightly brighter blue for transparency
      transparent: true,
      opacity: 0.75,    // Much more transparent (was 0.92)
      side: THREE.DoubleSide,
      shininess: 250,   // Higher shininess for glass-like effect
      specular: 0xbbddff, // Brighter specular for crystal clear effect
      emissive: 0x001133,
      emissiveIntensity: 0.05, // Reduced emissive for cleaner look
      flatShading: false,
      vertexColors: false,
      alphaTest: 0.1    // Better transparency rendering
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

    // Initialize wave data - reduced count for performance
    for (let i = 0; i < 5; i++) { // Reduced from 8 to 5 waves
      // Main flow direction with some variation
      const baseFlowDirection = Math.PI; // Left direction
      const flowVariation = (Math.random() - 0.5) * Math.PI * 0.4; // ±36 degrees variation
      const flowDirection = baseFlowDirection + flowVariation;
      
      waveData.push({
        speedX: -1 - Math.random() * 1,  // Slower speed: -1 to -2 (was -2 to -4)
        speedZ: (Math.random() - 0.5) * 1,  // Reduced Z variation: ±0.5 (was ±1)
        amplitude: 0.15 + Math.random() * 0.25,  // Slightly reduced amplitude
        frequencyX: 0.08 + Math.random() * 0.3,
        frequencyZ: 0.08 + Math.random() * 0.3,
        offsetX: Math.random() * Math.PI * 2,
        offsetZ: Math.random() * Math.PI * 2,
        directionAngle: flowDirection,  // Biased toward flow direction
        phase: Math.random() * Math.PI * 2,
        thickness: 0.5 + Math.random() * 0.5,
        flowWeight: 0.7 + Math.random() * 0.3  // How much this wave follows main flow
      });
    }
  }

  function createFloatingObjects() {
    const objectTypes = [
      { type: 'boat', size: 0.3, color: 0x8B4513 },
      { type: 'log', size: 0.2, color: 0x654321 },
      { type: 'bottle', size: 0.15, color: 0x228B22 },
      { type: 'barrel', size: 0.25, color: 0x8B4513 }
    ];

    // Create object pool
    for (let i = 0; i < 15; i++) {
      const objType = objectTypes[Math.floor(Math.random() * objectTypes.length)];
      let geometry, material;

      switch (objType.type) {
        case 'boat':
          // Simple boat shape
          geometry = new THREE.ConeGeometry(objType.size, objType.size * 2, 6);
          material = new THREE.MeshPhongMaterial({ 
            color: objType.color,
            shininess: 30
          });
          break;
        case 'log':
          // Cylindrical log
          geometry = new THREE.CylinderGeometry(objType.size * 0.5, objType.size * 0.5, objType.size * 3, 8);
          material = new THREE.MeshPhongMaterial({ 
            color: objType.color,
            shininess: 10
          });
          break;
        case 'bottle':
          // Bottle shape
          geometry = new THREE.CylinderGeometry(objType.size * 0.3, objType.size * 0.5, objType.size * 4, 8);
          material = new THREE.MeshPhongMaterial({ 
            color: objType.color,
            transparent: true,
            opacity: 0.7,
            shininess: 100
          });
          break;
        case 'barrel':
          // Barrel shape
          geometry = new THREE.CylinderGeometry(objType.size * 0.8, objType.size * 0.8, objType.size * 2, 8);
          material = new THREE.MeshPhongMaterial({ 
            color: objType.color,
            shininess: 20
          });
          break;
      }

      const mesh = new THREE.Mesh(geometry, material);
      mesh.visible = false;
      scene.add(mesh);

      objectPool.push({
        mesh: mesh,
        type: objType.type,
        size: objType.size,
        active: false,
        position: new THREE.Vector3(),
        velocity: new THREE.Vector3(),
        bobOffset: Math.random() * Math.PI * 2,
        rotationSpeed: (Math.random() - 0.5) * 0.02
      });
    }
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

      // Multiple wave layers with flow-biased movement
      waveData.forEach((wave, index) => {
        // Wave movement with flow direction bias
        const flowInfluence = wave.flowWeight;
        const randomInfluence = 1 - flowInfluence;
        
        // Combine flow-based movement with random variation
        const flowX = vert.x * wave.frequencyX + currentTime * wave.speedX * flowInfluence + wave.offsetX;
        const randomX = vert.x * wave.frequencyX * randomInfluence + currentTime * wave.speedX * randomInfluence + wave.phase;
        const waveX = flowX + randomX;
        
        const waveZ = vert.z * wave.frequencyZ + currentTime * wave.speedZ + wave.offsetZ;
        
        // Simplified wave patterns for better performance
        let wavePattern;
        if (index % 2 === 0) {
          // Main flow wave - simplified calculation
          const flowBase = Math.sin(waveX) * Math.cos(waveZ * 0.5);
          const thick = Math.sin(waveX * 1.3) * 0.3;
          wavePattern = (flowBase + thick) * wave.amplitude * wave.thickness;
        } else {
          // Diagonal flow wave - simplified
          const diagonalFlow = Math.sin(waveX * 0.8 + waveZ * 0.6);
          wavePattern = diagonalFlow * wave.amplitude * wave.thickness;
        }
        
        // Add directional volume wave that follows flow (slower)
        const volumeWave = Math.sin(
          vert.x * Math.cos(wave.directionAngle) * 0.6 + 
          vert.z * Math.sin(wave.directionAngle) * 0.3 + 
          currentTime * (0.8 + index * 0.1) * flowInfluence  // Reduced speed multiplier
        ) * wave.amplitude * wave.thickness * 0.3;  // Reduced amplitude
        
        height += wavePattern + volumeWave;
        
        // Add flow-biased horizontal movement
        offsetX += Math.sin(waveZ + wave.phase) * 0.008 * flowInfluence;
        offsetZ += Math.cos(waveX - wave.phase) * 0.006 * (1 - flowInfluence * 0.5);
      });
      
      // Simplified noise for performance (slower and reduced)
      const noise = Math.sin(vert.x * 5 + currentTime * 1) * 
                   Math.cos(vert.z * 4 - currentTime * 0.8) * 0.01;  // Reduced amplitude
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

  // Function to calculate wave height at specific position
  function getWaveHeightAtPosition(x, z, currentTime) {
    let height = 0;
    
    waveData.forEach((wave, index) => {
      const flowInfluence = wave.flowWeight;
      const randomInfluence = 1 - flowInfluence;
      
      const flowX = x * wave.frequencyX + currentTime * wave.speedX * flowInfluence + wave.offsetX;
      const randomX = x * wave.frequencyX * randomInfluence + currentTime * wave.speedX * randomInfluence + wave.phase;
      const waveX = flowX + randomX;
      const waveZ = z * wave.frequencyZ + currentTime * wave.speedZ + wave.offsetZ;
      
      let wavePattern;
      if (index % 2 === 0) {
        const flowBase = Math.sin(waveX) * Math.cos(waveZ * 0.5);
        const thick = Math.sin(waveX * 1.3) * 0.3;
        wavePattern = (flowBase + thick) * wave.amplitude * wave.thickness;
      } else {
        const diagonalFlow = Math.sin(waveX * 0.8 + waveZ * 0.6);
        wavePattern = diagonalFlow * wave.amplitude * wave.thickness;
      }
      
      const volumeWave = Math.sin(
        x * Math.cos(wave.directionAngle) * 0.6 + 
        z * Math.sin(wave.directionAngle) * 0.3 + 
        currentTime * (0.8 + index * 0.1) * flowInfluence
      ) * wave.amplitude * wave.thickness * 0.3;
      
      height += wavePattern + volumeWave;
    });
    
    // Add noise
    const noise = Math.sin(x * 5 + currentTime * 1) * 
                 Math.cos(z * 4 - currentTime * 0.8) * 0.01;
    height += noise;
    
    return height;
  }

  function startObjectSpawning() {
    // Spawn objects periodically
    setInterval(() => {
      spawnFloatingObject();
    }, 3000 + Math.random() * 2000); // Every 3-5 seconds
    
    // Spawn initial objects
    for (let i = 0; i < 3; i++) {
      setTimeout(() => spawnFloatingObject(), i * 1000);
    }
  }

  function spawnFloatingObject() {
    // Get available object from pool
    const availableObj = objectPool.find(obj => !obj.active);
    if (!availableObj) return;

    // Position object at right edge
    const startX = config.WORLD_X / 2 + 2;
    const startZ = (Math.random() - 0.5) * config.WORLD_Z * 0.8;
    const startY = config.WATER_Y;

    availableObj.position.set(startX, startY, startZ);
    availableObj.velocity.set(-0.5 - Math.random() * 0.5, 0, (Math.random() - 0.5) * 0.2);
    availableObj.active = true;
    availableObj.mesh.visible = true;
    availableObj.mesh.position.copy(availableObj.position);
    
    // Random rotation for variety
    availableObj.mesh.rotation.y = Math.random() * Math.PI * 2;
    
    floatingObjects.push(availableObj);
  }

  function updateFloatingObjects(deltaTime) {
    const currentTime = clock.getElapsedTime();
    
    for (let i = floatingObjects.length - 1; i >= 0; i--) {
      const obj = floatingObjects[i];
      
      // Update position
      obj.position.x += obj.velocity.x * deltaTime * 60; // 60fps normalized
      obj.position.z += obj.velocity.z * deltaTime * 60;
      
      // Calculate wave height at object position
      const waveHeight = getWaveHeightAtPosition(obj.position.x, obj.position.z, currentTime);
      obj.position.y = config.WATER_Y + waveHeight + 0.1; // Slightly above water
      
      // Add bobbing motion
      const bobbing = Math.sin(currentTime * 2 + obj.bobOffset) * 0.05;
      obj.position.y += bobbing;
      
      // Update mesh position
      obj.mesh.position.copy(obj.position);
      
      // Rotate object slightly
      obj.mesh.rotation.y += obj.rotationSpeed;
      obj.mesh.rotation.z = Math.sin(currentTime * 1.5 + obj.bobOffset) * 0.1; // Gentle rolling
      
      // Remove object if it's too far left
      if (obj.position.x < -config.WORLD_X / 2 - 3) {
        obj.active = false;
        obj.mesh.visible = false;
        floatingObjects.splice(i, 1);
      }
    }
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
    
    // Skip frames on low performance devices
    if (deltaTime > 0.05) { // Skip if frame took more than 50ms
      return;
    }

    // Update water surface waves
    updateWaterSurface();
    
    // Update floating objects
    updateFloatingObjects(deltaTime);

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