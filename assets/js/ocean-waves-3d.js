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
  
  // Splash effects
  let splashPool = [];
  let activeSplashes = [];
  
  // Animation state
  let animationId;
  let clock = new THREE.Clock();
  
  // Cleanup notification system
  let cleanupNotifications = [];

  function init() {
    const container = document.getElementById('ocean-waves-container');
    if (!container) return;

    const width = container.clientWidth || window.innerWidth;
    const height = 400; // Fixed height for footer

    // Scene setup
    scene = new THREE.Scene();

    // Renderer setup for higher quality
    renderer = new THREE.WebGLRenderer({ 
      antialias: true, // Always use antialiasing for smoother edges
      alpha: true,
      powerPreference: "high-performance"
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2)); // Limit for better performance
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

    // Create floating objects and splash effects
    createFloatingObjects();
    createSplashEffects();

    // Start object spawning
    startObjectSpawning();

    // Event listeners
    container.addEventListener('click', onContainerClick);
    window.addEventListener('resize', onWindowResize);

    // Start animation
    animate();
  }

  function createWaterSurface() {
    // Create unified water body using BoxGeometry optimized for maximum performance
    const segments = 64; // Reduced for better frame rate
    const geometry = new THREE.BoxGeometry(config.WORLD_X, 3, config.WORLD_Z, segments, 8, segments);
    
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

    // Initialize wave data - optimized for performance
    for (let i = 0; i < 4; i++) { // Reduced from 6 to 4 waves for better performance
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
      { type: 'boat', size: 0.7, color: 0x8B4513 },      // Even larger for easier clicking
      { type: 'log', size: 0.5, color: 0x654321 },       // Even larger for easier clicking
      { type: 'bottle', size: 0.4, color: 0x228B22 },    // Even larger for easier clicking
      { type: 'barrel', size: 0.6, color: 0x8B4513 }     // Even larger for easier clicking
    ];

    // Create optimized object pool for maximum performance
    for (let i = 0; i < 12; i++) { // Further reduced from 15 to 12 for better click responsiveness
      const objType = objectTypes[Math.floor(Math.random() * objectTypes.length)];
      let geometry, material;

      switch (objType.type) {
        case 'boat':
          // Ultra-simplified boat shape for maximum performance
          geometry = new THREE.ConeGeometry(objType.size, objType.size * 2, 3);
          material = new THREE.MeshBasicMaterial({ // Basic material for maximum performance
            color: objType.color
          });
          break;
        case 'log':
          // Ultra-simplified log with minimal segments
          geometry = new THREE.CylinderGeometry(objType.size * 0.5, objType.size * 0.5, objType.size * 3, 4);
          material = new THREE.MeshBasicMaterial({ 
            color: objType.color
          });
          break;
        case 'bottle':
          // Ultra-simplified bottle with minimal segments
          geometry = new THREE.CylinderGeometry(objType.size * 0.3, objType.size * 0.5, objType.size * 4, 4);
          material = new THREE.MeshBasicMaterial({ 
            color: objType.color,
            transparent: true,
            opacity: 0.8 // Slightly less transparent for better visibility
          });
          break;
        case 'barrel':
          // Ultra-simplified barrel with minimal segments
          geometry = new THREE.CylinderGeometry(objType.size * 0.8, objType.size * 0.8, objType.size * 2, 4);
          material = new THREE.MeshBasicMaterial({ 
            color: objType.color
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
        rotationSpeed: (Math.random() - 0.5) * 0.02,
        // Falling and floating states
        state: 'idle', // 'falling', 'submerged', 'floating'
        fallSpeed: 0,
        submersionDepth: 0,
        floatTimer: 0
      });
    }
  }

  function createSplashEffects() {
    // Create optimized splash particle pool for performance
    for (let i = 0; i < 18; i++) { // Reduced from 24 to 18 for better performance
      const splashGeo = new THREE.SphereGeometry(0.03, 4, 4); // Fewer segments for better performance
      const splashMat = new THREE.MeshBasicMaterial({ // Basic material for maximum performance
        color: 0x88ccff,
        transparent: true,
        opacity: 0.7
      });
      
      const splash = new THREE.Mesh(splashGeo, splashMat);
      splash.visible = false;
      scene.add(splash);
      
      splashPool.push({
        mesh: splash,
        active: false,
        velocity: new THREE.Vector3(),
        life: 0,
        maxLife: 1.0 // Shorter life for faster cleanup
      });
    }
  }

  // Ripple functions removed - no longer needed

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
        
        // Simplified directional wave for performance
        const volumeWave = Math.sin(
          (vert.x + vert.z) * 0.5 + currentTime * flowInfluence
        ) * wave.amplitude * 0.2;  // Much simpler calculation
        
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

    // Click ripples removed for cleaner ocean surface

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
    // Spawn objects at moderate frequency - not too fast, not too slow
    setInterval(() => {
      spawnFloatingObject();
    }, 5000 + Math.random() * 4000); // Every 5-9 seconds (balanced)
    
    // Spawn a couple initial objects
    for (let i = 0; i < 2; i++) {
      setTimeout(() => spawnFloatingObject(), i * 2500);
    }
  }

  function spawnFloatingObject() {
    // Get available object from pool
    const availableObj = objectPool.find(obj => !obj.active);
    if (!availableObj) return;

    // Position object ABOVE the left edge for falling effect
    const startX = -config.WORLD_X / 2 - 1;
    const startZ = (Math.random() - 0.5) * config.WORLD_Z * 0.6; // More restricted Z range
    const startY = config.WATER_Y + 3 + Math.random() * 2; // Start above water

    availableObj.position.set(startX, startY, startZ);
    // NO horizontal movement during fall - straight drop!
    availableObj.velocity.set(0, 0, 0);
    availableObj.active = true;
    availableObj.mesh.visible = true;
    availableObj.mesh.position.copy(availableObj.position);
    
    // Set falling state
    availableObj.state = 'falling';
    availableObj.fallSpeed = 0;
    availableObj.submersionDepth = 0;
    availableObj.floatTimer = 0;
    availableObj.targetZ = startZ; // Remember original Z for straight fall
    
    // Random rotation for variety
    availableObj.mesh.rotation.y = Math.random() * Math.PI * 2;
    
    floatingObjects.push(availableObj);
  }

  function createSplash(position, intensity = 1) {
    // Optimized particles for performance
    const numParticles = Math.floor(5 + Math.random() * 5);
    
    for (let i = 0; i < numParticles && i < 10; i++) { // Max 10 particles
      const splash = splashPool.find(s => !s.active);
      if (!splash) continue;
      
      splash.active = true;
      splash.life = 0;
      splash.mesh.visible = true;
      splash.mesh.position.copy(position);
      splash.mesh.position.y += Math.random() * 0.15;
      
      // Simpler splash velocity
      splash.velocity.set(
        (Math.random() - 0.5) * 1.5,
        0.8 + Math.random() * 1.2,
        (Math.random() - 0.5) * 1.5
      );
      
      activeSplashes.push(splash);
    }
    
    // Ripple effect removed for cleaner gameplay
  }

  function updateFloatingObjects(deltaTime) {
    const currentTime = clock.getElapsedTime();
    
    for (let i = floatingObjects.length - 1; i >= 0; i--) {
      const obj = floatingObjects[i];
      
      // Only update horizontal position for floating objects
      // (falling and submerged states handle their own movement)
      
      const waveHeight = getWaveHeightAtPosition(obj.position.x, obj.position.z, currentTime);
      const waterSurfaceY = config.WATER_Y + waveHeight;
      
      // State machine for falling and floating
      switch (obj.state) {
        case 'falling':
          // Apply stronger gravity for faster, more realistic fall
          obj.fallSpeed += 9.8 * deltaTime * 1.5; // Faster fall
          obj.position.y -= obj.fallSpeed * deltaTime;
          
          // Keep Z position fixed during fall (straight drop)
          obj.position.z = obj.targetZ;
          
          // Check if hit water surface
          if (obj.position.y <= waterSurfaceY) {
            obj.position.y = waterSurfaceY;
            obj.state = 'submerged';
            obj.submersionDepth = 0;
            obj.floatTimer = 0;
            
            // Set wave-following velocity after impact (slower horizontal speed, limited Z drift)
            obj.velocity.set(0.08 + Math.random() * 0.05, 0, (Math.random() - 0.5) * 0.02); // Even smaller Z drift
            
            // Create splash effect
            createSplash(obj.position.clone(), obj.size);
          }
          break;
          
        case 'submerged':
          // Faster buoyancy response - rapid sink and rise
          obj.floatTimer += deltaTime * 8; // Much faster buoyancy
          const sinkDepth = Math.sin(obj.floatTimer) * obj.size * 0.4; // Deeper sink
          obj.position.y = waterSurfaceY - Math.max(0, sinkDepth);
          
          // Start horizontal movement during buoyancy (much slower)
          obj.position.x += obj.velocity.x * deltaTime * 15; // Much slower during buoyancy
          obj.position.z += obj.velocity.z * deltaTime * 15;
          
          // Strict boundary enforcement during buoyancy
          const halfZ_buoyant = config.WORLD_Z * 0.35; // Even tighter bounds during buoyancy
          obj.position.z = THREE.MathUtils.clamp(obj.position.z, -halfZ_buoyant, halfZ_buoyant);
          
          // Transition to floating after rapid settling
          if (obj.floatTimer > Math.PI * 0.8) { // Faster transition
            obj.state = 'floating';
          }
          break;
          
        case 'floating':
          // Normal floating behavior with regular wave speed
          obj.position.y = waterSurfaceY + obj.size * 0.2;
          
          // Add bobbing motion
          const bobbing = Math.sin(currentTime * 2 + obj.bobOffset) * 0.05;
          obj.position.y += bobbing;
          
          // Slower horizontal movement 
          obj.position.x += obj.velocity.x * deltaTime * 25; // Much slower floating speed
          obj.position.z += obj.velocity.z * deltaTime * 25;
          
          // Strong boundary enforcement for floating objects (Z-axis only)
          const halfZ_floating = config.WORLD_Z * 0.4; // Tighter bounds for floating
          obj.position.z = THREE.MathUtils.clamp(obj.position.z, -halfZ_floating, halfZ_floating);
          
          // Let objects flow freely to the right edge where they'll be removed
          break;
      }
      
      // Update mesh position
      obj.mesh.position.copy(obj.position);
      
      // Rotate object slightly
      obj.mesh.rotation.y += obj.rotationSpeed;
      if (obj.state === 'floating') {
        obj.mesh.rotation.z = Math.sin(currentTime * 1.5 + obj.bobOffset) * 0.1;
      }
      
      // Remove object if it's too far right
      if (obj.position.x > config.WORLD_X / 2 + 3) {
        obj.active = false;
        obj.mesh.visible = false;
        floatingObjects.splice(i, 1);
      }
    }
    
    // Update splash particles
    for (let i = activeSplashes.length - 1; i >= 0; i--) {
      const splash = activeSplashes[i];
      splash.life += deltaTime;
      
      // Update splash position
      splash.mesh.position.add(splash.velocity.clone().multiplyScalar(deltaTime));
      splash.velocity.y -= 9.8 * deltaTime * 2; // Gravity on splash particles
      
      // Fade out splash
      const fadeProgress = splash.life / splash.maxLife;
      splash.mesh.material.opacity = 0.8 * (1 - fadeProgress);
      splash.mesh.scale.setScalar(1 + fadeProgress * 2);
      
      // Remove expired splash
      if (splash.life >= splash.maxLife) {
        splash.active = false;
        splash.mesh.visible = false;
        activeSplashes.splice(i, 1);
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
    
    // Check for intersections with floating objects
    const intersectableObjects = floatingObjects.map(obj => obj.mesh);
    const intersects = raycaster.intersectObjects(intersectableObjects);
    
    // If no direct hit, try with much larger click area for easier clicking
    if (intersects.length === 0) {
      // Try multiple nearby points with generous tolerance
      const clickTolerance = 0.08; // Increased tolerance for easier clicking
      const nearbyPoints = [
        new THREE.Vector2(x - clickTolerance, y),
        new THREE.Vector2(x + clickTolerance, y),
        new THREE.Vector2(x, y - clickTolerance),
        new THREE.Vector2(x, y + clickTolerance),
        new THREE.Vector2(x - clickTolerance, y - clickTolerance),
        new THREE.Vector2(x + clickTolerance, y + clickTolerance),
        new THREE.Vector2(x - clickTolerance, y + clickTolerance),
        new THREE.Vector2(x + clickTolerance, y - clickTolerance),
        // Add more diagonal points for better coverage
        new THREE.Vector2(x - clickTolerance * 0.5, y - clickTolerance * 0.5),
        new THREE.Vector2(x + clickTolerance * 0.5, y + clickTolerance * 0.5)
      ];
      
      for (const point of nearbyPoints) {
        raycaster.setFromCamera(point, camera);
        const nearbyIntersects = raycaster.intersectObjects(intersectableObjects);
        if (nearbyIntersects.length > 0) {
          intersects.push(...nearbyIntersects);
          break;
        }
      }
    }
    
    if (intersects.length > 0) {
      // Find the clicked object and remove it
      const clickedMesh = intersects[0].object;
      const objIndex = floatingObjects.findIndex(obj => obj.mesh === clickedMesh);
      
      if (objIndex !== -1) {
        const removedObj = floatingObjects[objIndex];
        
        // Remove from active objects
        removedObj.active = false;
        removedObj.mesh.visible = false;
        floatingObjects.splice(objIndex, 1);
        
        // Show cleanup notification
        showCleanupNotification(intersects[0].point);
        
        return; // Don't create ripple if object was clicked
      }
    }
  }

  function showCleanupNotification(position) {
    const container = document.getElementById('ocean-waves-container');
    if (!container) return;
    
    // Create notification element
    const notification = document.createElement('div');
    notification.textContent = 'Ocean cleaned!';
    notification.style.cssText = `
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      background: rgba(34, 139, 34, 0.9);
      color: white;
      padding: 12px 24px;
      border-radius: 25px;
      font-family: Arial, sans-serif;
      font-size: 18px;
      font-weight: bold;
      z-index: 1000;
      pointer-events: none;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
      animation: cleanupFade 2s ease-out forwards;
    `;
    
    // Add CSS animation if not already added
    if (!document.getElementById('cleanup-notification-styles')) {
      const style = document.createElement('style');
      style.id = 'cleanup-notification-styles';
      style.textContent = `
        @keyframes cleanupFade {
          0% {
            opacity: 0;
            transform: translate(-50%, -50%) scale(0.5);
          }
          20% {
            opacity: 1;
            transform: translate(-50%, -50%) scale(1.1);
          }
          100% {
            opacity: 0;
            transform: translate(-50%, -50%) scale(1) translateY(-20px);
          }
        }
      `;
      document.head.appendChild(style);
    }
    
    container.appendChild(notification);
    
    // Remove notification after animation
    setTimeout(() => {
      if (notification.parentNode) {
        notification.parentNode.removeChild(notification);
      }
    }, 2000);
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
    
    // Remove frame skipping for higher frame rates
    // Update everything every frame for smooth animation
    updateWaterSurface();
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