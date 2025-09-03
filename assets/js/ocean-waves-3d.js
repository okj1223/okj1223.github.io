// 3D Ocean Waves Implementation
// Based on WellRipple3D component

(function() {
  'use strict';

  // Configuration
  const config = {
    WORLD_X: 18,
    WORLD_Z: 10,
    WATER_Y: 0,
    waterAlpha: 0.7,
    maxActiveRipples: 5
  };

  let scene, renderer, camera, waterMesh;
  let raycaster = new THREE.Raycaster();
  let clickPoint = new THREE.Vector3();
  let waterPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  
  // Splash columns
  let columnPool = [];
  let columnsActive = [];
  
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
    const segments = 128;
    const geometry = new THREE.PlaneGeometry(config.WORLD_X, config.WORLD_Z, segments, segments);
    
    // Create water material with better ocean color
    const material = new THREE.MeshPhongMaterial({
      color: 0x006994,  // Deep ocean blue
      transparent: true,
      opacity: config.waterAlpha,
      side: THREE.DoubleSide,
      shininess: 100,
      specular: 0x111111,
      flatShading: false
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
  }

  function createColumnPool() {
    const poolSize = 100;
    const columnGeo = new THREE.CylinderGeometry(0.15, 0.15, 1, 8);
    const columnMat = new THREE.MeshPhongMaterial({
      color: 0x00aaff,
      transparent: true,
      opacity: 0.3
    });

    for (let i = 0; i < poolSize; i++) {
      const mesh = new THREE.Mesh(columnGeo, columnMat.clone());
      mesh.visible = false;
      scene.add(mesh);
      columnPool.push({
        mesh: mesh,
        age: 0,
        maxAge: 2,
        baseRadius: 0.15,
        position: new THREE.Vector3()
      });
    }
  }

  function spawnRipple(position) {
    if (columnsActive.length >= config.maxActiveRipples) return;

    // Get column from pool
    const column = columnPool.find(c => !columnsActive.includes(c));
    if (!column) return;

    // Initialize column
    column.position.copy(position);
    column.age = 0;
    column.mesh.position.copy(position);
    column.mesh.position.y = config.WATER_Y;
    column.mesh.visible = true;
    column.mesh.scale.set(0.1, 0.1, 0.1);
    
    columnsActive.push(column);

    // Create ripple effect on water mesh
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
    if (!waterMesh || !waterMesh.userData.ripples) return;

    const geometry = waterMesh.geometry;
    const positions = geometry.attributes.position.array;
    const originalVerts = waterMesh.userData.originalVertices;
    const currentTime = clock.getElapsedTime();

    // Reset to original positions
    for (let i = 0; i < originalVerts.length; i++) {
      const idx = i * 3;
      positions[idx + 2] = 0; // Reset Y (which is Z in world space due to rotation)
    }

    // Apply all active ripples
    waterMesh.userData.ripples.forEach(ripple => {
      const age = currentTime - ripple.startTime;
      const radius = age * ripple.speed;
      const amplitude = ripple.amplitude * Math.exp(-age * ripple.decay);

      for (let i = 0; i < originalVerts.length; i++) {
        const vert = originalVerts[i];
        const dx = vert.x - ripple.center.x;
        const dz = vert.y - ripple.center.z; // y in plane = z in world
        const distance = Math.sqrt(dx * dx + dz * dz);

        if (distance < radius + 1) {
          const wave = Math.sin((radius - distance) * 2) * amplitude;
          const falloff = Math.max(0, 1 - distance / (radius + 1));
          positions[i * 3 + 2] += wave * falloff;
        }
      }
    });

    geometry.attributes.position.needsUpdate = true;
    geometry.computeVertexNormals();
  }

  function updateColumns(deltaTime) {
    for (let i = columnsActive.length - 1; i >= 0; i--) {
      const column = columnsActive[i];
      column.age += deltaTime;

      if (column.age >= column.maxAge) {
        // Return to pool
        column.mesh.visible = false;
        columnsActive.splice(i, 1);
        continue;
      }

      // Animate column
      const progress = column.age / column.maxAge;
      const scale = 0.1 + progress * 2;
      const opacity = Math.max(0, 0.3 * (1 - progress));
      
      column.mesh.scale.set(scale, 0.5 + progress, scale);
      column.mesh.material.opacity = opacity;
      column.mesh.position.y = config.WATER_Y + progress * 0.5;
    }
  }

  function startAutoRipple() {
    // Create random ripples periodically
    setInterval(() => {
      if (columnsActive.length < config.maxActiveRipples) {
        const x = (Math.random() - 0.5) * config.WORLD_X * 0.8;
        const z = (Math.random() - 0.5) * config.WORLD_Z * 0.8;
        spawnRipple(new THREE.Vector3(x, config.WATER_Y, z));
      }
    }, 2000);
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

    // Update splash columns
    updateColumns(deltaTime);

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