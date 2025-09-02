// 3D Ocean Map Implementation
class OceanMap3D {
    constructor(containerId) {
        console.log('🌊 3D Ocean Map initializing...');
        
        this.container = document.getElementById(containerId);
        if (!this.container) {
            console.error('Container not found:', containerId);
            return;
        }
        
        // Three.js core objects
        this.scene = new THREE.Scene();
        this.camera = null;
        this.renderer = null;
        this.oceanMesh = null;
        this.time = 0;
        
        // Ocean parameters
        this.oceanSize = 200;
        this.oceanSegments = 128;
        
        this.init();
        this.createOcean();
        this.setupLighting();
        this.setupControls();
        this.animate();
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());
        
        console.log('✅ 3D Ocean Map initialized');
    }
    
    init() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight || 400;
        
        // Camera setup
        this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
        this.camera.position.set(0, 50, 80);
        this.camera.lookAt(0, 0, 0);
        
        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true, 
            alpha: true,
            powerPreference: "high-performance"
        });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        // Set background to ocean blue
        this.renderer.setClearColor(0x006994, 1);
        
        this.container.appendChild(this.renderer.domElement);
    }
    
    createOcean() {
        // Create ocean geometry
        const geometry = new THREE.PlaneGeometry(
            this.oceanSize, 
            this.oceanSize, 
            this.oceanSegments, 
            this.oceanSegments
        );
        
        // Ocean shader material
        const material = new THREE.ShaderMaterial({
            uniforms: {
                time: { value: 0 },
                waveHeight: { value: 3.0 },
                waveSpeed: { value: 1.0 },
                color1: { value: new THREE.Color('#006994') },
                color2: { value: new THREE.Color('#0099cc') },
                color3: { value: new THREE.Color('#66ccff') },
                foamColor: { value: new THREE.Color('#ffffff') }
            },
            vertexShader: `
                uniform float time;
                uniform float waveHeight;
                uniform float waveSpeed;
                
                varying vec2 vUv;
                varying float vElevation;
                varying vec3 vPosition;
                
                // Noise function for realistic waves
                float noise(vec2 st) {
                    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453123);
                }
                
                void main() {
                    vUv = uv;
                    vPosition = position;
                    
                    vec3 pos = position;
                    
                    // Multiple wave layers for realistic ocean
                    float wave1 = sin(pos.x * 0.02 + time * waveSpeed) * cos(pos.z * 0.02 + time * waveSpeed * 0.8) * waveHeight;
                    float wave2 = sin(pos.x * 0.04 + time * waveSpeed * 1.2) * cos(pos.z * 0.03 + time * waveSpeed * 0.9) * waveHeight * 0.5;
                    float wave3 = sin(pos.x * 0.08 + time * waveSpeed * 0.7) * sin(pos.z * 0.06 + time * waveSpeed * 1.1) * waveHeight * 0.3;
                    
                    // Add some noise for texture
                    float noiseValue = noise(pos.xz * 0.1 + time * 0.1) * waveHeight * 0.2;
                    
                    pos.y = wave1 + wave2 + wave3 + noiseValue;
                    vElevation = pos.y;
                    
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 1.0);
                }
            `,
            fragmentShader: `
                uniform vec3 color1;
                uniform vec3 color2;
                uniform vec3 color3;
                uniform vec3 foamColor;
                uniform float time;
                
                varying vec2 vUv;
                varying float vElevation;
                varying vec3 vPosition;
                
                void main() {
                    // Color based on elevation
                    float mixFactor = (vElevation + 4.0) / 8.0;
                    vec3 color = mix(color1, color2, mixFactor);
                    color = mix(color, color3, smoothstep(1.5, 3.0, vElevation));
                    
                    // Add foam on wave crests
                    float foam = smoothstep(2.0, 3.5, vElevation);
                    color = mix(color, foamColor, foam * 0.7);
                    
                    // Add some shimmer effect
                    float shimmer = sin(vPosition.x * 0.1 + time * 2.0) * sin(vPosition.z * 0.1 + time * 1.5) * 0.1 + 0.9;
                    color *= shimmer;
                    
                    gl_FragColor = vec4(color, 1.0);
                }
            `,
            side: THREE.DoubleSide,
            wireframe: false
        });
        
        // Create ocean mesh
        this.oceanMesh = new THREE.Mesh(geometry, material);
        this.oceanMesh.rotation.x = -Math.PI / 2;
        this.oceanMesh.receiveShadow = true;
        
        this.scene.add(this.oceanMesh);
        
        // Add some floating objects for scale
        this.addFloatingElements();
    }
    
    addFloatingElements() {
        // Add a few boats/islands for reference
        const boatGeometry = new THREE.BoxGeometry(4, 1, 8);
        const boatMaterial = new THREE.MeshPhongMaterial({ color: 0x8B4513 });
        
        for (let i = 0; i < 3; i++) {
            const boat = new THREE.Mesh(boatGeometry, boatMaterial);
            boat.position.set(
                (Math.random() - 0.5) * 150,
                2,
                (Math.random() - 0.5) * 150
            );
            boat.rotation.y = Math.random() * Math.PI * 2;
            boat.castShadow = true;
            
            // Store initial position for floating animation
            boat.userData = {
                initialY: boat.position.y,
                floatSpeed: 0.5 + Math.random() * 0.5,
                floatOffset: Math.random() * Math.PI * 2
            };
            
            this.scene.add(boat);
        }
        
        // Add some seagulls
        const seagullGeometry = new THREE.SphereGeometry(0.5, 8, 6);
        const seagullMaterial = new THREE.MeshPhongMaterial({ color: 0xffffff });
        
        for (let i = 0; i < 5; i++) {
            const seagull = new THREE.Mesh(seagullGeometry, seagullMaterial);
            seagull.position.set(
                (Math.random() - 0.5) * 200,
                10 + Math.random() * 20,
                (Math.random() - 0.5) * 200
            );
            
            seagull.userData = {
                flySpeed: 1 + Math.random() * 2,
                flyRadius: 20 + Math.random() * 30,
                centerX: seagull.position.x,
                centerZ: seagull.position.z,
                angle: Math.random() * Math.PI * 2
            };
            
            this.scene.add(seagull);
        }
    }
    
    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x87ceeb, 0.6);
        this.scene.add(ambientLight);
        
        // Sun light
        const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
        directionalLight.position.set(50, 80, 30);
        directionalLight.castShadow = true;
        directionalLight.shadow.camera.left = -100;
        directionalLight.shadow.camera.right = 100;
        directionalLight.shadow.camera.top = 100;
        directionalLight.shadow.camera.bottom = -100;
        directionalLight.shadow.camera.near = 0.1;
        directionalLight.shadow.camera.far = 200;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        
        this.scene.add(directionalLight);
        
        // Add fog for atmosphere
        this.scene.fog = new THREE.Fog(0x87ceeb, 100, 300);
    }
    
    setupControls() {
        // Simple mouse controls for camera
        let isMouseDown = false;
        let mouseX = 0;
        let mouseY = 0;
        
        this.container.addEventListener('mousedown', (e) => {
            isMouseDown = true;
            mouseX = e.clientX;
            mouseY = e.clientY;
        });
        
        this.container.addEventListener('mouseup', () => {
            isMouseDown = false;
        });
        
        this.container.addEventListener('mousemove', (e) => {
            if (!isMouseDown) return;
            
            const deltaX = e.clientX - mouseX;
            const deltaY = e.clientY - mouseY;
            
            // Rotate camera around the ocean
            const spherical = new THREE.Spherical();
            spherical.setFromVector3(this.camera.position);
            
            spherical.theta -= deltaX * 0.01;
            spherical.phi += deltaY * 0.01;
            spherical.phi = Math.max(0.1, Math.min(Math.PI * 0.4, spherical.phi));
            
            this.camera.position.setFromSpherical(spherical);
            this.camera.lookAt(0, 0, 0);
            
            mouseX = e.clientX;
            mouseY = e.clientY;
        });
        
        // Zoom with mouse wheel
        this.container.addEventListener('wheel', (e) => {
            const distance = this.camera.position.length();
            const newDistance = Math.max(30, Math.min(150, distance + e.deltaY * 0.1));
            
            this.camera.position.normalize().multiplyScalar(newDistance);
            this.camera.lookAt(0, 0, 0);
        });
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        this.time += 0.016;
        
        // Update ocean waves
        if (this.oceanMesh && this.oceanMesh.material.uniforms) {
            this.oceanMesh.material.uniforms.time.value = this.time;
        }
        
        // Animate floating objects
        this.scene.children.forEach(child => {
            if (child.userData && child.userData.floatSpeed !== undefined) {
                // Boats floating
                child.position.y = child.userData.initialY + 
                    Math.sin(this.time * child.userData.floatSpeed + child.userData.floatOffset) * 1.5;
                child.rotation.z = Math.sin(this.time * child.userData.floatSpeed * 0.7) * 0.1;
            }
            
            if (child.userData && child.userData.flySpeed !== undefined) {
                // Seagulls flying in circles
                child.userData.angle += child.userData.flySpeed * 0.01;
                child.position.x = child.userData.centerX + 
                    Math.cos(child.userData.angle) * child.userData.flyRadius;
                child.position.z = child.userData.centerZ + 
                    Math.sin(child.userData.angle) * child.userData.flyRadius;
                child.position.y += Math.sin(this.time * 2) * 0.5;
            }
        });
        
        // Subtle camera movement for dynamic feel
        const basePosition = this.camera.position.clone().normalize().multiplyScalar(this.camera.position.length());
        this.camera.position.x = basePosition.x + Math.sin(this.time * 0.2) * 2;
        this.camera.position.y = basePosition.y + Math.cos(this.time * 0.15) * 1;
        this.camera.lookAt(0, 0, 0);
        
        this.renderer.render(this.scene, this.camera);
    }
    
    onWindowResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight || 400;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        
        this.renderer.setSize(width, height);
    }
    
    dispose() {
        window.removeEventListener('resize', this.onWindowResize);
        
        // Clean up Three.js resources
        this.scene.traverse(child => {
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
                if (child.material instanceof Array) {
                    child.material.forEach(material => material.dispose());
                } else {
                    child.material.dispose();
                }
            }
        });
        
        this.renderer.dispose();
        if (this.container.contains(this.renderer.domElement)) {
            this.container.removeChild(this.renderer.domElement);
        }
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    const container = document.getElementById('ocean-map-container');
    if (container && typeof THREE !== 'undefined') {
        console.log('🌊 Starting 3D Ocean Map...');
        window.oceanMap3D = new OceanMap3D('ocean-map-container');
    } else if (!container) {
        console.log('Ocean map container not found');
    } else {
        console.error('THREE.js not loaded');
    }
});

export default OceanMap3D;