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
        
        // Enhanced Ocean shader material with better wave physics
        const material = new THREE.ShaderMaterial({
            uniforms: {
                time: { value: 0 },
                waveHeight: { value: 4.5 },
                waveSpeed: { value: 1.2 },
                waveFrequency: { value: 0.8 },
                waveSteepness: { value: 0.3 },
                waveDirection1: { value: new THREE.Vector2(1, 0.5) },
                waveDirection2: { value: new THREE.Vector2(-0.7, 1) },
                waveDirection3: { value: new THREE.Vector2(0.3, -0.8) },
                color1: { value: new THREE.Color('#004466') },
                color2: { value: new THREE.Color('#006994') },
                color3: { value: new THREE.Color('#0099cc') },
                color4: { value: new THREE.Color('#66ccff') },
                foamColor: { value: new THREE.Color('#ffffff') },
                deepColor: { value: new THREE.Color('#002233') }
            },
            vertexShader: `
                uniform float time;
                uniform float waveHeight;
                uniform float waveSpeed;
                uniform float waveFrequency;
                uniform float waveSteepness;
                uniform vec2 waveDirection1;
                uniform vec2 waveDirection2;
                uniform vec2 waveDirection3;
                
                varying vec2 vUv;
                varying float vElevation;
                varying vec3 vPosition;
                varying vec3 vNormal;
                
                // Improved noise function
                float hash(vec2 p) {
                    vec3 p3 = fract(vec3(p.xyx) * 0.13);
                    p3 += dot(p3, p3.yzx + 3.333);
                    return fract((p3.x + p3.y) * p3.z);
                }
                
                float noise(vec2 p) {
                    vec2 i = floor(p);
                    vec2 f = fract(p);
                    f = f * f * (3.0 - 2.0 * f);
                    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x),
                              mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
                }
                
                // Gerstner wave function for realistic ocean waves
                vec3 gerstnerWave(vec2 direction, float amplitude, float frequency, float phase, vec2 position) {
                    float steepness = waveSteepness / (frequency * amplitude);
                    float c = cos(frequency * dot(direction, position) + phase);
                    float s = sin(frequency * dot(direction, position) + phase);
                    
                    return vec3(
                        steepness * amplitude * direction.x * c,
                        amplitude * s,
                        steepness * amplitude * direction.y * c
                    );
                }
                
                void main() {
                    vUv = uv;
                    vPosition = position;
                    
                    vec2 pos2D = position.xz;
                    vec3 wavePos = position;
                    
                    // Multiple Gerstner waves for realistic ocean
                    vec3 wave1 = gerstnerWave(normalize(waveDirection1), waveHeight, waveFrequency, time * waveSpeed, pos2D);
                    vec3 wave2 = gerstnerWave(normalize(waveDirection2), waveHeight * 0.7, waveFrequency * 1.3, time * waveSpeed * 0.8, pos2D);
                    vec3 wave3 = gerstnerWave(normalize(waveDirection3), waveHeight * 0.4, waveFrequency * 2.1, time * waveSpeed * 1.2, pos2D);
                    
                    // Add smaller detail waves
                    float detailWave1 = sin(pos2D.x * 0.3 + time * 2.0) * cos(pos2D.y * 0.2 + time * 1.5) * waveHeight * 0.2;
                    float detailWave2 = noise(pos2D * 0.05 + time * 0.1) * waveHeight * 0.3;
                    
                    // Combine all waves
                    wavePos += wave1 + wave2 + wave3;
                    wavePos.y += detailWave1 + detailWave2;
                    
                    vElevation = wavePos.y;
                    
                    // Calculate normal for better lighting
                    float eps = 0.5;
                    vec3 dx = vec3(eps, 0.0, 0.0);
                    vec3 dz = vec3(0.0, 0.0, eps);
                    
                    float heightL = (gerstnerWave(normalize(waveDirection1), waveHeight, waveFrequency, time * waveSpeed, pos2D - dx.xz) +
                                   gerstnerWave(normalize(waveDirection2), waveHeight * 0.7, waveFrequency * 1.3, time * waveSpeed * 0.8, pos2D - dx.xz) +
                                   gerstnerWave(normalize(waveDirection3), waveHeight * 0.4, waveFrequency * 2.1, time * waveSpeed * 1.2, pos2D - dx.xz)).y;
                    
                    float heightR = (gerstnerWave(normalize(waveDirection1), waveHeight, waveFrequency, time * waveSpeed, pos2D + dx.xz) +
                                   gerstnerWave(normalize(waveDirection2), waveHeight * 0.7, waveFrequency * 1.3, time * waveSpeed * 0.8, pos2D + dx.xz) +
                                   gerstnerWave(normalize(waveDirection3), waveHeight * 0.4, waveFrequency * 2.1, time * waveSpeed * 1.2, pos2D + dx.xz)).y;
                    
                    float heightB = (gerstnerWave(normalize(waveDirection1), waveHeight, waveFrequency, time * waveSpeed, pos2D - dz.xz) +
                                   gerstnerWave(normalize(waveDirection2), waveHeight * 0.7, waveFrequency * 1.3, time * waveSpeed * 0.8, pos2D - dz.xz) +
                                   gerstnerWave(normalize(waveDirection3), waveHeight * 0.4, waveFrequency * 2.1, time * waveSpeed * 1.2, pos2D - dz.xz)).y;
                    
                    float heightF = (gerstnerWave(normalize(waveDirection1), waveHeight, waveFrequency, time * waveSpeed, pos2D + dz.xz) +
                                   gerstnerWave(normalize(waveDirection2), waveHeight * 0.7, waveFrequency * 1.3, time * waveSpeed * 0.8, pos2D + dz.xz) +
                                   gerstnerWave(normalize(waveDirection3), waveHeight * 0.4, waveFrequency * 2.1, time * waveSpeed * 1.2, pos2D + dz.xz)).y;
                    
                    vNormal = normalize(vec3(heightL - heightR, 2.0 * eps, heightB - heightF));
                    
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(wavePos, 1.0);
                }
            `,
            fragmentShader: `
                uniform vec3 color1;
                uniform vec3 color2;
                uniform vec3 color3;
                uniform vec3 color4;
                uniform vec3 foamColor;
                uniform vec3 deepColor;
                uniform float time;
                
                varying vec2 vUv;
                varying float vElevation;
                varying vec3 vPosition;
                varying vec3 vNormal;
                
                // Fresnel effect calculation
                float fresnel(vec3 viewDirection, vec3 normal, float power) {
                    return pow(1.0 - max(0.0, dot(viewDirection, normal)), power);
                }
                
                // Noise function for texture details
                float hash(vec2 p) {
                    vec3 p3 = fract(vec3(p.xyx) * 0.13);
                    p3 += dot(p3, p3.yzx + 3.333);
                    return fract((p3.x + p3.y) * p3.z);
                }
                
                float noise(vec2 p) {
                    vec2 i = floor(p);
                    vec2 f = fract(p);
                    f = f * f * (3.0 - 2.0 * f);
                    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x),
                              mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
                }
                
                void main() {
                    // Base color mixing based on elevation
                    float elevationFactor = clamp((vElevation + 6.0) / 12.0, 0.0, 1.0);
                    
                    vec3 baseColor = deepColor;
                    baseColor = mix(baseColor, color1, smoothstep(0.1, 0.3, elevationFactor));
                    baseColor = mix(baseColor, color2, smoothstep(0.3, 0.6, elevationFactor));
                    baseColor = mix(baseColor, color3, smoothstep(0.6, 0.8, elevationFactor));
                    baseColor = mix(baseColor, color4, smoothstep(0.8, 1.0, elevationFactor));
                    
                    // Foam on wave crests
                    float foamThreshold = 3.0;
                    float foamIntensity = smoothstep(foamThreshold, foamThreshold + 2.0, vElevation);
                    
                    // Dynamic foam based on wave steepness
                    float steepness = length(vec2(dFdx(vElevation), dFdy(vElevation)));
                    foamIntensity += smoothstep(0.5, 1.5, steepness) * 0.3;
                    
                    // Animated foam texture
                    vec2 foamUV = vPosition.xz * 0.1 + time * 0.1;
                    float foamNoise1 = noise(foamUV * 4.0);
                    float foamNoise2 = noise(foamUV * 8.0 + vec2(time * 0.2));
                    float foamPattern = foamNoise1 * 0.7 + foamNoise2 * 0.3;
                    
                    foamIntensity *= (0.7 + foamPattern * 0.3);
                    vec3 finalColor = mix(baseColor, foamColor, foamIntensity);
                    
                    // Fresnel reflection effect
                    vec3 viewDirection = normalize(cameraPosition - vPosition);
                    float fresnelValue = fresnel(viewDirection, vNormal, 3.0);
                    vec3 reflectionColor = color4;
                    finalColor = mix(finalColor, reflectionColor, fresnelValue * 0.4);
                    
                    // Surface shimmer and sparkles
                    vec2 shimmerUV = vPosition.xz * 0.05 + time * 0.08;
                    float shimmer1 = noise(shimmerUV * 3.0 + sin(time) * 0.1);
                    float shimmer2 = noise(shimmerUV * 6.0 - cos(time * 1.2) * 0.1);
                    float shimmerEffect = (shimmer1 * 0.6 + shimmer2 * 0.4);
                    
                    // Sparkles on wave peaks
                    float sparkleThreshold = 0.85;
                    float sparkles = step(sparkleThreshold, shimmerEffect) * smoothstep(2.0, 4.0, vElevation);
                    finalColor += sparkles * vec3(1.0, 1.0, 0.8) * 0.5;
                    
                    // Overall brightness variation
                    float brightness = 0.9 + shimmerEffect * 0.2;
                    finalColor *= brightness;
                    
                    // Subtle color temperature variation based on position
                    vec2 tempUV = vPosition.xz * 0.01 + time * 0.02;
                    float temperature = noise(tempUV) * 0.1;
                    finalColor += vec3(temperature * 0.1, -temperature * 0.05, -temperature * 0.1);
                    
                    gl_FragColor = vec4(finalColor, 1.0);
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