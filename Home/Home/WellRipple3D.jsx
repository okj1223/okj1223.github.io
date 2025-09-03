// WellRipple3D.jsx
import React, { useRef, useEffect, forwardRef, useImperativeHandle } from "react";
import * as THREE from "three";

/**
 * 카테고리(4종)
 * - android : 안드로이드 느낌(바이저, 안테나, 캡슐형 관절)
 * - monitor : 컴퓨터(모니터+본체+키보드)
 * - robo    : 박스형 산업 로봇
 * - car     : 자동차(세단)
 *
 * 구성
 * - 얇은 물막 스플래시(작고 낮게) + 각도 하모닉 + 씨드 지터 + 저주파 노이즈
 * - "진짜 물방울" 파티클: 링+제트 2방울, 소프트 엣지, 라이프 페이드, 잔상 없음
 * - 시작 순서 고정: android → monitor → robo → car
 */

const WellRipple3D = forwardRef(function WellRipple3D(
  { style = {}, waterAlpha = 0.7, maxActiveRipples = 5, waterY = 0 },
  ref
) {
  const mountRef = useRef(null);

  const api = useRef({
    scene: null,
    renderer: null,
    camera: null,
    waterPlane: null,
    WORLD_X: 18,
    WORLD_Z: 10,
    WATER_Y: waterY,
    raycaster: new THREE.Raycaster(),
    clickPoint: new THREE.Vector3(),
    spawnDrop: null,
    spawnDropWithCategory: null,
    mountEl: null,

    // splash sheet
    columnPool: [],
    columnsActive: [],

    // droplets
    dropletsPoints: null,
    dropletsData: null,

    // shared
    shared: { geos: {}, mats: {}, init: false },
  });

  function pullZToCenter(z, lerpFactor, halfZ, bandRatio) {
    const zPulled = z + (0 - z) * THREE.MathUtils.clamp(lerpFactor, 0, 1);
    const zLimit = halfZ * bandRatio;
    return THREE.MathUtils.clamp(zPulled, -zLimit, zLimit);
  }

  useImperativeHandle(ref, () => ({
    dropAtScreen: (clientX, clientY) => {
      const { renderer, camera, waterPlane, raycaster, clickPoint, WORLD_X, WORLD_Z, spawnDropWithCategory, mountEl } = api.current;
      if (!renderer || !camera || !spawnDropWithCategory || !mountEl) return;

      const rect = mountEl.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((clientX - rect.left) / rect.width) * 2 - 1,
        -(((clientY - rect.top) / rect.height) * 2 - 1)
      );
      raycaster.setFromCamera(ndc, camera);
      if (!raycaster.ray.intersectPlane(waterPlane, clickPoint)) return;

      const halfX = WORLD_X * 0.5 * 0.98;
      const halfZ = WORLD_Z * 0.5 * 0.98;
      clickPoint.x = THREE.MathUtils.clamp(clickPoint.x, -halfX, halfX);
      clickPoint.z = THREE.MathUtils.clamp(clickPoint.z, -halfZ, halfZ);
      clickPoint.y = api.current.WATER_Y;
      clickPoint.z = pullZToCenter(clickPoint.z, 0.5, halfZ, 0.7);

      const cats = ["android", "monitor", "robo", "car"];
      const cat = cats[Math.floor(Math.random() * cats.length)];
      api.current.spawnDropWithCategory(clickPoint.clone(), cat);
    },
  }));

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    api.current.mountEl = mount;

    // ===== 기본 세팅 =====
    let width = mount.clientWidth || window.innerWidth;
    let height = mount.clientHeight || window.innerHeight;

    const scene = new THREE.Scene();
    api.current.scene = scene;

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(width, height);
    renderer.setClearColor(0x000000, 0);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.NoToneMapping;
    mount.appendChild(renderer.domElement);
    api.current.renderer = renderer;

    // 필요 없으면 주석 처리
    renderer.domElement.style.transform = "translateY(40%)";

    const camera = new THREE.PerspectiveCamera(40, width / height, 0.1, 2000);
    camera.position.set(0, 1.5, 7.5);
    camera.lookAt(0, 0.2, 0);
    api.current.camera = camera;

    // 라이트
    scene.add(new THREE.HemisphereLight(0xbdbbff, 0x120f1a, 0.9));
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(-3, 8, 6);
    scene.add(dir);

    // ===== 물 면 =====
    const WORLD_X = api.current.WORLD_X;
    const aspect = width / Math.max(1, height);
    api.current.WORLD_Z = (WORLD_X / Math.max(1, aspect)) * 0.55;
    const WORLD_Z = api.current.WORLD_Z;
    const WATER_Y = api.current.WATER_Y;

    const MAX_RIPPLES = 16;

    const planeGeom = new THREE.PlaneGeometry(WORLD_X, WORLD_Z, 240, 120);
    planeGeom.rotateX(-Math.PI / 2);
    planeGeom.translate(0, WATER_Y, 0);

    const uniforms = {
      uTime: { value: 0 },
      uRipplePos: { value: Array.from({ length: MAX_RIPPLES }, () => new THREE.Vector2()) },
      uRippleStartTime: { value: new Array(MAX_RIPPLES).fill(-9999) },
      uRippleAmp: { value: new Array(MAX_RIPPLES).fill(0) },
      uRippleWaveLen: { value: new Array(MAX_RIPPLES).fill(1.2) },
      uRippleSpeed: { value: new Array(MAX_RIPPLES).fill(1.95) },
      uRippleDamp: { value: new Array(MAX_RIPPLES).fill(1.3) },
      uRippleActive: { value: new Array(MAX_RIPPLES).fill(0) },
      uRippleFrontW: { value: new Array(MAX_RIPPLES).fill(0.3) },
      uAlpha: { value: THREE.MathUtils.clamp(waterAlpha, 0, 1) },

      // 잔물결
      uBaseAmp: { value: 0.075 },
      uBaseFreq: { value: 2.1 },
      uBaseSpeed: { value: 2.0 },
      uBaseDir: { value: new THREE.Vector4(1.0, 0.3, -0.6, 0.8) },
    };

    const vert = /* glsl */ `
      uniform float uTime;
      uniform vec2  uRipplePos[${MAX_RIPPLES}];
      uniform float uRippleStartTime[${MAX_RIPPLES}];
      uniform float uRippleAmp[${MAX_RIPPLES}];
      uniform float uRippleWaveLen[${MAX_RIPPLES}];
      uniform float uRippleSpeed[${MAX_RIPPLES}];
      uniform float uRippleDamp[${MAX_RIPPLES}];
      uniform int   uRippleActive[${MAX_RIPPLES}];
      uniform float uRippleFrontW[${MAX_RIPPLES}];

      uniform float uBaseAmp;
      uniform float uBaseFreq;
      uniform float uBaseSpeed;
      uniform vec4  uBaseDir;

      varying vec3 vPos;
      varying vec3 vNormalW;

      float rippleHeight(vec2 p){
        float sumSq=0.0, maxAbs=0.0, maxSign=1.0;
        for(int i=0;i<${MAX_RIPPLES};i++){
          if(uRippleActive[i]==0) continue;
          float t=uTime-uRippleStartTime[i];
          if(t<0.0) continue;
          float dist=distance(p,uRipplePos[i]);
          float k=6.28318/max(uRippleWaveLen[i],0.0001);
          float c=uRippleSpeed[i];
          float phase=k*dist - c*k*t;
          float fw=max(uRippleFrontW[i], 0.001);
          float sigma2=2.0*fw*fw;
          float front=exp(-pow(dist - c*t, 2.0)/sigma2);
          float atten=exp(-uRippleDamp[i]*dist)*exp(-0.35*t);
          float ai=uRippleAmp[i]*sin(phase)*front*atten;
          float aAbs=abs(ai);
          if(aAbs>maxAbs){ maxAbs=aAbs; maxSign=(ai>=0.0)?1.0:-1.0; }
          sumSq += ai*ai;
        }
        return maxSign*sqrt(max(sumSq,0.0));
      }

      float baseWave(vec2 p){
        vec2 d1 = normalize(vec2(uBaseDir.x, uBaseDir.y));
        vec2 d2 = normalize(vec2(uBaseDir.z, uBaseDir.w));
        float ph1 = dot(p, d1) * uBaseFreq - uTime * uBaseSpeed;
        float ph2 = dot(p, d2) * (uBaseFreq * 1.12) - uTime * (uBaseSpeed * 0.84);
        return uBaseAmp * 0.5 * (sin(ph1) + sin(ph2));
      }

      float heightAt(vec2 p){ return baseWave(p) + rippleHeight(p); }

      void main(){
        vec3 pos=position;
        float h = heightAt(pos.xz);
        pos.y += h;
        vPos=pos;

        float eps=0.05;
        float hx = heightAt(pos.xz + vec2(eps, 0.0)) - heightAt(pos.xz - vec2(eps, 0.0));
        float hz = heightAt(pos.xz + vec2(0.0, eps)) - heightAt(pos.xz - vec2(0.0, eps));
        vec3 n=normalize(vec3(-hx, 2.0*eps, -hz));
        vNormalW=normalize((modelMatrix*vec4(n,0.0)).xyz);

        gl_Position = projectionMatrix * modelViewMatrix * vec4(pos,1.0);
      }
    `;

    const frag = /* glsl */ `
      precision highp float;
      varying vec3 vPos;
      varying vec3 vNormalW;
      uniform float uAlpha;

      void main(){
        vec3 deepColor = vec3(0.05, 0.10, 0.25);
        vec3 shallow   = vec3(0.25, 0.45, 0.70);

        float fresnel = pow(1.0 - max(dot(normalize(vNormalW), vec3(0.0,1.0,0.0)), 0.0), 3.0);
        vec3 col = mix(deepColor, shallow, fresnel * 0.5 + 0.35);

        float heightFactor = clamp(vPos.y * 2.0 + 0.5, 0.0, 1.0);
        col = mix(col, shallow, heightFactor * 0.30);

        float spec = pow(max(dot(normalize(vNormalW), normalize(vec3(0.2,1.0,0.3))), 0.0), 26.0);
        col += spec * 0.07;

        col = max(col, vec3(0.12));
        gl_FragColor = vec4(col, clamp(uAlpha, 0.0, 1.0));
      }
    `;

    const waterMat = new THREE.ShaderMaterial({
      vertexShader: vert,
      fragmentShader: frag,
      uniforms,
      transparent: waterAlpha < 1,
      depthWrite: waterAlpha >= 1,
    });

    const water = new THREE.Mesh(planeGeom, waterMat);
    scene.add(water);

    api.current.waterPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -WATER_Y);

    // ===== 공유 지오/머티리얼 =====
    if (!api.current.shared.init) {
      api.current.shared.geos = {
        // 컴퓨터
        monitorFrame: new THREE.BoxGeometry(0.5, 0.32, 0.05),
        monitorStand: new THREE.BoxGeometry(0.08, 0.14, 0.06),
        monitorBase: new THREE.BoxGeometry(0.22, 0.03, 0.14),
        monitorScreen: new THREE.PlaneGeometry(0.45, 0.26),

        // 자동차
        carBody: new THREE.BoxGeometry(0.7, 0.18, 0.32),
        carCabin: new THREE.BoxGeometry(0.36, 0.16, 0.3),
        carWheel: new THREE.CylinderGeometry(0.09, 0.09, 0.06, 20),

        // 안드로이드 로봇
        torsoCyl: new THREE.CylinderGeometry(0.22, 0.22, 0.38, 20),
        headSphere: new THREE.SphereGeometry(0.16, 20, 16),
        visorPlane: new THREE.PlaneGeometry(0.18, 0.07),
        shoulderCap: new THREE.SphereGeometry(0.08, 16, 12),
        upperArmCyl: new THREE.CylinderGeometry(0.06, 0.06, 0.22, 16),
        lowerArmCyl: new THREE.CylinderGeometry(0.055, 0.055, 0.20, 16),
        upperLegCyl: new THREE.CylinderGeometry(0.07, 0.07, 0.26, 16),
        lowerLegCyl: new THREE.CylinderGeometry(0.065, 0.065, 0.22, 16),
        pelvisPlate: new THREE.BoxGeometry(0.24, 0.10, 0.16),
        hand: new THREE.BoxGeometry(0.08, 0.06, 0.06),
        foot: new THREE.BoxGeometry(0.12, 0.05, 0.18),
        antennaRod: new THREE.CylinderGeometry(0.01, 0.01, 0.12, 10),
        antennaTip: new THREE.SphereGeometry(0.02, 10, 10),
        sphereJoint: new THREE.SphereGeometry(0.06, 16, 12),

        // 박스형 산업 로봇
        rBody: new THREE.BoxGeometry(0.34, 0.40, 0.24),
        rHead: new THREE.BoxGeometry(0.22, 0.16, 0.18),
        rShoulder: new THREE.CylinderGeometry(0.08, 0.08, 0.30, 16),
        rArm: new THREE.CylinderGeometry(0.05, 0.05, 0.22, 14),
        rForearm: new THREE.CylinderGeometry(0.045, 0.045, 0.22, 14),
        rClaw: new THREE.BoxGeometry(0.12, 0.04, 0.06),
        rHip: new THREE.BoxGeometry(0.22, 0.10, 0.18),
        rLeg: new THREE.CylinderGeometry(0.06, 0.06, 0.26, 14),
        rShin: new THREE.CylinderGeometry(0.055, 0.055, 0.24, 14),
        rFoot: new THREE.BoxGeometry(0.16, 0.05, 0.20),
        rEye: new THREE.CylinderGeometry(0.018, 0.018, 0.04, 10),
      };
      api.current.shared.mats = {
        blackMetal: new THREE.MeshStandardMaterial({ color: 0x22252b, metalness: 0.6, roughness: 0.35 }),
        screen:     new THREE.MeshStandardMaterial({ color: 0x66aaff, metalness: 0.0, roughness: 0.9, emissive: 0x113355, emissiveIntensity: 0.6 }),
        metalGrey:  new THREE.MeshStandardMaterial({ color: 0x9aa4ad, metalness: 0.7, roughness: 0.35 }),
        glassBlue:  new THREE.MeshStandardMaterial({ color: 0x88bfff, metalness: 0.0, roughness: 0.05, transparent: true, opacity: 0.35, emissive: 0x103040, emissiveIntensity: 0.35 }),
        tire:       new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.1, roughness: 0.95 }),
        white:      new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.2, roughness: 0.6 }),
        blue:       new THREE.MeshStandardMaterial({ color: 0x2d7df6, metalness: 0.4, roughness: 0.4 }),
        red:        new THREE.MeshStandardMaterial({ color: 0xff6b6b, metalness: 0.4, roughness: 0.35 }),
        lime:       new THREE.MeshStandardMaterial({ color: 0x7ad83b, metalness: 0.25, roughness: 0.5 }),
        teal:       new THREE.MeshStandardMaterial({ color: 0x21c1b6, metalness: 0.35, roughness: 0.45 }),
        gray:       new THREE.MeshStandardMaterial({ color: 0xbfc7cf, metalness: 0.2, roughness: 0.6 }),
        dark:       new THREE.MeshStandardMaterial({ color: 0x1c232a, metalness: 0.5, roughness: 0.5 }),
        accent:     new THREE.MeshStandardMaterial({ color: 0x29e0a9, metalness: 0.3, roughness: 0.4 }),
      };
      api.current.shared.init = true;
    }

    // ===== 드롭 생성 =====
    const drops = [];
    const DROP_SCALE = 0.55;

    // --- 안드로이드 로봇 ---
    function buildAndroid() {
      const { geos, mats } = api.current.shared;
      const g = new THREE.Group();

      const torso = new THREE.Mesh(geos.torsoCyl, mats.teal);
      const torsoTop = new THREE.Mesh(geos.shoulderCap, mats.teal);
      const torsoBot = new THREE.Mesh(geos.shoulderCap, mats.teal);
      torsoTop.position.set(0, 0.19, 0);
      torsoBot.position.set(0, -0.19, 0);
      torsoTop.scale.set(1.0, 0.6, 1.0);
      torsoBot.scale.set(1.0, 0.6, 1.0);

      const head = new THREE.Mesh(geos.headSphere, mats.teal);
      head.position.set(0, 0.46, 0);
      const visor = new THREE.Mesh(geos.visorPlane, mats.glassBlue);
      visor.position.set(0, 0.46, 0.145);
      visor.rotation.x = -0.08;

      const antL = new THREE.Mesh(geos.antennaRod, mats.accent);
      const antR = antL.clone();
      antL.position.set(-0.09, 0.58, 0); antL.rotation.z = 0.18;
      antR.position.set( 0.09, 0.58, 0); antR.rotation.z = -0.18;
      const antTipL = new THREE.Mesh(geos.antennaTip, mats.accent); antTipL.position.set(-0.11, 0.64, 0);
      const antTipR = new THREE.Mesh(geos.antennaTip, mats.accent); antTipR.position.set( 0.11, 0.64, 0);

      const sjL = new THREE.Mesh(geos.sphereJoint, mats.metalGrey);
      const sjR = sjL.clone(); sjL.position.set(-0.26, 0.26, 0); sjR.position.set(0.26, 0.26, 0);

      const uaL = new THREE.Mesh(geos.upperArmCyl, mats.teal);
      const laL = new THREE.Mesh(geos.lowerArmCyl, mats.accent);
      const uaR = uaL.clone(); const laR = laL.clone();
      [uaL, laL, uaR, laR].forEach(m => m.rotation.z = Math.PI / 2);
      uaL.position.set(-0.38, 0.26, 0); laL.position.set(-0.56, 0.26, 0);
      uaR.position.set( 0.38, 0.26, 0); laR.position.set( 0.56, 0.26, 0);

      const pelvis = new THREE.Mesh(geos.pelvisPlate, mats.accent);
      pelvis.position.set(0, 0.02, 0);

      const ulL = new THREE.Mesh(geos.upperLegCyl, mats.teal);
      const llL = new THREE.Mesh(geos.lowerLegCyl, mats.accent);
      const ulR = ulL.clone(); const llR = llL.clone();
      ulL.position.set(-0.12, -0.18, 0); llL.position.set(-0.12, -0.42, 0);
      ulR.position.set( 0.12, -0.18, 0); llR.position.set( 0.12, -0.42, 0);

      const footL = new THREE.Mesh(geos.foot, mats.dark);
      const footR = footL.clone();
      footL.position.set(-0.12, -0.52, 0.05);
      footR.position.set( 0.12, -0.52, 0.05);

      g.add(torso, torsoTop, torsoBot, head, visor, antL, antR, antTipL, antTipR,
            sjL, sjR, uaL, laL, uaR, laR, pelvis, ulL, llL, ulR, llR, footL, footR);
      return g;
    }

    // --- 박스형 산업 로봇 ---
    function buildBoxyRobot() {
      const { geos, mats } = api.current.shared;
      const g = new THREE.Group();

      const body = new THREE.Mesh(geos.rBody, mats.metalGrey);
      const head = new THREE.Mesh(geos.rHead, mats.blackMetal);
      head.position.set(0, 0.36, 0.02);

      const eyeL = new THREE.Mesh(geos.rEye, mats.glassBlue);
      const eyeR = eyeL.clone();
      [eyeL, eyeR].forEach(e => { e.rotation.x = Math.PI / 2; });
      eyeL.position.set(-0.06, 0.36, 0.12);
      eyeR.position.set( 0.06, 0.36, 0.12);

      const shaft = new THREE.Mesh(geos.rShoulder, mats.blackMetal);
      shaft.rotation.z = Math.PI / 2;
      shaft.position.set(0, 0.24, 0);

      const armL = new THREE.Mesh(geos.rArm, mats.blue);
      const armR = armL.clone();
      [armL, armR].forEach(a => a.rotation.z = Math.PI / 2);
      armL.position.set(-0.30, 0.24, 0);
      armR.position.set( 0.30, 0.24, 0);

      const foreL = new THREE.Mesh(geos.rForearm, mats.red);
      const foreR = foreL.clone();
      [foreL, foreR].forEach(a => a.rotation.z = Math.PI / 2);
      foreL.position.set(-0.48, 0.24, 0);
      foreR.position.set( 0.48, 0.24, 0);

      const clawL = new THREE.Mesh(geos.rClaw, mats.blackMetal);
      const clawR = clawL.clone();
      clawL.position.set(-0.60, 0.24, 0.02);
      clawR.position.set( 0.60, 0.24, 0.02);

      const hip = new THREE.Mesh(geos.rHip, mats.blackMetal);
      hip.position.set(0, 0.02, 0);

      const legL = new THREE.Mesh(geos.rLeg, mats.blue);
      const shinL = new THREE.Mesh(geos.rShin, mats.red);
      const legR = legL.clone(); const shinR = shinL.clone();
      legL.position.set(-0.10, -0.14, 0); shinL.position.set(-0.10, -0.38, 0);
      legR.position.set( 0.10, -0.14, 0); shinR.position.set( 0.10, -0.38, 0);

      const footL = new THREE.Mesh(geos.rFoot, mats.blackMetal);
      const footR = footL.clone();
      footL.position.set(-0.10, -0.50, 0.05);
      footR.position.set( 0.10, -0.50, 0.05);

      g.add(body, head, eyeL, eyeR, shaft, armL, armR, foreL, foreR, clawL, clawR,
            hip, legL, shinL, legR, shinR, footL, footR);
      return g;
    }

    // --- 컴퓨터 ---
    function buildComputer() {
      const { geos, mats } = api.current.shared;
      const g = new THREE.Group();

      const frame = new THREE.Mesh(geos.monitorFrame, mats.blackMetal);
      const stand = new THREE.Mesh(geos.monitorStand, mats.blackMetal);
      const base  = new THREE.Mesh(geos.monitorBase,  mats.blackMetal);
      const screenMesh = new THREE.Mesh(geos.monitorScreen, mats.screen);
      frame.position.set(0, 0.12, 0);
      stand.position.set(0, -0.1, 0);
      base.position.set(0, -0.18, 0);
      screenMesh.position.set(0, 0.12, 0.027);

      const tower = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.42, 0.38), mats.metalGrey);
      tower.position.set(-0.36, -0.02, 0.0);

      const keyboard = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.02, 0.12), mats.blackMetal);
      keyboard.position.set(0.0, -0.23, 0.24);
      keyboard.rotation.x = -0.18;

      g.add(frame, stand, base, screenMesh, tower, keyboard);
      return g;
    }

    // --- 자동차 ---
    function buildCar() {
      const { geos, mats } = api.current.shared;
      const g = new THREE.Group();

      const body = new THREE.Mesh(geos.carBody, mats.red);
      const cabin= new THREE.Mesh(geos.carCabin, mats.white);
      cabin.position.set(0.02, 0.16, 0);

      const windshield = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.14, 0.28), mats.glassBlue);
      windshield.position.set(0.18, 0.16, 0);
      windshield.rotation.z = -0.35;

      const backglass = windshield.clone();
      backglass.position.set(-0.12, 0.16, 0);
      backglass.rotation.z = 0.35;

      const w1 = new THREE.Mesh(geos.carWheel, mats.tire);
      const w2 = w1.clone(), w3 = w1.clone(), w4 = w1.clone();
      [w1, w2, w3, w4].forEach(w => w.rotation.z = Math.PI / 2);
      const wy = -0.10, wx = 0.26, wz = 0.16;
      w1.position.set( wx, wy,  wz);
      w2.position.set( wx, wy, -wz);
      w3.position.set(-wx, wy,  wz);
      w4.position.set(-wx, wy, -wz);

      g.add(body, cabin, windshield, backglass, w1, w2, w3, w4);
      return g;
    }

    // 카테고리별 Mesh/Group 생성
    function buildDropByCategory(category) {
      let obj;
      if (category === "android")      obj = buildAndroid();
      else if (category === "monitor") obj = buildComputer();
      else if (category === "robo")    obj = buildBoxyRobot();
      else if (category === "car")     obj = buildCar();
      else                             obj = new THREE.Group();

      obj.userData.category = category;
      obj.rotation.set(Math.random() * Math.PI, Math.random() * Math.PI, Math.random() * Math.PI);
      obj.scale.set(DROP_SCALE, DROP_SCALE, DROP_SCALE);
      return obj;
    }

    function createDropMesh(forcedCategory) {
      if (forcedCategory) return buildDropByCategory(forcedCategory);
      const keys = ["android", "monitor", "robo", "car"];
      const k = keys[Math.floor(Math.random() * keys.length)];
      return buildDropByCategory(k);
    }

    function spawnDropAt(x, z, forcedCategory) {
      const dropObj = createDropMesh(forcedCategory);
      dropObj.position.set(x, 2.5, z);
      scene.add(dropObj);

      const rotVel = new THREE.Vector3(
        (Math.random() * 0.02 + 0.01) * (Math.random() < 0.5 ? -1 : 1),
        (Math.random() * 0.02 + 0.01) * (Math.random() < 0.5 ? -1 : 1),
        (Math.random() * 0.02) * (Math.random() < 0.5 ? -1 : 1)
      );

      drops.push({
        mesh: dropObj,
        vel: new THREE.Vector3(0, -0.012, 0),
        rotVel,
        targetXZ: new THREE.Vector2(x, z),
      });
    }

    api.current.spawnDrop = (p) => { spawnDropAt(p.x, p.z); };
    api.current.spawnDropWithCategory = (p, cat) => { spawnDropAt(p.x, p.z, cat); };

    // ===== 얇은 물막 스플래시(원통 → 시트 변형, 랜덤 파형/하모닉/지터/노이즈) =====
    const sheetBaseGeo = new THREE.CylinderGeometry(1, 1, 1, 64, 1, true);

    const sheetVert = /* glsl */`
      uniform float uT;
      uniform float uR0, uRSpan, uHeight, uCurl, uWob;

      // 시간/형상 랜덤 파라미터
      uniform float uRiseA, uRiseB, uFallA, uFallB, uThickness, uThetaWarp, uAsym, uSkew, uFrontSharp, uWobFreq;

      // 각도 하모닉
      uniform vec3 uRAmp, uRFreq, uRPhase;
      uniform vec3 uHAmp, uHFreq, uHPhase;
      uniform vec3 uCAmp, uCFreq, uCPhase;

      // 씨드/지터/노이즈
      uniform float uSeed;
      uniform float uJitterAmp;
      uniform float uNoiseAmp;
      uniform float uNoiseFreq;

      varying float vRim01;
      varying float vY01;
      varying float vSpeed;

      float n3(vec3 p){
        return sin(p.x)*0.42 + sin(p.y*1.37)*0.33 + sin(p.z*1.93)*0.19;
      }

      float angProfile(vec3 A, vec3 F, vec3 P, float th){
        return 1.0 + A.x*cos(F.x*th + P.x)
                   + A.y*cos(F.y*th + P.y)
                   + A.z*cos(F.z*th + P.z);
      }

      void main(){
        vec3 pos = position; // y in [-0.5,0.5]
        float y01 = pos.y + 0.5;
        vY01 = y01;

        float rise = smoothstep(0.00, 0.35, uT);
        float fall = smoothstep(0.55, 1.00, uT);
        rise = pow(rise, max(0.2, uRiseA)) * (0.8 + 0.4*uRiseB);
        fall = pow(fall, max(0.2, uFallA)) * (0.8 + 0.4*uFallB);
        float upK = mix(rise, 1.0 - fall, step(0.55, uT));

        float theta = atan(pos.x, pos.z);
        theta += sin(theta*2.0 + uT*2.0) * uThetaWarp;

        float tFront = (0.15 + 0.85 * uT);
        tFront = smoothstep(0.0, 1.0, tFront);
        tFront = pow(tFront, mix(0.6, 1.6, uFrontSharp));

        float ringR = uR0 + uRSpan * tFront;

        // --- 지터/저주파 노이즈 ---
        float jitter = sin(theta * uNoiseFreq + uSeed*6.28318) *
                       sin(theta * (uNoiseFreq*0.5 + 0.7) + uSeed*3.11) * uJitterAmp;

        float wobLobe = cos(theta*3.0 + uSeed*1.7)*0.5 + cos(theta*5.0 + uSeed*0.9)*0.5;
        jitter += wobLobe * (uNoiseAmp*0.5);
        ringR *= (1.0 + jitter);

        // 반지름 하모닉 + 비대칭
        float rMod = angProfile(uRAmp, uRFreq, uRPhase, theta);
        ringR *= clamp(rMod, 0.75, 1.25);
        ringR += cos(theta) * uAsym;

        // 높이 하모닉
        float bell = pow(1.0 - pow(y01, 1.35), 0.9);
        float h = uHeight * (0.6 + 0.4*upK) * (1.0 - 0.15*bell);
        float hMod = angProfile(uHAmp, uHFreq, uHPhase, theta);
        h *= clamp(hMod, 0.80, 1.20);

        // 림 말림(컬) 하모닉
        float cMod = angProfile(uCAmp, uCFreq, uCPhase, theta);
        float rimCurl = uCurl * pow(y01, 2.0) * clamp(cMod, 0.85, 1.15);

        float rr = ringR + (y01 - 0.5) * uThickness;
        float skewOff = (y01 - 0.5) * uSkew;

        float wob = n3(vec3(theta*uWobFreq, y01*3.0 + uT*2.0, uT*1.3)) * uWob;
        float yExtra = rimCurl * y01 * (0.6 + 0.4*sin(theta*2.0 + skewOff)) + wob*0.08;

        vec3 p;
        p.x = sin(theta) * rr;
        p.z = cos(theta) * rr;
        p.y = y01 * h + yExtra;

        vRim01 = smoothstep(ringR - 0.02, ringR + 0.02, rr);
        vSpeed = upK;

        gl_Position = projectionMatrix * modelViewMatrix * vec4(p, 1.0);
      }
    `;

    const sheetFrag = /* glsl */`
      precision highp float;
      uniform float uOpacity;
      uniform vec3  uTint;
      varying float vRim01;
      varying float vY01;
      varying float vSpeed;

      void main(){
        float rimGlow = pow(vRim01, 2.0);
        float midBand = smoothstep(0.15, 0.55, vY01) * smoothstep(0.95, 0.55, vY01);
        float speedBoost = mix(0.6, 1.2, vSpeed);

        float a = (0.05 + 0.20*rimGlow + 0.10*midBand) * speedBoost;
        vec3 col = mix(vec3(0.80,0.90,1.00), uTint, 0.5);

        gl_FragColor = vec4(col, a * uOpacity);
        if (gl_FragColor.a < 0.02) discard;
      }
    `;

    const sheetMatBase = new THREE.ShaderMaterial({
      vertexShader: sheetVert,
      fragmentShader: sheetFrag,
      transparent: true,
      depthWrite: false,
      uniforms: {
        uT:      { value: 0 },
        uR0:     { value: 0.05 },
        uRSpan:  { value: 0.50 },
        uHeight: { value: 0.65 },
        uCurl:   { value: 0.20 },
        uWob:    { value: 0.05 },
        uOpacity:{ value: 0.28 },
        uTint:   { value: new THREE.Color(0xcfe4ff) },

        // 랜덤 파형 기본값
        uRiseA:  { value: 1.0 },
        uRiseB:  { value: 0.0 },
        uFallA:  { value: 1.0 },
        uFallB:  { value: 0.0 },
        uThickness:  { value: 0.02 },
        uThetaWarp:  { value: 0.0 },
        uAsym:       { value: 0.0 },
        uSkew:       { value: 0.0 },
        uFrontSharp: { value: 1.0 },
        uWobFreq:    { value: 2.0 },

        // 하모닉 기본값
        uRAmp:   { value: new THREE.Vector3(0,0,0) },
        uRFreq:  { value: new THREE.Vector3(1,2,3) },
        uRPhase: { value: new THREE.Vector3(0,0,0) },

        uHAmp:   { value: new THREE.Vector3(0,0,0) },
        uHFreq:  { value: new THREE.Vector3(1,2,3) },
        uHPhase: { value: new THREE.Vector3(0,0,0) },

        uCAmp:   { value: new THREE.Vector3(0,0,0) },
        uCFreq:  { value: new THREE.Vector3(1,2,3) },
        uCPhase: { value: new THREE.Vector3(0,0,0) },

        // 씨드/지터/노이즈 기본값
        uSeed:      { value: Math.random() },
        uJitterAmp: { value: 0.0 },
        uNoiseAmp:  { value: 0.0 },
        uNoiseFreq: { value: 2.0 },
      }
    });

    function getSheetMesh() {
      if (api.current.columnPool.length > 0) return api.current.columnPool.pop();
      const mat = sheetMatBase.clone();
      const mesh = new THREE.Mesh(sheetBaseGeo, mat);
      mesh.visible = false;
      scene.add(mesh);
      return mesh;
    }

    // 얇은 물막 스플래시 (작고 낮게) + 랜덤 파형/하모닉/지터/노이즈
    function spawnColumnAt(x, y, z, energy = 1.0) {
      const mesh = getSheetMesh();
      mesh.position.set(x, y, z);
      mesh.visible = true;

      // 시트 자체 타원/회전 랜덤화
      const sx = 0.88 + Math.random()*0.35;
      const sz = 0.88 + Math.random()*0.35;
      mesh.scale.set(sx, 1, sz);
      mesh.rotation.y = Math.random() * Math.PI * 2;

      const e = THREE.MathUtils.clamp(energy, 0.5, 1.6);
      const lerp = (a,b,t)=>a+(b-a)*t;
      const t = (e - 0.5) / (1.6 - 0.5);

      const u = mesh.material.uniforms;
      u.uT.value = 0;

      // 작은 범위 + 소폭 랜덤 가중치
      const r0Var     = (Math.random() - 0.5) * 0.02;
      const rSpanVar  = (Math.random() - 0.5) * 0.04;
      const heightVar = (Math.random() - 0.5) * 0.05;
      const curlVar   = (Math.random() - 0.5) * 0.04;
      const wobVar    = (Math.random() - 0.5) * 0.02;
      const opaVar    = (Math.random() - 0.5) * 0.04;

      u.uR0.value     = lerp(0.01, 0.05, t) + r0Var;
      u.uRSpan.value  = lerp(0.10, 0.20, t) + rSpanVar;
      u.uHeight.value = lerp(0.10, 0.25, t) + heightVar;
      u.uCurl.value   = lerp(0.12, 0.28, t) + curlVar;
      u.uWob.value    = lerp(0.03, 0.06, t) + wobVar;
      u.uOpacity.value= lerp(0.10, 0.22, t) + opaVar;

      // 각도 하모닉 랜덤 세팅
      const rnd = (min, max) => min + Math.random()*(max-min);
      const rndi = (min, max) => Math.floor(rnd(min, max+1));

      u.uRAmp.value.set( rnd(0.03,0.10), rnd(0.02,0.07), rnd(0.02,0.06) );
      u.uRFreq.value.set( rndi(1,4),      rndi(2,5),      rndi(1,3) );
      u.uRPhase.value.set( rnd(0,Math.PI*2), rnd(0,Math.PI*2), rnd(0,Math.PI*2) );

      u.uHAmp.value.set( rnd(0.02,0.06), rnd(0.02,0.05), rnd(0.01,0.04) );
      u.uHFreq.value.set( rndi(1,3),     rndi(1,4),      rndi(2,5) );
      u.uHPhase.value.set( rnd(0,Math.PI*2), rnd(0,Math.PI*2), rnd(0,Math.PI*2) );

      u.uCAmp.value.set( rnd(0.00,0.04), rnd(0.00,0.03), rnd(0.00,0.02) );
      u.uCFreq.value.set( rndi(1,3),     rndi(1,4),      rndi(2,5) );
      u.uCPhase.value.set( rnd(0,Math.PI*2), rnd(0,Math.PI*2), rnd(0,Math.PI*2) );

      // 씨드/지터/노이즈(샷마다 다르게)
      u.uSeed.value      = Math.random();
      u.uJitterAmp.value = 0.05 + Math.random()*0.10;
      u.uNoiseAmp.value  = 0.03 + Math.random()*0.07;
      u.uNoiseFreq.value = 1.5 + Math.random()*2.5;

      const life = lerp(0.26, 0.52, t);
      api.current.columnsActive.push({ mesh, t: 0, life });
    }

    // ===== Real Droplets: Points + Shader (원형 소프트 엣지, 라이프 페이드) =====
    const MAX_DROPS = 120;
    const dPos   = new Float32Array(MAX_DROPS * 3);
    const dVel   = new Float32Array(MAX_DROPS * 3);
    const dLife  = new Float32Array(MAX_DROPS);
    const dAlive = new Uint8Array(MAX_DROPS);
    const dSize  = new Float32Array(MAX_DROPS);
    const dAlpha = new Float32Array(MAX_DROPS);

    const dropletsGeo = new THREE.BufferGeometry();
    dropletsGeo.setAttribute("position", new THREE.BufferAttribute(dPos, 3));
    dropletsGeo.setAttribute("aSize",    new THREE.BufferAttribute(dSize, 1));
    dropletsGeo.setAttribute("aAlpha",   new THREE.BufferAttribute(dAlpha, 1));

    // 초기엔 다 오프스크린
    for (let i = 0; i < MAX_DROPS; i++) {
      dPos[i*3+0] = dPos[i*3+1] = dPos[i*3+2] = 1e9;
      dSize[i] = 1.0;
      dAlpha[i] = 0.0;
    }

    const dropletVert = /* glsl */ `
      attribute float aSize;
      attribute float aAlpha;
      varying float vAlpha;
      void main() {
        vAlpha = aAlpha;
        vec4 mv = modelViewMatrix * vec4(position, 1.0);
        // 원근 감쇠
        float ps = aSize * (1.0 / max(0.0001, -mv.z));
        gl_PointSize = ps;
        gl_Position = projectionMatrix * mv;
      }
    `;
    const dropletFrag = /* glsl */ `
      precision highp float;
      varying float vAlpha;
      void main(){
        // 원형 소프트 엣지
        vec2 uv = gl_PointCoord - vec2(0.5);
        float d = length(uv);
        float soft = smoothstep(0.5, 0.45, d); // 0(바깥) → 1(안쪽)
        vec3 col = vec3(0.90, 0.98, 1.0);
        gl_FragColor = vec4(col, vAlpha * soft);
        if (gl_FragColor.a < 0.02) discard;
      }
    `;
    const dropletsMat = new THREE.ShaderMaterial({
      vertexShader: dropletVert,
      fragmentShader: dropletFrag,
      transparent: true,
      depthWrite: false,
      blending: THREE.NormalBlending,
    });
    const dropletsPoints = new THREE.Points(dropletsGeo, dropletsMat);
    scene.add(dropletsPoints);
    api.current.dropletsPoints = dropletsPoints;
    api.current.dropletsData = { dPos, dVel, dLife, dAlive, dSize, dAlpha };

    function emitDroplets(x, y, z, energy = 1.0) {
      const { dPos, dVel, dLife, dAlive, dSize, dAlpha } = api.current.dropletsData;
      const e = THREE.MathUtils.clamp(energy, 0.5, 1.4);

      // --- 1) 링 물방울(낮게, 옆으로) ---
      const ringCount = 6 + Math.floor(Math.random() * 4); // 6~9
      for (let n = 0; n < ringCount; n++) {
        let idx = -1;
        for (let i = 0; i < MAX_DROPS; i++) { if (!dAlive[i]) { idx = i; break; } }
        if (idx === -1) break;

        const theta = Math.random() * Math.PI * 2;
        const radial = (0.010 + Math.random()*0.020) * e; // 옆으로
        const up     = (0.020 + Math.random()*0.030) * e; // 살짝 위
        dPos[idx*3+0] = x; dPos[idx*3+1] = y + 0.006; dPos[idx*3+2] = z;
        dVel[idx*3+0] = Math.cos(theta) * radial;
        dVel[idx*3+1] = up;
        dVel[idx*3+2] = Math.sin(theta) * radial;
        dLife[idx] = 0.22 + Math.random()*0.18;
        dSize[idx] = 12.0 + Math.random()*10.0; // px 단위 감각
        dAlpha[idx]= 0.95;
        dAlive[idx]= 1;
      }

      // --- 2) 제트 물방울(2방울, 위로) ---
      const jetCount = 2;
      for (let n = 0; n < jetCount; n++) {
        let idx = -1;
        for (let i = 0; i < MAX_DROPS; i++) { if (!dAlive[i]) { idx = i; break; } }
        if (idx === -1) break;

        const theta = Math.random() * Math.PI * 2;
        const radial = (0.004 + Math.random()*0.008) * e; // 거의 위
        const up     = (0.055 + Math.random()*0.045) * e; // 더 위
        dPos[idx*3+0] = x; dPos[idx*3+1] = y + 0.008; dPos[idx*3+2] = z;
        dVel[idx*3+0] = Math.cos(theta) * radial;
        dVel[idx*3+1] = up;
        dVel[idx*3+2] = Math.sin(theta) * radial;
        dLife[idx] = 0.28 + Math.random()*0.22;
        dSize[idx] = 14.0 + Math.random()*10.0;
        dAlpha[idx]= 0.98;
        dAlive[idx]= 1;
      }

      dropletsGeo.attributes.position.needsUpdate = true;
      dropletsGeo.attributes.aSize.needsUpdate = true;
      dropletsGeo.attributes.aAlpha.needsUpdate = true;
    }

    // ===== 파문 =====
    const activeIdxQueue = [];
    function addRippleAt(
      x, z,
      { amp = 0.2, waveLen = 0.4 + Math.random() * 0.16, speed = 1.1 + Math.random() * 0.25, damp = 1.05 + Math.random() * 0.25, frontW = 0.26 + Math.random() * 0.14 } = {}
    ) {
      const u = uniforms;
      let idx = u.uRippleActive.value.findIndex(v => v === 0);
      if (activeIdxQueue.length >= maxActiveRipples) {
        idx = activeIdxQueue.shift();
      } else if (idx === -1) {
        idx = 0;
      }
      u.uRipplePos.value[idx].set(x, z);
      u.uRippleStartTime.value[idx] = u.uTime.value;
      u.uRippleAmp.value[idx] = amp;
      u.uRippleWaveLen.value[idx] = waveLen;
      u.uRippleSpeed.value[idx] = speed;
      u.uRippleDamp.value[idx] = damp;
      u.uRippleFrontW.value[idx] = frontW;
      u.uRippleActive.value[idx] = 1;

      const qpos = activeIdxQueue.indexOf(idx);
      if (qpos !== -1) activeIdxQueue.splice(qpos, 1);
      activeIdxQueue.push(idx);
    }

    // 클릭 → 랜덤 4종 드롭
    function onCanvasPointerDown(e) {
      const rect = renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -(((e.clientY - rect.top) / rect.height) * 2 - 1)
      );
      api.current.raycaster.setFromCamera(ndc, camera);
      const p = api.current.clickPoint;
      if (api.current.raycaster.ray.intersectPlane(api.current.waterPlane, p)) {
        const halfX = WORLD_X * 0.5 * 0.98;
        const halfZ = WORLD_Z * 0.5 * 0.98;
        p.x = THREE.MathUtils.clamp(p.x, -halfX, halfX);
        p.z = THREE.MathUtils.clamp(p.z, -halfZ, halfZ);
        p.y = WATER_Y;

        const cats = ["android", "monitor", "robo", "car"];
        const cat = cats[Math.floor(Math.random() * cats.length)];
        spawnDropAt(p.x, p.z, cat);
      }
    }
    renderer.domElement.addEventListener("pointerdown", onCanvasPointerDown);

    // ===== 루프 =====
    const clock = new THREE.Clock();
    function animate() {
      const dt = Math.min(clock.getDelta(), 0.05);
      uniforms.uTime.value += dt;

      // 낙하 & 충돌
      for (let i = drops.length - 1; i >= 0; i--) {
        const d = drops[i];
        d.vel.y -= 0.0009; // 중력
        d.mesh.position.add(d.vel);
        d.mesh.rotation.x += d.rotVel.x;
        d.mesh.rotation.y += d.rotVel.y;
        d.mesh.rotation.z += d.rotVel.z;

        if (d.mesh.position.y <= WATER_Y + 0.02) {
          const impactSpeed = Math.abs(d.vel.y);
          const energy = THREE.MathUtils.mapLinear(impactSpeed, 0.006, 0.035, 0.6, 1.4);

          addRippleAt(d.targetXZ.x, d.targetXZ.y, {
            amp: THREE.MathUtils.clamp(0.16 + impactSpeed * 14.0, 0.16, 0.28) * (0.9 + Math.random()*0.2),
            waveLen: (0.34 + Math.random()*0.18) * (0.95 + Math.random()*0.1),
            speed: (1.00 + Math.random()*0.32) * (0.95 + Math.random()*0.1),
            damp:  (1.00 + Math.random()*0.32) * (0.95 + Math.random()*0.1),
            frontW:(0.22 + Math.random()*0.12) * (0.9 + Math.random()*0.2)
          });

          // 얇은 물막 스플래시
          spawnColumnAt(d.targetXZ.x, WATER_Y, d.targetXZ.y, energy);

          // 진짜 물방울 튀김(링 + 제트 2방울)
          emitDroplets(d.targetXZ.x, WATER_Y, d.targetXZ.y, energy);

          api.current.scene.remove(d.mesh);
          drops.splice(i, 1);
        }
      }

      // Droplets update (공기저항 + 중력, 수면 닿으면 제거)
      if (api.current.dropletsPoints) {
        const { dPos, dVel, dLife, dAlive, dSize, dAlpha } = api.current.dropletsData;

        const dragXY = Math.pow(0.985, dt / 0.016); // 프레임 독립적 약한 공기저항
        const dragZ  = dragXY;
        const g = 0.020; // 중력 가속도

        for (let i = 0; i < MAX_DROPS; i++) {
          if (!dAlive[i]) continue;

          // 수명 감소 + 알파 페이드
          dLife[i] -= dt;
          const lifeK = Math.max(0, dLife[i]);
          dAlpha[i] = Math.min(1.0, lifeK * 3.0); // 초반 0.33s내 풀알파, 이후 급 페이드

          // 공기저항
          dVel[i*3+0] *= dragXY;
          dVel[i*3+2] *= dragZ;
          dVel[i*3+1] -= g; // 중력

          // 이동
          dPos[i*3+0] += dVel[i*3+0];
          dPos[i*3+1] += dVel[i*3+1];
          dPos[i*3+2] += dVel[i*3+2];

          // 수면 충돌 → 제거(오프스크린)
          if (dPos[i*3+1] <= WATER_Y || dLife[i] <= 0) {
            dAlive[i] = 0;
            dPos[i*3+0] = dPos[i*3+1] = dPos[i*3+2] = 1e9;
            dAlpha[i] = 0.0;
          }
        }

        dropletsGeo.attributes.position.needsUpdate = true;
        dropletsGeo.attributes.aAlpha.needsUpdate = true;
        dropletsGeo.attributes.aSize.needsUpdate = true;
      }

      // 스플래시 시트 update
      if (api.current.columnsActive.length) {
        for (let i = api.current.columnsActive.length - 1; i >= 0; i--) {
          const c = api.current.columnsActive[i];
          c.t += dt;
          const mat = c.mesh.material;
          const t = c.t / Math.max(1e-3, c.life);
          mat.uniforms.uT.value = THREE.MathUtils.clamp(t, 0, 1);
          if (c.t >= c.life) {
            c.mesh.visible = false;
            api.current.columnPool.push(c.mesh);
            api.current.columnsActive.splice(i, 1);
          }
        }
      }

      api.current.renderer.render(scene, camera);
      requestAnimationFrame(animate);
    }
    animate();

    // ===== 시작 시 고정 순서 시퀀스 =====
    (function runStartupSequence() {
      const seq = ["android", "monitor", "robo", "car"];
      const halfX = WORLD_X * 0.5 * 0.8;
      const halfZ = WORLD_Z * 0.5 * 0.6;

      seq.forEach((cat, i) => {
        setTimeout(() => {
          const x = THREE.MathUtils.lerp(-halfX * 0.6, halfX * 0.6, Math.random());
          const z = THREE.MathUtils.lerp(-halfZ * 0.4, halfZ * 0.4, Math.random());
          spawnDropAt(x, z, cat);
        }, 350 + i * 420);
      });
    })();

    // 리사이즈
    function onResize() {
      width = mount.clientWidth || window.innerWidth;
      height = mount.clientHeight || window.innerHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    }
    window.addEventListener("resize", onResize);

    // 정리
    return () => {
      window.removeEventListener("resize", onResize);
      renderer.domElement.removeEventListener("pointerdown", onCanvasPointerDown);
      mount.removeChild(renderer.domElement);

      planeGeom.dispose();
      waterMat.dispose();

      sheetBaseGeo.dispose();
      sheetMatBase.dispose();

      // droplets 리소스 정리
      dropletsGeo.dispose();
      dropletsMat.dispose();

      api.current.columnsActive.forEach(c => {
        c.mesh.geometry?.dispose?.();
        c.mesh.material?.dispose?.();
      });
      api.current.columnPool.forEach(m => {
        m.geometry?.dispose?.();
        m.material?.dispose?.();
      });

      if (api.current.shared?.init) {
        Object.values(api.current.shared.geos).forEach(g => g.dispose?.());
        Object.values(api.current.shared.mats).forEach(m => m.dispose?.());
        api.current.shared.init = false;
      }

      renderer.dispose();
    };
  }, [waterAlpha, maxActiveRipples]);

  return (
    <div
      ref={mountRef}
      style={{ position: "relative", width: "100%", height: "100%", overflow: "hidden", ...style }}
    />
  );
});

export default WellRipple3D;
