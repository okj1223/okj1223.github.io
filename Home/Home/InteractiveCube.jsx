// RobotView360.jsx
import React, { useEffect, useRef } from "react";
import * as THREE from "three";

export default function RobotView360({ width = 400, height = 400 }) {
  const mountRef = useRef(null);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;

    // Renderer (투명 배경)
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(width, height);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.shadowMap.enabled = true;
    renderer.setClearColor(0x000000, 0);
    mount.appendChild(renderer.domElement);

    // Scene & Camera
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    let dist =4, rotX = 75, rotY = 30;
    const updateCam = () => {
      const phi = THREE.MathUtils.degToRad(90 - rotY);
      const theta = THREE.MathUtils.degToRad(rotX);
      camera.position.set(
        dist * Math.sin(phi) * Math.cos(theta),
        dist * Math.cos(phi),
        dist * Math.sin(phi) * Math.sin(theta)
      );
      camera.lookAt(0, 1.3, 0);
    };
    updateCam();

    // Lights
    scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 0.9));
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(5, 10, 7);
    dir.castShadow = true;
    scene.add(dir);

    // ===== Robot =====
    const robot = new THREE.Group();
    scene.add(robot);

    // Materials
    const shirtMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.45 }); // 흰 셔츠
    const limbMat  = new THREE.MeshStandardMaterial({ color: 0x2b5c97, roughness: 0.6 });
    const headMat  = new THREE.MeshStandardMaterial({ color: 0xe6f2ff, roughness: 0.35 });
    const jointMat = new THREE.MeshStandardMaterial({ color: 0x1f4068, metalness: 0.25, roughness: 0.5 });

    // Torso (셔츠)
    const torso = new THREE.Mesh(new THREE.BoxGeometry(0.9, 1.2, 0.5), shirtMat);
    torso.position.y = 1.3;
    torso.castShadow = true;
    robot.add(torso);

    // 넥타이(검정)
    const tieMat = new THREE.MeshStandardMaterial({ color: 0x000000, roughness: 0.3 });
    const tieKnot = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.05), tieMat);
    tieKnot.position.set(0, 1.9, 0.26);
    const tieBody = new THREE.Mesh(new THREE.ConeGeometry(0.12, 0.5, 4), tieMat);
    tieBody.rotation.x = Math.PI;
    tieBody.position.set(0, 1.65, 0.26);
    robot.add(tieKnot, tieBody);

    // Neck & Head
    const neck = new THREE.Group();
    neck.position.set(0, 1.9, 0);
    const head = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.6, 0.6), headMat);
    head.position.y = 0.3;
    head.castShadow = true;
    neck.add(head);
    robot.add(neck);

    // ===== 얼굴 파츠 =====
    const eyeWhiteMat = new THREE.MeshStandardMaterial({ color: 0xffffff });
    const frameMat    = new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.8, roughness: 0.35 });

    // 눈 파라미터
    const eyeY = 0.12, eyeZ = 0.295, eyeOffsetX = 0.14, eyeRadius = 0.065;

    // 왼쪽 눈: 흰자(구)
    const eyeL = new THREE.Mesh(new THREE.SphereGeometry(eyeRadius, 16, 16), eyeWhiteMat);
    eyeL.position.set(-eyeOffsetX, eyeY, eyeZ);

    // 표면 밀착 홍채/동공
    const irisMat = new THREE.MeshStandardMaterial({
      color: 0x1e90ff, roughness: 0.4, metalness: 0.1,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1
    });
    const pupilMat = new THREE.MeshStandardMaterial({
      color: 0x000000, polygonOffset: true, polygonOffsetFactor: -1.5, polygonOffsetUnits: -1
    });
    const irisRadius = 0.038, pupilRadius = 0.018;
    const insetIris = 0.0006, insetPupil = 0.0004;

    const irisL = new THREE.Mesh(new THREE.CircleGeometry(irisRadius, 24), irisMat);
    irisL.position.set(0, 0, eyeRadius - insetIris);
    const pupilL = new THREE.Mesh(new THREE.CircleGeometry(pupilRadius, 24), pupilMat);
    pupilL.position.set(0, 0, eyeRadius - insetPupil);
    eyeL.add(irisL, pupilL);

    // 오른쪽 눈
    const eyeR = eyeL.clone();
    eyeR.position.x = eyeOffsetX;

    // 코/입
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.038, 0.11, 12), new THREE.MeshStandardMaterial({ color: 0xffa978 }));
    nose.rotation.x = Math.PI / 2;
    nose.position.set(0, 0.04, 0.298);
    const mouth = new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.04, 0.02), new THREE.MeshStandardMaterial({ color: 0x7a1d1d }));
    mouth.position.set(0, -0.12, 0.29);

    // 안경(정면 평평)
    const ringGeo = new THREE.TorusGeometry(0.095, 0.012, 12, 24);
    const glassL = new THREE.Mesh(ringGeo, frameMat);
    glassL.position.set(-eyeOffsetX, eyeY, 0.292);
    const glassR = glassL.clone(); glassR.position.x = eyeOffsetX;
    const bridge = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.09, 10), frameMat);
    bridge.rotation.z = Math.PI / 2;
    bridge.position.set(0, eyeY, 0.292);

    head.add(eyeL, eyeR, nose, mouth, glassL, glassR, bridge);

    // ===== Shoulders & Arms — torso에 부착 + 소켓 연결 =====
    // ⬆ 어깨/팔 더 위로: shoulderHeight ↑
    const shoulderHeight = 0.42;   // (기존 0.30) -> 더 위로 올림
    const torsoHalfW     = 0.45;   // 0.9 / 2
    const pivotOutset    = 0.17;   // 몸통 옆면에서 바깥으로
    const pivotX         = torsoHalfW + pivotOutset;

    // 피벗(어깨 그룹) - torso의 자식
    const shoulderL = new THREE.Group(); shoulderL.position.set(-pivotX, shoulderHeight, 0);
    const shoulderR = new THREE.Group(); shoulderR.position.set( pivotX, shoulderHeight, 0);

    // 팔(기존 길이 그대로, 위치만 상승에 따라 같이 올라감)
    const armGeo = new THREE.BoxGeometry(0.16, 0.64, 0.16);
    const armL = new THREE.Mesh(armGeo, limbMat); armL.position.y = -0.32; armL.castShadow = true;
    const armR = armL.clone();
    shoulderL.add(armL); shoulderR.add(armR);

    // 어깨 볼조인트
    const jointRadius = 0.11;
    const jointL = new THREE.Mesh(new THREE.SphereGeometry(jointRadius, 24, 16), jointMat);
    const jointR = jointL.clone();
    jointL.position.copy(shoulderL.position);
    jointR.position.copy(shoulderR.position);
    jointL.castShadow = jointR.castShadow = true;

    // 몸통-어깨 소켓(실린더) — 높이 자동 반영
    const socketMat = shirtMat;
    const socketRadius = 0.06;
    const socketLen = Math.abs(pivotX - torsoHalfW); // ≈ 0.17
    const socketL = new THREE.Mesh(new THREE.CylinderGeometry(socketRadius, socketRadius, socketLen, 16), socketMat);
    socketL.rotation.z = Math.PI / 2;
    socketL.position.set(-(torsoHalfW + pivotOutset / 2), shoulderHeight, 0);
    const socketR = socketL.clone();
    socketR.position.x = (torsoHalfW + pivotOutset / 2);

    torso.add(shoulderL, shoulderR, jointL, jointR, socketL, socketR);

    // ===== Legs (차렷 고정) — ⬇ 다리 더 길게
    const hipL = new THREE.Group(); hipL.position.set(-0.28, 0.95, 0);
    const hipR = new THREE.Group(); hipR.position.set( 0.28, 0.95, 0);
    const legGeo = new THREE.BoxGeometry(0.22, 0.9, 0.22);      // ← 길이 0.7 -> 0.9
    const legL = new THREE.Mesh(legGeo, limbMat); legL.position.y = -0.45; // 중심 보정
    const legR = legL.clone();
    hipL.add(legL); hipR.add(legR);
    robot.add(hipL, hipR);

    // ===== Interaction =====
    let dragging = false, lastX = 0, lastY = 0;
    const onDown = (x, y) => { dragging = true; lastX = x; lastY = y; };
    const onMove = (x, y) => {
      if (!dragging) return;
      const dx = x - lastX, dy = y - lastY;
      lastX = x; lastY = y;
      rotX += dx * 0.5;
      rotY = Math.max(-80, Math.min(80, rotY + dy * 0.5));
      updateCam();
    };
    const onUp = () => { dragging = false; };

    mount.addEventListener("mousedown", e => onDown(e.clientX, e.clientY));
    window.addEventListener("mousemove", e => onMove(e.clientX, e.clientY));
    window.addEventListener("mouseup", onUp);
    mount.addEventListener("touchstart", e => onDown(e.touches[0].clientX, e.touches[0].clientY));
    window.addEventListener("touchmove", e => onMove(e.touches[0].clientX, e.touches[0].clientY), { passive: false });
    window.addEventListener("touchend", onUp);
    mount.addEventListener("wheel", e => { dist = THREE.MathUtils.clamp(dist + e.deltaY * 0.01, 2, 10); updateCam(); });

    // ===== Animation (자연스러운 4동작) =====
    const { degToRad, clamp, lerp } = THREE.MathUtils;
    const ease = (t) => 0.5 - 0.5 * Math.cos(Math.PI * t);
    const deg = (d) => degToRad(d);
    const DUR = { tPose: 0.9, up: 1.0, swing: 1.8, rest: 0.9 };

    const makeTween = (getTarget) => {
      let fromL = { x: 0, z: 0 }, fromR = { x: 0, z: 0 };
      let toL = { x: 0, z: 0 }, toR = { x: 0, z: 0 };
      return {
        init: () => {
          fromL = { x: shoulderL.rotation.x, z: shoulderL.rotation.z };
          fromR = { x: shoulderR.rotation.x, z: shoulderR.rotation.z };
          const t = getTarget();
          toL = t.L; toR = t.R;
        },
        apply: (p) => {
          const t = ease(p);
          shoulderL.rotation.x = lerp(fromL.x, toL.x, t);
          shoulderL.rotation.z = lerp(fromL.z, toL.z, t);
          shoulderR.rotation.x = lerp(fromR.x, toR.x, t);
          shoulderR.rotation.z = lerp(fromR.z, toR.z, t);
        }
      };
    };

    // 1) 가로 벌리기(T-포즈)
    const actionTPose = makeTween(() => ({
      L: { x: shoulderL.rotation.x, z: -deg(90) },
      R: { x: shoulderR.rotation.x, z:  deg(90) },
    }));

    // 2) 팔 위로 올리기(머리 위)
    const upAngle = -deg(160);
    const actionArmsUp = makeTween(() => ({
      L: { x: upAngle, z: 0 },
      R: { x: upAngle, z: 0 },
    }));

    // 3) 양손 앞/뒤 스윙(연속 위상)
    const actionSwing = (() => {
      let base = upAngle, amp = deg(42), phase = 0;
      return {
        init: () => {
          const cur = shoulderL.rotation.x;
          const s = clamp((cur - base) / amp, -1, 1);
          phase = Math.asin(s);
        },
        apply: (p) => {
          const s = Math.sin(p * Math.PI * 2 + phase);
          shoulderL.rotation.x = base + s * amp;
          shoulderR.rotation.x = base - s * amp;
          shoulderL.rotation.z = 0;
          shoulderR.rotation.z = 0;
        }
      };
    })();

    // 4) 차렷
    const actionRest = makeTween(() => ({
      L: { x: 0, z: 0 },
      R: { x: 0, z: 0 },
    }));

    const routine = [
      { duration: DUR.tPose, init: actionTPose.init, apply: actionTPose.apply },
      { duration: DUR.up,    init: actionArmsUp.init, apply: actionArmsUp.apply },
      { duration: DUR.swing, init: actionSwing.init,  apply: actionSwing.apply },
      { duration: DUR.rest,  init: actionRest.init,   apply: actionRest.apply },
    ];

    let idx = 0, elapsed = 0, last = performance.now() / 1000;
    routine[0].init && routine[0].init();

    const animate = () => {
      const now = performance.now() / 1000;
      const dt = Math.min(0.033, now - last);
      last = now;

      const cur = routine[idx];
      elapsed += dt;
      const p = clamp(elapsed / cur.duration, 0, 1);
      cur.apply(p);

      if (elapsed >= cur.duration) {
        idx = (idx + 1) % routine.length;
        elapsed = 0;
        routine[idx].init && routine[idx].init();
      }

      renderer.render(scene, camera);
      requestAnimationFrame(animate);
    };
    animate();

    // cleanup
    return () => {
      mount.removeChild(renderer.domElement);
      renderer.dispose();
    };
  }, [width, height]);

  return (
    <div
      ref={mountRef}
      style={{
        width,
        height,
        background: "transparent",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
      }}
    />
  );
}
