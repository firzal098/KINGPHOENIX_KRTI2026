<script>
  import { onMount, onDestroy } from 'svelte';
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import {
    dronePose,
    fcuState,
    refinedGatePoses,
    targetGateIndex,
    targetSubGateIndex,
    targetGateLabel,
    previewGateIndex,
    previewSubGateIndex,
    setTargetGate,
    callResetGates,
    addToast
  } from '../ros.js';

  let containerEl;
  let canvasEl;

  let scene;
  let camera;
  let renderer;
  let controls;
  let animFrameId;

  let droneGroup;
  let gridHelper;
  let rotorMeshes = [];
  let gateGroups = [];
  let trailLine;
  let trailPositions = [];
  let lastTrailX = null, lastTrailY = null, lastTrailZ = null;

  // Gate estimation correction trails (tracks how estimator moves each gate over time)
  let gateTrailLines = [];
  let gateTrailPositions = [[], [], [], [], []];
  let lastGatePos = [null, null, null, null, null];
  let priorOriginMarkers = [];

  let cameraMode = 'orbit'; // 'orbit' | 'chase' | 'topdown'
  let autoFollow = false;

  // Default gate priors if estimator hasn't streamed refined poses yet
  const defaultPriorsRDF = [
    { id: 1, x: 0.41, y: -0.75, z: 29.33, nx: 0.0, ny: 0.0, nz: 1.0 },
    { id: 2, x: 5.46, y: -0.75, z: 19.31, nx: 0.0, ny: 0.0, nz: 1.0 },
    { id: 3, x: 9.49, y: -0.75, z: 10.51, nx: 1.0, ny: 0.0, nz: 0.0 },
    { id: 4, x: 12.41, y: -0.75, z: 12.43, nx: 0.0, ny: 0.0, nz: 1.0 },
    { id: 5, x: 17.47, y: -0.75, z: 29.38, nx: 0.0, ny: 0.0, nz: 1.0 }
  ];

  function enuToThree(x, y, z) {
    // ROS ENU: X=East (+X), Y=North (+Y), Z=Up (+Z)
    // Three.js: X=Right (+X), Y=Up (+Y), Z=South (+Z, North is -Z)
    return {
      x: x,
      y: z,
      z: -y
    };
  }

  function createTextSprite(text, color = '#ffffff') {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    ctx.fillStyle = 'rgba(10, 16, 26, 0.85)';
    ctx.strokeStyle = color;
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.roundRect(8, 8, 240, 48, 10);
    ctx.fill();
    ctx.stroke();

    ctx.font = 'bold 24px monospace';
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 128, 32);

    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    const spriteMat = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.scale.set(1.4, 0.35, 1);
    return sprite;
  }

  function buildDroneMesh() {
    const group = new THREE.Group();

    // Central Fuselage / Carbon Core
    const bodyGeo = new THREE.BoxGeometry(0.35, 0.08, 0.35);
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x161d2b,
      metalness: 0.8,
      roughness: 0.2
    });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.castShadow = true;
    group.add(body);

    // Center Top Dome / Flight Controller Box
    const domeGeo = new THREE.CylinderGeometry(0.08, 0.1, 0.06, 16);
    const domeMat = new THREE.MeshStandardMaterial({
      color: 0x00f0ff,
      emissive: 0x005577,
      metalness: 0.5,
      roughness: 0.3
    });
    const dome = new THREE.Mesh(domeGeo, domeMat);
    dome.position.y = 0.06;
    group.add(dome);

    // Front / Nose Direction Arrow (Points in +X forward along track)
    const noseGeo = new THREE.ConeGeometry(0.06, 0.16, 8);
    const noseMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const nose = new THREE.Mesh(noseGeo, noseMat);
    nose.rotation.z = -Math.PI / 2; // Point cone tip along +X
    nose.position.set(0.24, 0.02, 0);
    group.add(nose);

    // 4 Carbon X-Arms
    const armGeo = new THREE.CylinderGeometry(0.015, 0.015, 0.45, 8);
    const armMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.9, roughness: 0.1 });

    const arm1 = new THREE.Mesh(armGeo, armMat);
    arm1.rotation.z = Math.PI / 2;
    arm1.rotation.y = Math.PI / 4;
    group.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, armMat);
    arm2.rotation.z = Math.PI / 2;
    arm2.rotation.y = -Math.PI / 4;
    group.add(arm2);

    // 4 Motors & Spinning Propeller Disks (Front at +X, Rear at -X)
    const motorOffsets = [
      { x: 0.16, z: -0.16, isFront: true },  // Front-Left
      { x: 0.16, z: 0.16, isFront: true },   // Front-Right
      { x: -0.16, z: -0.16, isFront: false }, // Rear-Left
      { x: -0.16, z: 0.16, isFront: false }  // Rear-Right
    ];

    rotorMeshes = [];

    motorOffsets.forEach((pos) => {
      // Motor mount
      const motorGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.04, 12);
      const motorMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8 });
      const motor = new THREE.Mesh(motorGeo, motorMat);
      motor.position.set(pos.x, 0.03, pos.z);
      group.add(motor);

      // Propeller Blades (2-blade)
      const propGeo = new THREE.BoxGeometry(0.24, 0.005, 0.025);
      const propMat = new THREE.MeshBasicMaterial({
        color: pos.isFront ? 0x00f0ff : 0xf59e0b,
        transparent: true,
        opacity: 0.75
      });
      const prop = new THREE.Mesh(propGeo, propMat);
      prop.position.set(pos.x, 0.055, pos.z);
      group.add(prop);
      rotorMeshes.push(prop);
    });

    return group;
  }

  function buildGateMesh(id, subIndex = 0, isSubgate = false, offset = 0.0) {
    const group = new THREE.Group();
    group.name = isSubgate ? `gate_${id}_sub_${subIndex}` : `gate_${id}`;

    const gateWidth = 2.0;
    const gateHeight = 2.0;
    const beamThick = isSubgate ? 0.08 : 0.1;

    const frameMat = new THREE.MeshStandardMaterial({
      color: isSubgate ? 0x38bdf8 : 0x00f0ff,
      metalness: 0.6,
      roughness: 0.3,
      emissive: isSubgate ? 0x001a2e : 0x002233
    });

    // Top Beam (Width along Z, so opening faces +X)
    const beamGeo = new THREE.BoxGeometry(beamThick, beamThick, gateWidth);
    const topBeam = new THREE.Mesh(beamGeo, frameMat);
    topBeam.position.y = gateHeight / 2;
    group.add(topBeam);

    // Bottom Beam
    const botBeam = new THREE.Mesh(beamGeo, frameMat);
    botBeam.position.y = -gateHeight / 2;
    group.add(botBeam);

    // Column 1 (at -Z/2)
    const colGeo = new THREE.BoxGeometry(beamThick, gateHeight, beamThick);
    const leftCol = new THREE.Mesh(colGeo, frameMat);
    leftCol.position.z = -gateWidth / 2;
    group.add(leftCol);

    // Column 2 (at +Z/2)
    const rightCol = new THREE.Mesh(colGeo, frameMat);
    rightCol.position.z = gateWidth / 2;
    group.add(rightCol);

    // Outer Target Pulse Ring (Oriented in YZ plane, facing along +X)
    const ringGeo = new THREE.RingGeometry(1.3, 1.4, 32);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.4
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.name = 'target_ring';
    ring.rotation.y = Math.PI / 2;
    ring.visible = false;
    group.add(ring);

    // Sprite Label above gate
    let labelText = isSubgate ? `GATE #${id}.${subIndex} (SUB)` : `GATE #${id}`;
    if (id === 4 && subIndex === 4) {
      labelText = 'VIRTUAL GATE 4 (+6m)';
    } else if (id === 5 && isSubgate) {
      labelText = 'VIRTUAL EXIT (+3m)';
    }
    const labelColor = isSubgate ? '#38bdf8' : '#00f0ff';
    const label = createTextSprite(labelText, labelColor);
    label.name = 'gate_label';
    label.position.set(0, gateHeight / 2 + (isSubgate ? 0.35 : 0.45), 0);
    group.add(label);

    return { group, frameMat, ring, label, mainId: id, subIndex, isSubgate, offset };
  }

  function initThree() {
    if (!canvasEl || !containerEl) return;

    const width = containerEl.clientWidth || 640;
    const height = containerEl.clientHeight || 420;

    // 1. Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x060913);
    scene.fog = new THREE.FogExp2(0x060913, 0.015);

    // 2. Camera
    camera = new THREE.PerspectiveCamera(55, width / height, 0.1, 200);
    camera.position.set(-6, 5, 8);

    // 3. Renderer
    renderer = new THREE.WebGLRenderer({
      canvas: canvasEl,
      antialias: true,
      powerPreference: 'high-performance'
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // 4. Orbit Controls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 + 0.05;
    controls.minDistance = 1;
    controls.maxDistance = 80;
    controls.target.set(0, 1, 0);

    // 5. Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(10, 20, 10);
    scene.add(dirLight);

    const cyanRim = new THREE.PointLight(0x00f0ff, 1.5, 30);
    cyanRim.position.set(-10, 5, -10);
    scene.add(cyanRim);

    // 6. Ground Grid (Neon Sci-Fi Arena)
    gridHelper = new THREE.GridHelper(80, 80, 0x00f0ff, 0x1e293b);
    gridHelper.position.y = 0;
    gridHelper.material.opacity = 0.4;
    gridHelper.material.transparent = true;
    scene.add(gridHelper);

    // 7. Drone 3D Mesh
    droneGroup = buildDroneMesh();
    scene.add(droneGroup);

    // 8. Gate & Sub-Gate Meshes
    gateGroups = [];

    // Gate 1: Main (sub 0)
    const g1 = buildGateMesh(1, 0, false, 0.0);
    scene.add(g1.group);
    gateGroups.push(g1);

    // Gate 2: Main (sub 0)
    const g2 = buildGateMesh(2, 0, false, 0.0);
    scene.add(g2.group);
    gateGroups.push(g2);

    // Gate 3: Main (sub 0) + Sub-Gate 3.1 (sub 1, offset 1.0m) + Sub-Gate 3.2 (sub 2, offset 2.5m)
    const g3 = buildGateMesh(3, 0, false, 0.0);
    scene.add(g3.group);
    gateGroups.push(g3);

    const g3_sub1 = buildGateMesh(3, 1, true, 1.0);
    scene.add(g3_sub1.group);
    gateGroups.push(g3_sub1);

    const g3_sub2 = buildGateMesh(3, 2, true, 2.5);
    scene.add(g3_sub2.group);
    gateGroups.push(g3_sub2);

    // Gate 4: Main (sub 0) + Sub-Gate 4.1 (sub 1, 1m) + Sub-Gate 4.2 (sub 2, 2m) + Sub-Gate 4.3 (sub 3, 3m) + Virtual Gate (+6m)
    const g4 = buildGateMesh(4, 0, false, 0.0);
    scene.add(g4.group);
    gateGroups.push(g4);

    const g4_sub1 = buildGateMesh(4, 1, true, 1.0);
    scene.add(g4_sub1.group);
    gateGroups.push(g4_sub1);

    const g4_sub2 = buildGateMesh(4, 2, true, 2.0);
    scene.add(g4_sub2.group);
    gateGroups.push(g4_sub2);

    const g4_sub3 = buildGateMesh(4, 3, true, 3.0);
    scene.add(g4_sub3.group);
    gateGroups.push(g4_sub3);

    const g4_virtual = buildGateMesh(4, 4, true, 6.0);
    scene.add(g4_virtual.group);
    gateGroups.push(g4_virtual);

    // Gate 5: Main (sub 0) + Virtual Exit Ghost Gate (sub 1, offset 3.0m)
    const g5 = buildGateMesh(5, 0, false, 0.0);
    scene.add(g5.group);
    gateGroups.push(g5);

    const g5_exit = buildGateMesh(5, 1, true, 3.0);
    scene.add(g5_exit.group);
    gateGroups.push(g5_exit);

    // 9. Permanent Flight Trajectory Ribbon (Cyan Line)
    const trailGeo = new THREE.BufferGeometry();
    const trailMat = new THREE.LineBasicMaterial({
      color: 0x00f0ff,
      transparent: true,
      opacity: 0.95,
      linewidth: 3,
      depthWrite: false
    });
    trailLine = new THREE.Line(trailGeo, trailMat);
    trailLine.frustumCulled = false;
    trailLine.renderOrder = 10;
    scene.add(trailLine);

    // 10. Gate Estimation Correction History Trails (5 distinct gate paths)
    const gateColors = [0x10b981, 0xf59e0b, 0xa855f7, 0x06b6d4, 0xec4899];
    gateTrailLines = [];
    gateTrailPositions = [[], [], [], [], []];
    lastGatePos = [null, null, null, null, null];
    priorOriginMarkers = [];

    for (let i = 0; i < 5; i++) {
      // Gate Estimation Movement Line
      const gGeo = new THREE.BufferGeometry();
      const gMat = new THREE.LineBasicMaterial({
        color: gateColors[i],
        transparent: true,
        opacity: 0.95,
        linewidth: 3,
        depthWrite: false
      });
      const gLine = new THREE.Line(gGeo, gMat);
      gLine.frustumCulled = false;
      gLine.renderOrder = 10;
      scene.add(gLine);
      gateTrailLines.push(gLine);

      // Prior Origin Anchor Marker (Ghost wireframe octahedron where gate prior started)
      const pGeo = new THREE.OctahedronGeometry(0.18, 0);
      const pMat = new THREE.MeshBasicMaterial({
        color: gateColors[i],
        wireframe: true,
        transparent: true,
        opacity: 0.6
      });
      const pMarker = new THREE.Mesh(pGeo, pMat);
      pMarker.visible = false;
      scene.add(pMarker);
      priorOriginMarkers.push(pMarker);
    }

    updateGatesFromPriorsOrPoses();

    // Start Animation Loop
    animate();
  }

  // Initial Drone Position & Yaw Anchor
  let initialDronePos = null; // { x, y, z, yaw (degrees) }

  function getRelPose(x, y, z) {
    if (!initialDronePos) {
      return { x: 0, y: 0, z: 0 };
    }
    // Direct subtraction: current_pos - initial_drone_pos
    const dx = x - initialDronePos.x;
    const dy = y - initialDronePos.y;
    const dz = z - initialDronePos.z;

    // Convert ENU delta to Three.js (Three.X = East = dx, Three.Y = Up = dz, Three.Z = South = -dy)
    return {
      x: dx,
      y: dz,
      z: -dy
    };
  }

  function updateGatesFromPriorsOrPoses() {
    if (!gateGroups || gateGroups.length === 0) return;

    let poses = $refinedGatePoses;
    const currentTarget = $targetGateIndex;
    const currentSubTarget = $targetSubGateIndex;

    // 1. Compute Base World/Local Poses for 5 Main Gates
    const mainGatePoses = [];
    for (let i = 0; i < 5; i++) {
      let posX = 0, posY = 1.0, posZ = 0;
      let relGateYawRad = 0;

      if (poses && poses.length > i && initialDronePos) {
        const p = poses[i];
        const relPos = getRelPose(p.x, p.y, p.z);
        posX = relPos.x;
        posY = 1.0; // Gates centered at nominal 1.0m height
        posZ = relPos.z;
        relGateYawRad = Math.atan2(2.0 * (p.qw * p.qz + p.qx * p.qy), 1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz));
      } else {
        const p = defaultPriorsRDF[i];
        const fwd = p.z;
        const left = -p.x;
        const initYawRad = initialDronePos ? (initialDronePos.yaw * (Math.PI / 180)) : (Math.PI / 2);
        const dx = Math.cos(initYawRad) * fwd - Math.sin(initYawRad) * left;
        const dy = Math.sin(initYawRad) * fwd + Math.cos(initYawRad) * left;
        posX = dx;
        posY = 1.0;
        posZ = -dy;
        relGateYawRad = (i === 2 ? (initYawRad - Math.PI / 2) : initYawRad);
      }
      mainGatePoses.push({ posX, posY, posZ, yaw: relGateYawRad });

      // Record Gate Estimation Correction History Trail (for main gate i)
      const lastP = lastGatePos[i];
      const distSq = lastP
        ? Math.pow(posX - lastP.x, 2) + Math.pow(posY - lastP.y, 2) + Math.pow(posZ - lastP.z, 2)
        : 999;

      if (distSq > 0.0001) { // Shift > 1cm
        if (gateTrailPositions[i].length === 0) {
          gateTrailPositions[i].push(posX, posY, posZ, posX, posY, posZ);
          if (priorOriginMarkers[i]) {
            priorOriginMarkers[i].position.set(posX, posY, posZ);
            priorOriginMarkers[i].visible = true;
          }
        } else {
          gateTrailPositions[i].push(posX, posY, posZ);
        }
        lastGatePos[i] = { x: posX, y: posY, z: posZ };

        if (gateTrailLines[i] && gateTrailPositions[i].length >= 6) {
          gateTrailLines[i].geometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute(gateTrailPositions[i], 3)
          );
          gateTrailLines[i].geometry.attributes.position.needsUpdate = true;
          gateTrailLines[i].geometry.computeBoundingSphere();
        }
      }
    }

    // 2. Position and Color each Gate & Sub-Gate
    gateGroups.forEach((gate) => {
      const mainIdx = gate.mainId - 1;
      const base = mainGatePoses[mainIdx];
      const offset = gate.offset || 0.0;

      // In Three.js coordinates:
      // Forward normal vector in Three.js (Three.X = cos(yaw), Three.Z = -sin(yaw))
      const offX = offset * Math.cos(base.yaw);
      const offZ = -offset * Math.sin(base.yaw);

      const posX = base.posX + offX;
      const posY = base.posY;
      const posZ = base.posZ + offZ;

      gate.group.position.set(posX, posY, posZ);
      gate.group.rotation.set(0, base.yaw, 0);

      // Highlighting logic matching live controller preview topics:
      let currentPreviewMain = 1;
      let currentPreviewSub = 0;
      previewGateIndex.subscribe((v) => (currentPreviewMain = v))();
      previewSubGateIndex.subscribe((v) => (currentPreviewSub = v))();

      const isCurrentActive = (mainIdx === currentTarget && gate.subIndex === currentSubTarget);
      const isPreviewGate = (mainIdx === currentPreviewMain && gate.subIndex === currentPreviewSub);
      const isPassed = (mainIdx < currentTarget) || (mainIdx === currentTarget && gate.subIndex < currentSubTarget);

      if (isCurrentActive) {
        // Active Target Gate/Sub-gate -> Vibrant Emerald Green with Pulse Ring
        gate.frameMat.color.setHex(0x10b981);
        gate.frameMat.emissive.setHex(0x064e3b);
        gate.ring.visible = true;
      } else if (isPreviewGate) {
        // Next Preview Gate/Sub-gate -> Amber Gold
        gate.frameMat.color.setHex(0xf59e0b);
        gate.frameMat.emissive.setHex(0x78350f);
        gate.ring.visible = false;
      } else if (isPassed) {
        // Passed Gate/Sub-gate -> Dim Silver/Slate
        gate.frameMat.color.setHex(0x475569);
        gate.frameMat.emissive.setHex(0x0f172a);
        gate.ring.visible = false;
      } else {
        // Upcoming Gate -> Cyber Cyan
        gate.frameMat.color.setHex(gate.isSubgate ? 0x38bdf8 : 0x00f0ff);
        gate.frameMat.emissive.setHex(0x002233);
        gate.ring.visible = false;
      }
    });
  }

  function updateDrone() {
    if (!droneGroup) return;

    const { x, y, z, roll, pitch, yaw } = $dronePose;
    if (x === undefined || isNaN(x)) return;

    // Only latch when real telemetry is received (timestamp > 0)
    if ($dronePose.timestamp > 0) {
      if (initialDronePos === null || (!$fcuState.armed && initialDronePos.isDisarmed)) {
        initialDronePos = {
          x: x,
          y: y,
          z: z,
          yaw: (yaw !== undefined ? yaw : 0),
          isDisarmed: !$fcuState.armed
        };
        if (gridHelper) {
          gridHelper.rotation.y = (initialDronePos.yaw !== undefined ? initialDronePos.yaw * (Math.PI / 180) : 0);
        }
      }
    }

    // current_drone_pos - initial_drone_pos
    const relPos = getRelPose(x, y, z);
    const dispAlt = ($dronePose.rel_alt !== undefined && !isNaN($dronePose.rel_alt))
      ? Math.max(0.02, $dronePose.rel_alt)
      : Math.max(0.02, relPos.y);

    droneGroup.position.set(relPos.x, dispAlt, relPos.z);

    // Drone orientation in ENU world frame
    const yawRad = (yaw !== undefined ? yaw * (Math.PI / 180) : 0);
    const pitchRad = (pitch !== undefined ? pitch * (Math.PI / 180) : 0);
    const rollRad = (roll !== undefined ? roll * (Math.PI / 180) : 0);

    droneGroup.rotation.set(pitchRad, yawRad, -rollRad);

    // Spin Propellers
    const spinSpeed = 0.45;
    rotorMeshes.forEach((rotor, idx) => {
      rotor.rotation.y += (idx % 2 === 0 ? 1 : -1) * spinSpeed;
    });

    // Update Permanent Flight Path Trail (filters small noise < 3cm to run indefinitely without loss)
    const curX = relPos.x;
    const curY = dispAlt;
    const curZ = relPos.z;

    const distSq = (lastTrailX !== null)
      ? Math.pow(curX - lastTrailX, 2) + Math.pow(curY - lastTrailY, 2) + Math.pow(curZ - lastTrailZ, 2)
      : 999;

    if (distSq > 0.0009) {
      if (trailPositions.length === 0) {
        trailPositions.push(curX, curY, curZ, curX, curY, curZ);
      } else {
        trailPositions.push(curX, curY, curZ);
      }
      lastTrailX = curX;
      lastTrailY = curY;
      lastTrailZ = curZ;

      if (trailLine && trailPositions.length >= 6) {
        trailLine.geometry.setAttribute(
          'position',
          new THREE.Float32BufferAttribute(trailPositions, 3)
        );
        trailLine.geometry.attributes.position.needsUpdate = true;
        trailLine.geometry.computeBoundingSphere();
      }
    }

    // Camera follow modes
    if (cameraMode === 'chase' && camera && controls) {
      const dronePos = droneGroup.position.clone();

      // Backward offset vector behind drone (+X is forward, -X is backward)
      const backwardOffset = new THREE.Vector3(-2.8, 1.2, 0);
      backwardOffset.applyQuaternion(droneGroup.quaternion);

      // Target ahead of the drone along +X
      const forwardTarget = new THREE.Vector3(2.0, 0.2, 0);
      forwardTarget.applyQuaternion(droneGroup.quaternion);
      const lookTarget = dronePos.clone().add(forwardTarget);

      camera.position.lerp(dronePos.clone().add(backwardOffset), 0.15);
      camera.lookAt(lookTarget);
      controls.target.copy(dronePos);
    } else if (cameraMode === 'topdown' && camera && controls) {
      const dronePos = droneGroup.position;
      camera.position.set(dronePos.x, 25, dronePos.z);
      camera.lookAt(dronePos.x, 0, dronePos.z);
      controls.target.set(dronePos.x, 0, dronePos.z);
    }
  }

  function animate() {
    animFrameId = requestAnimationFrame(animate);

    updateDrone();
    updateGatesFromPriorsOrPoses();

    if (controls && cameraMode === 'orbit') {
      controls.update();
    }

    if (renderer && scene && camera) {
      renderer.render(scene, camera);
    }
  }

  function setCameraMode(mode) {
    cameraMode = mode;
    if (!controls || !camera || !droneGroup) return;

    if (mode === 'orbit') {
      controls.enabled = true;
    } else if (mode === 'chase') {
      controls.enabled = false;
      const dronePos = droneGroup.position.clone();
      const backwardOffset = new THREE.Vector3(-2.8, 1.2, 0);
      backwardOffset.applyQuaternion(droneGroup.quaternion);
      camera.position.copy(dronePos.clone().add(backwardOffset));
      const forwardTarget = new THREE.Vector3(2.0, 0.2, 0);
      forwardTarget.applyQuaternion(droneGroup.quaternion);
      camera.lookAt(dronePos.clone().add(forwardTarget));
      controls.target.copy(dronePos);
    } else if (mode === 'topdown') {
      controls.enabled = false;
      const dronePos = droneGroup.position;
      camera.position.set(dronePos.x, 25, dronePos.z);
      camera.lookAt(dronePos.x, 0, dronePos.z);
      controls.target.set(dronePos.x, 0, dronePos.z);
    }
  }

  function resetCamera() {
    if (!camera || !controls || !droneGroup) return;
    cameraMode = 'orbit';
    controls.enabled = true;
    const dPos = droneGroup.position;
    camera.position.set(dPos.x - 6, dPos.y + 5, dPos.z + 8);
    controls.target.set(dPos.x, dPos.y, dPos.z);
    controls.update();
    addToast('3D Camera Reset to Drone', 'info', 1500);
  }

  function clearTrail() {
    trailPositions = [];
    lastTrailX = null;
    lastTrailY = null;
    lastTrailZ = null;
    if (trailLine) {
      trailLine.geometry.setAttribute(
        'position',
        new THREE.Float32BufferAttribute([], 3)
      );
      trailLine.geometry.attributes.position.needsUpdate = true;
    }

    // Reset Gate Estimation Trails & Prior Markers
    for (let i = 0; i < 5; i++) {
      gateTrailPositions[i] = [];
      lastGatePos[i] = null;
      if (gateTrailLines[i]) {
        gateTrailLines[i].geometry.setAttribute(
          'position',
          new THREE.Float32BufferAttribute([], 3)
        );
        gateTrailLines[i].geometry.attributes.position.needsUpdate = true;
      }
      if (priorOriginMarkers[i]) {
        priorOriginMarkers[i].visible = false;
      }
    }
    addToast('Flight & Gate Estimation Trails Cleared', 'info', 1500);
  }

  async function handleResetGates() {
    try {
      // 1. Re-anchor 3D visualizer to current drone position & heading
      const { x, y, z, yaw } = $dronePose;
      if (x !== undefined && !isNaN(x)) {
        initialDronePos = { x: x, y: y, z: z, yaw: (yaw !== undefined ? yaw : 0) };
        if (gridHelper) {
          gridHelper.rotation.y = (initialDronePos.yaw !== undefined ? initialDronePos.yaw * (Math.PI / 180) : 0);
        }
      }

      // 2. Call estimator reset service (re-anchors gate estimator in ROS)
      await callResetGates();

      // 3. Clear all flight and gate trails
      clearTrail();

      // 4. Update gate positions
      updateGatesFromPriorsOrPoses();

      addToast('Gate estimation reset & anchored to drone position', 'success', 2000);
    } catch (e) {
      console.error('Failed to reset gate estimation:', e);
      addToast('Failed to reset gate estimation', 'error', 2000);
    }
  }

  function handleResize() {
    if (!containerEl || !camera || !renderer) return;
    const width = containerEl.clientWidth;
    const height = containerEl.clientHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }

  onMount(() => {
    initThree();
    window.addEventListener('resize', handleResize);
  });

  onDestroy(() => {
    if (animFrameId) cancelAnimationFrame(animFrameId);
    window.removeEventListener('resize', handleResize);
    if (renderer) renderer.dispose();
    if (controls) controls.dispose();
  });
</script>

<div class="viewer-3d-wrapper" bind:this={containerEl}>
  <canvas bind:this={canvasEl} class="webgl-canvas"></canvas>

  <!-- 3D Viewport Controls & HUD Overlay -->
  <div class="hud-overlay">
    <!-- Top Controls Row -->
    <div class="top-row">
      <!-- Top Left Mode Buttons -->
      <div class="btn-group font-hud">
        <button
          class="cam-btn {cameraMode === 'orbit' ? 'active' : ''}"
          on:click={() => setCameraMode('orbit')}
          title="Free 360 Orbit Camera (Drag with mouse)"
        >
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <circle cx="12" cy="12" r="10"/>
            <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"/>
          </svg>
          <span>ORBIT</span>
        </button>

        <button
          class="cam-btn {cameraMode === 'chase' ? 'active' : ''}"
          on:click={() => setCameraMode('chase')}
          title="Follow Behind Drone in 3D"
        >
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <polygon points="12 2 2 7 12 12 22 7 12 2" />
            <polyline points="2 17 12 22 22 17" />
            <polyline points="2 12 12 17 22 12" />
          </svg>
          <span>CHASE</span>
        </button>

        <button
          class="cam-btn {cameraMode === 'topdown' ? 'active' : ''}"
          on:click={() => setCameraMode('topdown')}
          title="Top-Down Map View"
        >
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <rect x="3" y="3" width="18" height="18" rx="2"/>
            <circle cx="12" cy="12" r="3"/>
          </svg>
          <span>TOP-DOWN</span>
        </button>

        <button class="cam-btn" on:click={resetCamera} title="Center Camera on Drone">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <circle cx="12" cy="12" r="3"/>
            <path d="M3 12h3M18 12h3M12 3v3M12 18v3"/>
          </svg>
          <span>SNAP</span>
        </button>
      </div>

      <!-- Top Center Active Target Gate Switcher -->
      <div class="gate-picker-hud font-hud">
        <span class="gate-picker-title">TARGET:</span>
        <button
          class="gate-nav-btn"
          on:click={() => setTargetGate(Math.max(0, $targetGateIndex - 1))}
          title="Previous Target Gate"
        >&lsaquo;</button>

        {#each [0, 1, 2, 3, 4] as gIdx}
          <button
            class="gate-select-pill { $targetGateIndex === gIdx ? 'active' : '' }"
            on:click={() => setTargetGate(gIdx)}
            title="Switch Active Target to Gate #{gIdx + 1}"
          >
            G{gIdx + 1}
          </button>
        {/each}

        <button
          class="gate-nav-btn"
          on:click={() => setTargetGate(Math.min(4, $targetGateIndex + 1))}
          title="Next Target Gate"
        >&rsaquo;</button>
      </div>

      <!-- Top Right Quick Actions -->
      <div class="top-right-tools">
        <!-- Re-Anchor Gate Estimator Service Button -->
        <button
          class="reanchor-btn font-hud"
          on:click={handleResetGates}
          title="Re-anchors all 5 gate priors relative to current drone position and resets all trails"
        >
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"/>
            <path d="M21 3v5h-5"/>
            <path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"/>
            <path d="M3 21v-5h5"/>
          </svg>
          <span>RESET GATE ESTIMATION</span>
        </button>

        <button class="tool-icon-btn" on:click={clearTrail} title="Clear Flight & Gate Trails">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
          </svg>
        </button>
      </div>
    </div>

    <!-- Bottom Telemetry Ribbon -->
    <div class="bottom-stats font-mono">
      <div class="stat-pill">
        <span class="stat-lbl">DRONE:</span>
        <span class="stat-val">
          X:{$dronePose.x.toFixed(2)}
          Y:{$dronePose.y.toFixed(2)}
          Z:{($dronePose.rel_alt !== undefined ? $dronePose.rel_alt : $dronePose.z).toFixed(2)}m
        </span>
      </div>

      <div class="stat-pill target-pill">
        <span class="stat-lbl">TARGET:</span>
        <span class="stat-val emerald">{$targetGateLabel}</span>
      </div>

      <div class="stat-pill help-pill">
        <span class="stat-lbl">MOUSE:</span>
        <span class="stat-val">Orbit: Left-Drag &bull; Pan: Right-Drag &bull; Zoom: Scroll</span>
      </div>
    </div>
  </div>
</div>

<style>
  .viewer-3d-wrapper {
    position: relative;
    width: 100%;
    height: 100%;
    min-height: 380px;
    background: #070b12;
    overflow: hidden;
    display: flex;
  }

  .webgl-canvas {
    width: 100% !important;
    height: 100% !important;
    display: block;
    cursor: grab;
  }

  .webgl-canvas:active {
    cursor: grabbing;
  }

  .hud-overlay {
    position: absolute;
    inset: 0;
    pointer-events: none;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    padding: 12px 14px;
  }

  .top-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    width: 100%;
    flex-wrap: wrap;
  }

  .btn-group {
    pointer-events: auto;
    display: flex;
    align-items: center;
    gap: 6px;
    background: rgba(7, 11, 18, 0.75);
    backdrop-filter: blur(8px);
    padding: 4px;
    border-radius: 6px;
    border: 1px solid rgba(0, 240, 255, 0.2);
    box-shadow: 0 4px 14px rgba(0, 0, 0, 0.5);
    width: fit-content;
  }

  .cam-btn {
    display: inline-flex;
    align-items: center;
    gap: 5px;
    background: transparent;
    border: none;
    color: var(--text-secondary, #94a3b8);
    font-size: 0.68rem;
    font-weight: 700;
    padding: 4px 8px;
    border-radius: 4px;
    cursor: pointer;
    transition: all 0.2s ease;
  }

  .cam-btn:hover {
    color: var(--accent-cyan, #00f0ff);
    background: rgba(0, 240, 255, 0.1);
  }

  .cam-btn.active {
    color: #070b12;
    background: var(--accent-cyan, #00f0ff);
    font-weight: 800;
    box-shadow: 0 0 10px rgba(0, 240, 255, 0.4);
  }

  .gate-picker-hud {
    pointer-events: auto;
    display: flex;
    align-items: center;
    gap: 4px;
    background: rgba(7, 11, 18, 0.85);
    backdrop-filter: blur(10px);
    padding: 4px 8px;
    border-radius: 6px;
    border: 1px solid rgba(16, 185, 129, 0.35);
    box-shadow: 0 4px 16px rgba(0, 0, 0, 0.6);
  }

  .gate-picker-title {
    color: #64748b;
    font-size: 0.65rem;
    font-weight: 800;
    letter-spacing: 0.5px;
    margin-right: 2px;
  }

  .gate-nav-btn {
    background: rgba(255, 255, 255, 0.05);
    border: 1px solid rgba(255, 255, 255, 0.1);
    color: #94a3b8;
    font-size: 0.95rem;
    line-height: 1;
    padding: 2px 6px;
    border-radius: 4px;
    cursor: pointer;
    transition: all 0.15s ease;
  }

  .gate-nav-btn:hover {
    color: #34d399;
    background: rgba(16, 185, 129, 0.15);
    border-color: #34d399;
  }

  .gate-select-pill {
    background: rgba(255, 255, 255, 0.04);
    border: 1px solid rgba(255, 255, 255, 0.1);
    color: #94a3b8;
    font-size: 0.66rem;
    font-weight: 700;
    padding: 4px 7px;
    border-radius: 4px;
    cursor: pointer;
    transition: all 0.15s ease;
  }

  .gate-select-pill:hover {
    color: #34d399;
    background: rgba(16, 185, 129, 0.15);
    border-color: rgba(16, 185, 129, 0.4);
  }

  .gate-select-pill.active {
    color: #070b12;
    background: #10b981;
    font-weight: 800;
    border-color: #34d399;
    box-shadow: 0 0 10px rgba(16, 185, 129, 0.5);
  }

  .top-right-tools {
    pointer-events: auto;
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .reanchor-btn {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    background: rgba(16, 185, 129, 0.18);
    color: #34d399;
    border: 1px solid rgba(16, 185, 129, 0.45);
    padding: 5px 12px;
    border-radius: 6px;
    font-size: 0.72rem;
    font-weight: 800;
    letter-spacing: 0.5px;
    cursor: pointer;
    box-shadow: 0 0 12px rgba(16, 185, 129, 0.25);
    backdrop-filter: blur(8px);
    transition: all 0.2s ease;
  }

  .reanchor-btn:hover {
    background: rgba(16, 185, 129, 0.35);
    color: #ffffff;
    border-color: #34d399;
    box-shadow: 0 0 18px rgba(16, 185, 129, 0.5);
    transform: translateY(-1px);
  }

  .reanchor-btn:active {
    transform: scale(0.97);
  }

  .tool-icon-btn {
    background: rgba(7, 11, 18, 0.75);
    border: 1px solid rgba(255, 255, 255, 0.15);
    color: #94a3b8;
    padding: 6px 8px;
    border-radius: 6px;
    cursor: pointer;
    backdrop-filter: blur(8px);
    transition: all 0.2s ease;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .tool-icon-btn:hover {
    color: #ef4444;
    border-color: #ef4444;
    background: rgba(239, 68, 68, 0.1);
  }

  .bottom-stats {
    display: flex;
    align-items: center;
    gap: 10px;
    flex-wrap: wrap;
  }

  .stat-pill {
    background: rgba(7, 11, 18, 0.8);
    backdrop-filter: blur(8px);
    padding: 4px 10px;
    border-radius: 5px;
    border: 1px solid rgba(0, 240, 255, 0.2);
    font-size: 0.68rem;
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }

  .stat-lbl {
    color: #64748b;
    font-weight: 700;
  }

  .stat-val {
    color: #00f0ff;
    font-weight: 600;
  }

  .stat-val.emerald {
    color: #34d399;
    font-weight: 800;
  }

  .target-pill {
    border-color: rgba(16, 185, 129, 0.4);
    box-shadow: 0 0 10px rgba(16, 185, 129, 0.15);
  }

  .help-pill {
    margin-left: auto;
    border-color: rgba(255, 255, 255, 0.1);
    color: #94a3b8;
  }

  @media (max-width: 768px) {
    .help-pill {
      display: none;
    }
  }
</style>
