<script>
  import { onMount, onDestroy } from 'svelte';
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import {
    dronePose,
    refinedGatePoses,
    targetGateIndex,
    targetGateLabel,
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
  let rotorMeshes = [];
  let gateGroups = [];
  let trailLine;
  let trailPositions = [];
  const MAX_TRAIL_POINTS = 250;

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
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(1.5, 0.38, 1.0);
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

    // Front / Nose Direction Arrow (Points in -Z in Three.js standard)
    const noseGeo = new THREE.ConeGeometry(0.06, 0.15, 8);
    const noseMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const nose = new THREE.Mesh(noseGeo, noseMat);
    nose.rotation.x = -Math.PI / 2;
    nose.position.z = -0.22;
    nose.position.y = 0.02;
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

    // 4 Motors & Spinning Propeller Disks
    const motorOffsets = [
      { x: 0.16, z: -0.16 }, // Front-Right
      { x: -0.16, z: -0.16 }, // Front-Left
      { x: 0.16, z: 0.16 },  // Rear-Right
      { x: -0.16, z: 0.16 },  // Rear-Left
    ];

    rotorMeshes = [];

    motorOffsets.forEach((pos, idx) => {
      // Motor mount
      const motorGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.04, 12);
      const motorMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8 });
      const motor = new THREE.Mesh(motorGeo, motorMat);
      motor.position.set(pos.x, 0.03, pos.z);
      group.add(motor);

      // Propeller Blades (2-blade)
      const propGeo = new THREE.BoxGeometry(0.24, 0.005, 0.025);
      const propMat = new THREE.MeshBasicMaterial({
        color: idx < 2 ? 0x00f0ff : 0xf59e0b,
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

  function buildGateMesh(id) {
    const group = new THREE.Group();
    group.name = `gate_${id}`;

    const gateWidth = 2.0;
    const gateHeight = 2.0;
    const beamThick = 0.1;

    const frameMat = new THREE.MeshStandardMaterial({
      color: 0x00f0ff,
      metalness: 0.6,
      roughness: 0.3,
      emissive: 0x002233
    });

    // Top Beam
    const topGeo = new THREE.BoxGeometry(gateWidth, beamThick, beamThick);
    const topBeam = new THREE.Mesh(topGeo, frameMat);
    topBeam.position.y = gateHeight / 2;
    group.add(topBeam);

    // Bottom Beam
    const botBeam = new THREE.Mesh(topGeo, frameMat);
    botBeam.position.y = -gateHeight / 2;
    group.add(botBeam);

    // Left Column
    const colGeo = new THREE.BoxGeometry(beamThick, gateHeight, beamThick);
    const leftCol = new THREE.Mesh(colGeo, frameMat);
    leftCol.position.x = -gateWidth / 2;
    group.add(leftCol);

    // Right Column
    const rightCol = new THREE.Mesh(colGeo, frameMat);
    rightCol.position.x = gateWidth / 2;
    group.add(rightCol);

    // Outer Target Pulse Ring (Only visible on active target)
    const ringGeo = new THREE.RingGeometry(1.3, 1.4, 32);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.4
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.name = 'target_ring';
    ring.visible = false;
    group.add(ring);

    // Sprite Label above gate
    const label = createTextSprite(`GATE #${id}`, '#00f0ff');
    label.name = 'gate_label';
    label.position.y = gateHeight / 2 + 0.45;
    group.add(label);

    return { group, frameMat, ring, label, id };
  }

  function initThree() {
    if (!canvasEl || !containerEl) return;

    const width = containerEl.clientWidth || 640;
    const height = containerEl.clientHeight || 420;

    // 1. Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070b12);
    scene.fog = new THREE.FogExp2(0x070b12, 0.018);

    // 2. Camera
    camera = new THREE.PerspectiveCamera(55, width / height, 0.1, 300);
    camera.position.set(0, 8, 14);

    // 3. Renderer
    renderer = new THREE.WebGLRenderer({
      canvas: canvasEl,
      antialias: true,
      alpha: true,
      powerPreference: 'high-performance'
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;

    // 4. OrbitControls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;
    controls.maxDistance = 120;
    controls.minDistance = 0.5;
    controls.maxPolarAngle = Math.PI / 2 + 0.05; // Don't go deep below ground
    controls.target.set(0, 1.0, 0);

    // 5. Lighting
    const ambientLight = new THREE.AmbientLight(0x20354b, 1.8);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 2.2);
    dirLight.position.set(20, 40, 20);
    dirLight.castShadow = true;
    scene.add(dirLight);

    const pointLight = new THREE.PointLight(0x00f0ff, 1.5, 30);
    pointLight.position.set(0, 5, 0);
    scene.add(pointLight);

    // 6. Ground Grid & Coordinate Helpers
    const gridHelper = new THREE.GridHelper(80, 80, 0x00f0ff, 0x142333);
    gridHelper.position.y = 0;
    scene.add(gridHelper);

    // Circular Launch Pad marker
    const padGeo = new THREE.RingGeometry(0.1, 1.5, 32);
    const padMat = new THREE.MeshBasicMaterial({
      color: 0x00f0ff,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.35
    });
    const launchPad = new THREE.Mesh(padGeo, padMat);
    launchPad.rotation.x = Math.PI / 2;
    launchPad.position.y = 0.01;
    scene.add(launchPad);

    // 7. Quadcopter Mesh
    droneGroup = buildDroneMesh();
    droneGroup.position.set(0, 1.0, 0);
    scene.add(droneGroup);

    // 8. Gate Meshes (5 gates)
    gateGroups = [];
    for (let i = 1; i <= 5; i++) {
      const gate = buildGateMesh(i);
      scene.add(gate.group);
      gateGroups.push(gate);
    }
    updateGatesFromPriorsOrPoses();

    // 9. Flight Trajectory Ribbon
    const trailGeo = new THREE.BufferGeometry();
    const trailMat = new THREE.LineBasicMaterial({
      color: 0x00f0ff,
      transparent: true,
      opacity: 0.7,
      linewidth: 2
    });
    trailLine = new THREE.Line(trailGeo, trailMat);
    scene.add(trailLine);

    // Start Animation Loop
    animate();
  }

  function updateGatesFromPriorsOrPoses() {
    if (!gateGroups || gateGroups.length === 0) return;

    let poses = $refinedGatePoses;
    const currentTarget = $targetGateIndex;

    gateGroups.forEach((gate, idx) => {
      let posX = 0, posY = 1.0, posZ = 0;
      let qx = 0, qy = 0, qz = 0, qw = 1;

      if (poses && poses.length > idx) {
        const p = poses[idx];
        const tPos = enuToThree(p.x, p.y, p.z);
        posX = tPos.x;
        posY = tPos.y;
        posZ = tPos.z;
        qx = p.qx;
        qy = p.qz;
        qz = -p.qy;
        qw = p.qw;
      } else {
        // Fallback default prior
        const p = defaultPriorsRDF[idx];
        const enuX = p.z;
        const enuY = -p.x;
        const enuZ = -p.y;
        const tPos = enuToThree(enuX, enuY, enuZ);
        posX = tPos.x;
        posY = tPos.y;
        posZ = tPos.z;
      }

      gate.group.position.set(posX, posY, posZ);
      if (poses && poses.length > idx) {
        gate.group.quaternion.set(qx, qy, qz, qw);
      }

      // Dynamic Color Highlighting based on Target Gate
      if (idx === currentTarget) {
        // Active Target Gate -> Vibrant Emerald Green
        gate.frameMat.color.setHex(0x10b981);
        gate.frameMat.emissive.setHex(0x064e3b);
        gate.ring.visible = true;
      } else if (idx === currentTarget + 1) {
        // Next Preview Gate -> Amber Gold
        gate.frameMat.color.setHex(0xf59e0b);
        gate.frameMat.emissive.setHex(0x78350f);
        gate.ring.visible = false;
      } else if (idx < currentTarget) {
        // Passed Gate -> Dim Silver/Cyan
        gate.frameMat.color.setHex(0x475569);
        gate.frameMat.emissive.setHex(0x0f172a);
        gate.ring.visible = false;
      } else {
        // Upcoming Gate -> Cyber Cyan
        gate.frameMat.color.setHex(0x00f0ff);
        gate.frameMat.emissive.setHex(0x002233);
        gate.ring.visible = false;
      }
    });
  }

  function updateDrone() {
    if (!droneGroup) return;

    const { x, y, z, qx, qy, qz, qw } = $dronePose;
    const tPos = enuToThree(x, y, z);

    droneGroup.position.set(tPos.x, tPos.y, tPos.z);

    // Convert ROS ENU quaternion to Three.js orientation:
    // Three.js: (qx, qz, -qy, qw)
    if (qx !== undefined && qw !== undefined) {
      droneGroup.quaternion.set(qx, qz, -qy, qw);
    }

    // Spin Propellers
    const spinSpeed = 0.45;
    rotorMeshes.forEach((rotor, idx) => {
      rotor.rotation.y += (idx % 2 === 0 ? 1 : -1) * spinSpeed;
    });

    // Update Flight Path Trail
    trailPositions.push(tPos.x, tPos.y, tPos.z);
    if (trailPositions.length > MAX_TRAIL_POINTS * 3) {
      trailPositions.splice(0, 3);
    }

    if (trailLine && trailPositions.length >= 6) {
      trailLine.geometry.setAttribute(
        'position',
        new THREE.Float32BufferAttribute(trailPositions, 3)
      );
      trailLine.geometry.attributes.position.needsUpdate = true;
    }

    // Camera follow modes
    if (cameraMode === 'chase' && camera && controls) {
      const dronePos = droneGroup.position;
      // Get backwards vector from drone
      const backward = new THREE.Vector3(0, 0.8, 2.5);
      backward.applyQuaternion(droneGroup.quaternion);
      camera.position.lerp(dronePos.clone().add(backward), 0.1);
      controls.target.lerp(dronePos, 0.1);
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
    } else if (mode === 'topdown') {
      controls.enabled = false;
    }
  }

  function resetCamera() {
    if (!camera || !controls || !droneGroup) return;
    cameraMode = 'orbit';
    controls.enabled = true;
    const dPos = droneGroup.position;
    camera.position.set(dPos.x, dPos.y + 6, dPos.z + 10);
    controls.target.set(dPos.x, dPos.y, dPos.z);
    controls.update();
    addToast('3D Camera Reset to Drone', 'info', 1500);
  }

  function clearTrail() {
    trailPositions = [];
    if (trailLine) {
      trailLine.geometry.setAttribute(
        'position',
        new THREE.Float32BufferAttribute([], 3)
      );
    }
    addToast('Flight trajectory trail cleared', 'info', 1500);
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

    <!-- Top Right Quick Actions -->
    <div class="top-right-tools">
      <!-- Re-Anchor Gate Estimator Service Button -->
      <button
        class="reanchor-btn font-hud"
        on:click={callResetGates}
        title="Re-anchors all 5 gate priors relative to current drone position"
      >
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"/>
          <path d="M21 3v5h-5"/>
          <path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"/>
          <path d="M3 21v-5h5"/>
        </svg>
        <span>RESET GATE ESTIMATION</span>
      </button>

      <button class="tool-icon-btn" on:click={clearTrail} title="Clear Flight Path Trail">
        <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
        </svg>
      </button>
    </div>

    <!-- Bottom Telemetry Ribbon -->
    <div class="bottom-stats font-mono">
      <div class="stat-pill">
        <span class="stat-lbl">DRONE:</span>
        <span class="stat-val">X:{$dronePose.x.toFixed(1)} Y:{$dronePose.y.toFixed(1)} Z:{$dronePose.z.toFixed(1)}m</span>
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

  .top-right-tools {
    pointer-events: auto;
    position: absolute;
    top: 12px;
    right: 14px;
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
