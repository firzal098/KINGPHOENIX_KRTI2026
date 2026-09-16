import { writable, derived } from 'svelte/store';
import ROSLIB from 'roslib';

export function getDefaultRosbridgeUrl() {
  if (typeof window !== 'undefined' && window.location) {
    const loc = window.location;
    // When served via Vite dev server (port 5173), route through the built-in WebSocket proxy
    if (loc.port === '5173') {
      const proto = loc.protocol === 'https:' ? 'wss:' : 'ws:';
      return `${proto}//${loc.host}/rosbridge`;
    }
    const host = loc.hostname || 'localhost';
    return `ws://${host}:9090`;
  }
  return 'ws://localhost:9090';
}

export const connectionStatus = writable('disconnected'); // 'connected' | 'connecting' | 'disconnected' | 'error'
export const connectionError = writable('');
export const rosbridgeUrl = writable(getDefaultRosbridgeUrl());

// Active Controller FSM State ('OFF' | 'HOVER' | 'RUN' | 'CLIMBING' | 'LANDING' | 'ARMING' | etc.)
export const controllerFsmState = writable('OFF');

// Active Target Gate Index (0 = Gate #1, 1 = Gate #2, etc.) & Sub-Gate Index (0, 1, 2, 3)
export const targetGateIndex = writable(0);
export const targetSubGateIndex = writable(0);
export const targetGateLabel = derived([targetGateIndex, targetSubGateIndex], ([$idx, $sub]) => {
  if ($idx >= 5) return 'ALL GATES CLEARED (5/5)';
  if ($idx === 0 && $sub === 1) {
    return 'GATE #1.1 (VIRTUAL WP)';
  }
  if ($idx === 1 && $sub === 1) {
    return 'GATE #2.1 (VIRTUAL WP)';
  }
  if ($idx === 2) {
    return `GATE #3 [${$sub + 1}/3]${$sub > 0 ? ` (SUB ${$sub})` : ''}`;
  }
  if ($idx === 3) {
    return `GATE #4 [${$sub + 1}/4]${$sub > 0 ? ` (SUB ${$sub})` : ''}`;
  }
  return `GATE #${$idx + 1} (${$idx + 1}/5)`;
});

export const previewGateIndex = writable(1);
export const previewSubGateIndex = writable(0);

export const previewGateLabel = derived([previewGateIndex, previewSubGateIndex], ([$pIdx, $pSub]) => {
  if ($pIdx === 0 && $pSub === 1) {
    return 'GATE #1.1 (VIRTUAL WP)';
  }
  if ($pIdx === 1 && $pSub === 1) {
    return 'GATE #2.1 (VIRTUAL WP)';
  }
  if ($pIdx === 3 && $pSub === 4) {
    return 'VIRTUAL GATE 4 (+6.0m)';
  }
  if ($pIdx === 4 && $pSub === 1) {
    return 'VIRTUAL EXIT (+3.0m)';
  }
  if ($pSub > 0) {
    return `GATE #${$pIdx + 1}.${$pSub} (SUB ${$pSub})`;
  }
  if ($pIdx >= 0 && $pIdx < 5) {
    return `GATE #${$pIdx + 1}`;
  }
  return 'ALL CLEARED';
});

export const fcuState = writable({
  connected: false,
  armed: false,
  guided: false,
  mode: 'DISARMED',
});

export const dronePose = writable({
  x: 0.0,
  y: 0.0,
  z: 0.0,
  qx: 0.0,
  qy: 0.0,
  qz: 0.0,
  qw: 1.0,
  roll: 0.0,
  pitch: 0.0,
  yaw: 0.0,
  timestamp: 0,
});

// Refined 3D Gate Poses from Estimator (Array of { id, x, y, z, qx, qy, qz, qw })
export const refinedGatePoses = writable([]);

// Gripper State (true = opened, false = closed)
export const gripperOpened = writable(false);
export const gripperPending = writable(false);

// Real-Time Topic Frequencies (Hz / FPS)
export const policyActionHz = writable(0);
export const gateEstimatorHz = writable(0);
export const cameraFps = writable(0);

// Action Space Maximum Constraints
export const maxActionMagnitude = writable(16.0); // Range: 1.0 to 16.0 m/s
export const maxYawRateDeg = writable(360.0);      // Range: 20 to 360 deg/s

// Gate 3.2 Auto-Ungrip Delay (seconds)
export const gate3UngripDelay = writable(0.0);

// Landing Pad Telemetry (/estimator/landing_pad)
export const landingPadPose = writable({ x: 0, y: 0, z: 0, valid: false });
export const manualLandingPadActive = writable(false);

export const droneVel = writable({
  vx: 0.0,
  vy: 0.0,
  vz: 0.0,
  speed: 0.0,
});

export const debugImage = writable({
  width: 640,
  height: 480,
  encoding: 'bgr8',
  data: null, // raw buffer
  fps: 0,
  timestamp: 0,
});

// Compressed JPEG data URI stream for instant, lag-free browser rendering
export const debugImageSrc = writable('');

// Toast notification store
export const toasts = writable([]);

export function addToast(message, type = 'info', duration = 3500) {
  const id = Date.now() + Math.random();
  toasts.update((items) => [...items, { id, message, type }]);
  if (duration > 0) {
    setTimeout(() => {
      removeToast(id);
    }, duration);
  }
  return id;
}

export function removeToast(id) {
  toasts.update((items) => items.filter((t) => t.id !== id));
}

// Raw 42D Observation Array
export const rawObservation = writable(new Array(42).fill(0.0));

// Structured Observation Derived Store
export const structuredObservation = derived(rawObservation, ($obs) => {
  if (!$obs || $obs.length < 42) {
    return null;
  }

  // 1. Body Velocity (FLU) [0:3]
  const vel_B = {
    vx: $obs[0] || 0.0,
    vy: $obs[1] || 0.0,
    vz: $obs[2] || 0.0,
    speed: Math.sqrt(($obs[0] || 0) ** 2 + ($obs[1] || 0) ** 2 + ($obs[2] || 0) ** 2),
  };

  // 2. Projected Gravity [3:6]
  const grav_B = {
    gx: $obs[3] || 0.0,
    gy: $obs[4] || 0.0,
    gz: $obs[5] || 0.0,
  };

  // 3. Body Angular Velocity (omega_B in Body FLU) [6:9]
  const omega_B = {
    wx: $obs[6] || 0.0,
    wy: $obs[7] || 0.0,
    wz: $obs[8] || 0.0,
    wxDeg: ($obs[6] || 0.0) * (180.0 / Math.PI),
    wyDeg: ($obs[7] || 0.0) * (180.0 / Math.PI),
    wzDeg: ($obs[8] || 0.0) * (180.0 / Math.PI),
  };

  // 4. Active Gate 15D [9:24]
  const active_gate = {
    tl: [$obs[9] || 0.0, $obs[10] || 0.0, $obs[11] || 0.0],
    tr: [$obs[12] || 0.0, $obs[13] || 0.0, $obs[14] || 0.0],
    bl: [$obs[15] || 0.0, $obs[16] || 0.0, $obs[17] || 0.0],
    br: [$obs[18] || 0.0, $obs[19] || 0.0, $obs[20] || 0.0],
    center: [$obs[21] || 0.0, $obs[22] || 0.0, $obs[23] || 0.0],
    dist: Math.hypot($obs[21] || 0, $obs[22] || 0, $obs[23] || 0),
  };

  // 5. Next Gate 15D [24:39]
  const isZero = $obs.slice(24, 39).every((v) => Math.abs(v) < 1e-5);
  const next_gate = {
    has_next: !isZero,
    tl: [$obs[24] || 0.0, $obs[25] || 0.0, $obs[26] || 0.0],
    tr: [$obs[27] || 0.0, $obs[28] || 0.0, $obs[29] || 0.0],
    bl: [$obs[30] || 0.0, $obs[31] || 0.0, $obs[32] || 0.0],
    br: [$obs[33] || 0.0, $obs[34] || 0.0, $obs[35] || 0.0],
    center: [$obs[36] || 0.0, $obs[37] || 0.0, $obs[38] || 0.0],
    dist: Math.hypot($obs[36] || 0, $obs[37] || 0, $obs[38] || 0),
  };

  // 6. Previous Action (3D) [39:42]
  const prev_action = {
    vfwd: $obs[39] || 0.0,
    vleft: $obs[40] || 0.0,
    yawRate: $obs[41] || 0.0,
    yawRateDeg: ($obs[41] || 0.0) * (180.0 / Math.PI),
  };

  return { vel_B, grav_B, omega_B, active_gate, next_gate, prev_action };
});

// Raw 3D Action Array [v_fwd, v_left, yaw_rate]
export const rawAction = writable([0.0, 0.0, 0.0]);

// Structured Action Derived Store
export const structuredAction = derived(rawAction, ($act) => {
  const vfwd = $act[0] || 0.0;
  const vleft = $act[1] || 0.0;
  const yawRate = $act[2] || 0.0;
  return {
    vfwd,
    vleft,
    yawRate,
    yawRateDeg: yawRate * (180.0 / Math.PI),
    totalSpeed: Math.hypot(vfwd, vleft),
  };
});

// Reset Action and Observation space telemetry to zero
export function resetPolicyTelemetry() {
  rawObservation.set(new Array(42).fill(0.0));
  rawAction.set([0.0, 0.0, 0.0]);
}

export const serviceResponseLog = writable([]);

// ROS instance and topics
let ros = null;
let reconnectTimer = null;
let imageFrameCount = 0;
let lastFpsTime = performance.now();
let actionFrameCount = 0;
let lastActionFpsTime = performance.now();
let estimatorFrameCount = 0;
let lastEstimatorFpsTime = performance.now();

// Reset rate counters to 0 if messages stop arriving
if (typeof window !== 'undefined') {
  setInterval(() => {
    const now = performance.now();
    if (now - lastActionFpsTime > 2000) policyActionHz.set(0);
    if (now - lastEstimatorFpsTime > 2000) gateEstimatorHz.set(0);
    if (now - lastFpsTime > 2000) {
      cameraFps.set(0);
      debugImage.update((prev) => ({ ...prev, fps: 0 }));
    }
  }, 1000);
}

function quaternionToEuler(x, y, z, w) {
  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (w * x + y * z);
  const cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI);

  // Pitch (y-axis rotation)
  const sinp = 2 * (w * y - z * x);
  let pitch = 0;
  if (Math.abs(sinp) >= 1) {
    pitch = Math.sign(sinp) * (Math.PI / 2) * (180 / Math.PI);
  } else {
    pitch = Math.asin(sinp) * (180 / Math.PI);
  }

  // Yaw (z-axis rotation)
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI);

  return { roll, pitch, yaw };
}

export function initRosConnection(url) {
  const targetUrl = url || getDefaultRosbridgeUrl();
  rosbridgeUrl.set(targetUrl);

  if (ros) {
    try {
      ros.close();
    } catch (e) {}
  }

  connectionStatus.set('connecting');
  connectionError.set('');

  ros = new ROSLIB.Ros({ url: targetUrl });

  ros.on('connection', () => {
    connectionStatus.set('connected');
    connectionError.set('');
    addToast(`Connected to ROSBridge (${targetUrl})`, 'success', 2500);
    subscribeTopics();
  });

  ros.on('error', (error) => {
    connectionStatus.set('error');
    cameraFps.set(0);
    debugImage.update((prev) => ({ ...prev, fps: 0 }));
    if (perceptionSub) {
      try {
        perceptionSub.unsubscribe();
      } catch (e) {}
      perceptionSub = null;
    }
    connectionError.set(error?.message || `Failed to connect to ${targetUrl}`);
  });

  ros.on('close', () => {
    connectionStatus.set('disconnected');
    cameraFps.set(0);
    debugImage.update((prev) => ({ ...prev, fps: 0 }));
    if (perceptionSub) {
      try {
        perceptionSub.unsubscribe();
      } catch (e) {}
      perceptionSub = null;
    }
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        initRosConnection(targetUrl);
      }, 2500);
    }
  });
}

function subscribeTopics() {
  if (!ros) return;

  // 1. Controller FSM State (/controller/current_state)
  const fsmSub = new ROSLIB.Topic({
    ros,
    name: '/controller/current_state',
    messageType: 'std_msgs/msg/String',
  });
  fsmSub.subscribe((msg) => {
    if (msg?.data) {
      const stateUpper = msg.data.toUpperCase();
      controllerFsmState.set(stateUpper);
      if (stateUpper === 'OFF' || stateUpper === 'LANDING') {
        rawAction.set([0.0, 0.0, 0.0]);
      }
    }
  });

  // 1b. Active Target Gate Index & Sub-Gate Index
  let prevTargetGate = null;
  let prevTargetSubGate = null;

  function handleGateAdvance(newIndex, newSubIndex = null) {
    if (newSubIndex !== null && typeof newSubIndex === 'number') {
      if (prevTargetSubGate !== null && prevTargetGate === newIndex && newSubIndex > prevTargetSubGate) {
        addToast(`🎯 GATE #${newIndex + 1} SUB-GATE #${prevTargetSubGate + 1} PASSED! Target: SUB-GATE #${newSubIndex + 1}`, 'success', 3000);
      }
      prevTargetSubGate = newSubIndex;
      targetSubGateIndex.set(newSubIndex);
    }

    if (newIndex !== null && typeof newIndex === 'number') {
      if (prevTargetGate !== null && newIndex > prevTargetGate) {
        const passedGateNum = prevTargetGate + 1;
        if (newIndex >= 5) {
          addToast(`🏁 ALL GATES CLEARED! Course Completed (Gate #${passedGateNum}/5 Passed)!`, 'success', 5000);
        } else {
          addToast(`🎯 GATE #${passedGateNum} CLEARED! Next Target: GATE #${newIndex + 1}`, 'success', 3500);
        }
        prevTargetSubGate = 0;
        targetSubGateIndex.set(0);
      }
      prevTargetGate = newIndex;
      targetGateIndex.set(newIndex);
    }
  }

  const targetGateSub = new ROSLIB.Topic({
    ros,
    name: '/controller/target_gate_index',
    messageType: 'std_msgs/msg/Int32',
  });
  targetGateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      handleGateAdvance(msg.data, null);
    }
  });

  const policyTargetGateSub = new ROSLIB.Topic({
    ros,
    name: '/policy/target_gate',
    messageType: 'std_msgs/msg/Int32',
  });
  policyTargetGateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      handleGateAdvance(msg.data, null);
    }
  });

  const targetSubgateSub = new ROSLIB.Topic({
    ros,
    name: '/controller/target_subgate_index',
    messageType: 'std_msgs/msg/Int32',
  });
  targetSubgateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      let currMain = 0;
      targetGateIndex.subscribe((v) => (currMain = v))();
      handleGateAdvance(currMain, msg.data);
    }
  });

  const policyTargetSubgateSub = new ROSLIB.Topic({
    ros,
    name: '/policy/target_subgate',
    messageType: 'std_msgs/msg/Int32',
  });
  policyTargetSubgateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      let currMain = 0;
      targetGateIndex.subscribe((v) => (currMain = v))();
      handleGateAdvance(currMain, msg.data);
    }
  });

  const previewGateSub = new ROSLIB.Topic({
    ros,
    name: '/controller/preview_gate_index',
    messageType: 'std_msgs/msg/Int32',
  });
  previewGateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      previewGateIndex.set(msg.data);
    }
  });

  const previewSubgateSub = new ROSLIB.Topic({
    ros,
    name: '/controller/preview_subgate_index',
    messageType: 'std_msgs/msg/Int32',
  });
  previewSubgateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      previewSubGateIndex.set(msg.data);
    }
  });

  // 2. MAVROS State
  const stateSub = new ROSLIB.Topic({
    ros,
    name: '/mavros/state',
    messageType: 'mavros_msgs/msg/State',
  });
  stateSub.subscribe((msg) => {
    fcuState.set({
      connected: msg.connected,
      armed: msg.armed,
      guided: msg.guided,
      mode: msg.mode || 'UNKNOWN',
    });
  });

  // 3. Relative Altitude Above Home (Matches Mission Planner)
  let mavrosRelAlt = null;
  let groundZOrigin = null;

  const relAltSub = new ROSLIB.Topic({
    ros,
    name: '/mavros/global_position/rel_alt',
    messageType: 'std_msgs/msg/Float64',
  });
  relAltSub.subscribe((msg) => {
    mavrosRelAlt = msg.data;
  });

  // 4. Drone Pose
  const poseSub = new ROSLIB.Topic({
    ros,
    name: '/mavros/local_position/pose',
    messageType: 'geometry_msgs/msg/PoseStamped',
  });
  poseSub.subscribe((msg) => {
    const { x, y, z } = msg.pose.position;
    const { x: qx, y: qy, z: qz, w: qw } = msg.pose.orientation;
    const { roll, pitch, yaw } = quaternionToEuler(qx, qy, qz, qw);

    // Track ground altitude when disarmed
    let stateVal;
    fcuState.subscribe((s) => (stateVal = s))();
    if (!stateVal?.armed || groundZOrigin === null) {
      groundZOrigin = z;
    }

    const relAlt = (mavrosRelAlt !== null && !isNaN(mavrosRelAlt))
      ? mavrosRelAlt
      : (z - (groundZOrigin !== null ? groundZOrigin : 0));

    dronePose.set({
      x,
      y,
      z,
      rel_alt: relAlt,
      qx,
      qy,
      qz,
      qw,
      roll,
      pitch,
      yaw,
      timestamp: Date.now(),
    });
  });

  // 4. Drone Velocity
  const velSub = new ROSLIB.Topic({
    ros,
    name: '/mavros/local_position/velocity_local',
    messageType: 'geometry_msgs/msg/TwistStamped',
  });
  velSub.subscribe((msg) => {
    const { x: vx, y: vy, z: vz } = msg.twist.linear;
    droneVel.set({
      vx,
      vy,
      vz,
      speed: Math.hypot(vx, vy, vz),
    });
  });

  // 5. Refined Gate Poses (/estimator/refined_gate_poses)
  const gatePosesSub = new ROSLIB.Topic({
    ros,
    name: '/estimator/refined_gate_poses',
    messageType: 'geometry_msgs/msg/PoseArray',
  });
  gatePosesSub.subscribe((msg) => {
    estimatorFrameCount++;
    const now = performance.now();
    if (now - lastEstimatorFpsTime >= 1000) {
      gateEstimatorHz.set(Math.round((estimatorFrameCount * 1000) / (now - lastEstimatorFpsTime)));
      estimatorFrameCount = 0;
      lastEstimatorFpsTime = now;
    }

    if (msg && Array.isArray(msg.poses)) {
      refinedGatePoses.set(
        msg.poses.map((p, idx) => ({
          id: idx + 1,
          x: p.position.x,
          y: p.position.y,
          z: p.position.z,
          qx: p.orientation.x,
          qy: p.orientation.y,
          qz: p.orientation.z,
          qw: p.orientation.w,
        }))
      );
    }
  });

  // 6. Policy Observation (42D)
  const obsSub = new ROSLIB.Topic({
    ros,
    name: '/policy/observation',
    messageType: 'std_msgs/msg/Float64MultiArray',
  });
  obsSub.subscribe((msg) => {
    if (msg?.data && msg.data.length >= 42) {
      rawObservation.set(msg.data);
    }
  });

  // 7. Policy Action (3D)
  const actSub = new ROSLIB.Topic({
    ros,
    name: '/policy/action',
    messageType: 'std_msgs/msg/Float64MultiArray',
  });
  actSub.subscribe((msg) => {
    actionFrameCount++;
    const now = performance.now();
    if (now - lastActionFpsTime >= 1000) {
      policyActionHz.set(Math.round((actionFrameCount * 1000) / (now - lastActionFpsTime)));
      actionFrameCount = 0;
      lastActionFpsTime = now;
    }

    if (msg?.data && msg.data.length >= 3) {
      rawAction.set(msg.data);
    }
  });

  // 8. Action Space Maximum Limits Telemetry
  const actionLimitsSub = new ROSLIB.Topic({
    ros,
    name: '/controller/action_limits',
    messageType: 'std_msgs/msg/Float64MultiArray',
  });
  actionLimitsSub.subscribe((msg) => {
    if (msg?.data && msg.data.length >= 2) {
      maxActionMagnitude.set(Number(msg.data[0].toFixed(1)));
      maxYawRateDeg.set(Math.round(msg.data[1]));
    }
  });

  // 9. Gripper State (/gripper/state)
  const gripperSub = new ROSLIB.Topic({
    ros,
    name: '/gripper/state',
    messageType: 'std_msgs/msg/Bool',
  });
  gripperSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'boolean') {
      gripperOpened.set(msg.data);
    }
  });

  // 10. Gate 3.2 Ungrip Delay Telemetry (/controller/gate3_ungrip_delay)
  const ungripDelaySub = new ROSLIB.Topic({
    ros,
    name: '/controller/gate3_ungrip_delay',
    messageType: 'std_msgs/msg/Float64',
  });
  ungripDelaySub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      gate3UngripDelay.set(Number(msg.data.toFixed(2)));
    }
  });

  // 11. Landing Pad Position (/estimator/landing_pad)
  const landingPadSub = new ROSLIB.Topic({
    ros,
    name: '/estimator/landing_pad',
    messageType: 'geometry_msgs/msg/PoseStamped',
  });
  landingPadSub.subscribe((msg) => {
    if (msg?.pose?.position) {
      landingPadPose.set({
        x: msg.pose.position.x,
        y: msg.pose.position.y,
        z: msg.pose.position.z,
        valid: true,
      });
    }
  });

  // 12. Manual Landing Pad Status (/estimator/manual_landing_pad_active)
  const manualPadActiveSub = new ROSLIB.Topic({
    ros,
    name: '/estimator/manual_landing_pad_active',
    messageType: 'std_msgs/msg/Bool',
  });
  manualPadActiveSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'boolean') {
      manualLandingPadActive.set(msg.data);
    }
  });

  // Clear any stale perception subscription handle and resubscribe if active
  if (perceptionSub) {
    try {
      perceptionSub.unsubscribe();
    } catch (e) {}
    perceptionSub = null;
  }

  let active;
  isPerceptionStreamActive.subscribe((v) => (active = v))();
  if (active) {
    subscribePerceptionStream(true);
  }
}

let perceptionSub = null;
let lastFrameArrival = 0;
let lastResubscribeAttempt = 0;
export const isPerceptionStreamActive = writable(true);

export function subscribePerceptionStream(force = false) {
  if (!ros) return;
  if (perceptionSub && !force) return;

  // Clean up existing topic before recreating
  if (perceptionSub) {
    try {
      perceptionSub.unsubscribe();
    } catch (e) {}
    perceptionSub = null;
  }

  imageFrameCount = 0;
  lastFpsTime = performance.now();
  lastResubscribeAttempt = performance.now();

  perceptionSub = new ROSLIB.Topic({
    ros,
    name: '/perception/debug_image/compressed',
    messageType: 'sensor_msgs/msg/CompressedImage',
    compression: 'none',
    throttle_rate: 0,
    queue_size: 1,
  });

  perceptionSub.subscribe((msg) => {
    lastFrameArrival = performance.now();
    imageFrameCount++;
    const now = performance.now();
    let currentFps = 0;
    if (now - lastFpsTime >= 1000) {
      currentFps = Math.round((imageFrameCount * 1000) / (now - lastFpsTime));
      cameraFps.set(currentFps);
      imageFrameCount = 0;
      lastFpsTime = now;
    }

    if (msg.data) {
      debugImageSrc.set(`data:image/jpeg;base64,${msg.data}`);
    }

    debugImage.update((prev) => ({
      ...prev,
      fps: currentFps > 0 ? currentFps : prev.fps,
      timestamp: Date.now(),
    }));
  });

  isPerceptionStreamActive.set(true);
}

export function unsubscribePerceptionStream() {
  if (perceptionSub) {
    try {
      perceptionSub.unsubscribe();
    } catch (e) {}
    perceptionSub = null;
  }
  debugImageSrc.set('');
  cameraFps.set(0);
  debugImage.update((prev) => ({ ...prev, fps: 0 }));
  isPerceptionStreamActive.set(false);
}

export function togglePerceptionStream() {
  let active;
  isPerceptionStreamActive.subscribe((v) => (active = v))();
  if (active) {
    unsubscribePerceptionStream();
    addToast('Perception FPV stream paused (unsubscribed)', 'info', 2000);
  } else {
    subscribePerceptionStream(true);
    addToast('Perception FPV stream resumed (subscribed)', 'success', 2000);
  }
}

// Watchdog for camera stream liveness and auto-recovery on relaunch
if (typeof window !== 'undefined') {
  setInterval(() => {
    const now = performance.now();

    // 1. If frames haven't arrived for > 1500ms, reset camera FPS to 0
    if (lastFrameArrival > 0 && now - lastFrameArrival > 1500) {
      cameraFps.set(0);
      debugImage.update((prev) => ({ ...prev, fps: 0 }));
    }

    // 2. If ROS is connected and stream is active, but stalled for > 4000ms:
    // re-assert topic subscription to heal from publisher relaunch without page refresh
    let active = false;
    isPerceptionStreamActive.subscribe((v) => (active = v))();

    let status = '';
    connectionStatus.subscribe((v) => (status = v))();

    if (active && status === 'connected' && ros) {
      if (
        (lastFrameArrival === 0 || now - lastFrameArrival > 4000) &&
        now - lastResubscribeAttempt > 5000
      ) {
        subscribePerceptionStream(true);
      }
    }
  }, 1000);
}

// Call change_state service with interactive notifications
export function callChangeState(stateName) {
  if (stateName === 'HOVER' || stateName === 'OFF' || stateName === 'HOME' || stateName === 'FREE') {
    resetPolicyTelemetry();
  }

  if (!ros) {
    addToast('Cannot send command: ROS is not connected!', 'error', 4000);
    return;
  }

  addToast(`Requesting State: ${stateName}...`, 'info', 2000);

  const srv = new ROSLIB.Service({
    ros,
    name: '/controller/change_state',
    serviceType: 'mavros_controller/srv/SetString',
  });

  const request = new ROSLIB.ServiceRequest({
    data: stateName,
  });

  srv.callService(
    request,
    (result) => {
      if (result.success) {
        addToast(`State Changed: ${stateName} (${result.message})`, 'success', 3500);
      } else {
        addToast(`State Request Failed: ${result.message}`, 'warning', 4500);
      }

      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[${stateName}] ${result.message || (result.success ? 'Success' : 'Failed')}`,
          success: result.success,
        },
        ...logs.slice(0, 19),
      ]);
    },
    (error) => {
      addToast(`Service Call Error: ${error}`, 'error', 5000);
      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[${stateName}] Call failed: ${error}`,
          success: false,
        },
        ...logs.slice(0, 19),
      ]);
    }
  );
}

// Call /estimator/reset_gates service (re-anchors gates to current drone pose)
export function callResetGates() {
  if (!ros) {
    addToast('Cannot reset gates: ROS is not connected!', 'error', 4000);
    return;
  }

  addToast('Resetting Gate Estimation & Re-anchoring...', 'info', 2000);

  const srv = new ROSLIB.Service({
    ros,
    name: '/estimator/reset_gates',
    serviceType: 'std_srvs/srv/Trigger',
  });

  const request = new ROSLIB.ServiceRequest({});

  srv.callService(
    request,
    (result) => {
      if (result.success) {
        prevTargetGate = null;
        prevTargetSubGate = null;
        targetGateIndex.set(0);
        targetSubGateIndex.set(0);
        addToast(`✅ Gates Reset: ${result.message}`, 'success', 4000);
      } else {
        addToast(`⚠️ Gate Reset: ${result.message}`, 'warning', 4000);
      }

      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[RESET_GATES] ${result.message || (result.success ? 'Success' : 'Failed')}`,
          success: result.success,
        },
        ...logs.slice(0, 19),
      ]);
    },
    (error) => {
      addToast(`Gate Reset Failed: ${error}`, 'error', 5000);
      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[RESET_GATES] Call failed: ${error}`,
          success: false,
        },
        ...logs.slice(0, 19),
      ]);
    }
  );
}

// Call /estimator/tag_landing_pad service (sets landing pad to current drone pose)
export function callTagLandingPad() {
  if (!ros) {
    addToast('Cannot tag landing pad: ROS is not connected!', 'error', 4000);
    return;
  }

  addToast('Tagging current drone position as Landing Pad...', 'info', 2000);

  const srv = new ROSLIB.Service({
    ros,
    name: '/estimator/tag_landing_pad',
    serviceType: 'std_srvs/srv/Trigger',
  });

  const request = new ROSLIB.ServiceRequest({});

  srv.callService(
    request,
    (result) => {
      if (result.success) {
        addToast(`✅ Landing Pad Tagged: ${result.message}`, 'success', 5000);
      } else {
        addToast(`⚠️ Failed to Tag: ${result.message}`, 'warning', 4000);
      }
    },
    (err) => {
      addToast(`❌ Service Error: ${err}`, 'error', 4000);
    }
  );
}

// Call /estimator/reset_landing_pad service (clears manual override, reverts to auto-refined prior)
export function callResetLandingPad() {
  if (!ros) {
    addToast('Cannot reset landing pad: ROS is not connected!', 'error', 4000);
    return;
  }

  addToast('Resetting Landing Pad to auto-refined prior...', 'info', 2000);

  const srv = new ROSLIB.Service({
    ros,
    name: '/estimator/reset_landing_pad',
    serviceType: 'std_srvs/srv/Trigger',
  });

  const request = new ROSLIB.ServiceRequest({});

  srv.callService(
    request,
    (result) => {
      if (result.success) {
        addToast(`✅ Landing Pad Reverted: ${result.message}`, 'success', 4000);
      } else {
        addToast(`⚠️ Failed to Reset: ${result.message}`, 'warning', 4000);
      }
    },
    (err) => {
      addToast(`❌ Service Error: ${err}`, 'error', 4000);
    }
  );
}

// Manually change active target gate (0 = Gate #1, 1 = Gate #2, etc.)
export function setTargetGate(index) {
  if (!ros) {
    addToast('Cannot switch gate: ROS is not connected!', 'error', 3000);
    return;
  }
  const targetIndex = Math.max(0, Math.min(4, Math.floor(index)));
  const topic = new ROSLIB.Topic({
    ros,
    name: '/controller/set_target_gate',
    messageType: 'std_msgs/msg/Int32',
  });
  topic.publish(new ROSLIB.Message({ data: targetIndex }));
  targetGateIndex.set(targetIndex);
  targetSubGateIndex.set(0);
  addToast(`🎯 Active Target switched to: GATE #${targetIndex + 1}`, 'info', 2000);
}

// Dynamically set Maximum Action Space Magnitude (Forward and Lateral Combined, 1.0 - 16.0 m/s)
export function setMaxActionMagnitude(val) {
  const clamped = Math.max(1.0, Math.min(16.0, parseFloat(val) || 16.0));
  maxActionMagnitude.set(Number(clamped.toFixed(1)));
  if (!ros) return;
  const topic = new ROSLIB.Topic({
    ros,
    name: '/controller/set_max_action_magnitude',
    messageType: 'std_msgs/msg/Float64',
  });
  topic.publish(new ROSLIB.Message({ data: clamped }));
}

// Dynamically set Maximum Yaw Rate Limit (20 - 360 deg/s)
export function setMaxYawRateDeg(valDeg) {
  const clamped = Math.max(20.0, Math.min(360.0, parseFloat(valDeg) || 360.0));
  maxYawRateDeg.set(Math.round(clamped));
  if (!ros) return;
  const topic = new ROSLIB.Topic({
    ros,
    name: '/controller/set_max_yaw_rate',
    messageType: 'std_msgs/msg/Float64',
  });
  topic.publish(new ROSLIB.Message({ data: clamped }));
}

// Dynamically set Gate 3.2 Ungrip Delay in seconds (0.0 to 10.0s)
export function setGate3UngripDelay(val) {
  const clamped = Math.max(0.0, Math.min(10.0, parseFloat(val) || 0.0));
  gate3UngripDelay.set(Number(clamped.toFixed(2)));
  if (!ros) return;
  const topic = new ROSLIB.Topic({
    ros,
    name: '/controller/set_gate3_ungrip_delay',
    messageType: 'std_msgs/msg/Float64',
  });
  topic.publish(new ROSLIB.Message({ data: clamped }));
  addToast(`⏳ Gate 3.2 Ungrip Delay set to ${clamped.toFixed(2)}s`, 'info', 1500);
}

// Call /gripper/set_state service (std_srvs/srv/SetBool)
export function callSetGripperState(openState) {
  if (!ros) {
    addToast('Cannot control gripper: ROS is not connected!', 'error', 3500);
    return;
  }

  const targetStr = openState ? 'OPEN' : 'CLOSE';
  addToast(`Commanding Gripper: ${targetStr}...`, 'info', 1500);
  gripperPending.set(true);

  const srv = new ROSLIB.Service({
    ros,
    name: '/gripper/set_state',
    serviceType: 'std_srvs/srv/SetBool',
  });

  const request = new ROSLIB.ServiceRequest({
    data: !!openState,
  });

  srv.callService(
    request,
    (result) => {
      gripperPending.set(false);
      if (result.success) {
        gripperOpened.set(!!openState);
        addToast(`✅ Gripper ${openState ? 'OPENED' : 'CLOSED'}`, 'success', 3000);
      } else {
        addToast(`⚠️ Gripper command: ${result.message}`, 'warning', 4000);
      }

      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[GRIPPER_${targetStr}] ${result.message || (result.success ? 'Success' : 'Failed')}`,
          success: result.success,
        },
        ...logs.slice(0, 19),
      ]);
    },
    (error) => {
      gripperPending.set(false);
      addToast(`Gripper Service Error: ${error}`, 'error', 4500);
      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[GRIPPER_${targetStr}] Call failed: ${error}`,
          success: false,
        },
        ...logs.slice(0, 19),
      ]);
    }
  );
}

// Toggle gripper state helper
export function toggleGripper() {
  let current = false;
  gripperOpened.subscribe((v) => (current = v))();
  callSetGripperState(!current);
}

// Option 1: Flush Gyro Bias (MAV_CMD_PREFLIGHT_CALIBRATION 241) & Reset EKF Home/Origin
export function callFlushGyroAndHome() {
  if (!ros) {
    addToast('Cannot flush gyro: ROS is not connected!', 'error', 4000);
    return;
  }

  addToast('Flushing Gyro Zero-Bias & Resetting Origin...', 'info', 2500);

  const calibSrv = new ROSLIB.Service({
    ros,
    name: '/mavros/cmd/command',
    serviceType: 'mavros_msgs/srv/CommandLong',
  });

  const calibReq = new ROSLIB.ServiceRequest({
    broadcast: false,
    command: 241, // MAV_CMD_PREFLIGHT_CALIBRATION
    confirmation: 0,
    param1: 1.0,  // 3D Gyro calibration
    param2: 0.0,
    param3: 0.0,
    param4: 0.0,
    param5: 0.0,
    param6: 0.0,
    param7: 0.0,
  });

  calibSrv.callService(
    calibReq,
    (res) => {
      if (res.success) {
        addToast('✅ Gyro Bias Calibrated & Flushed', 'success', 3500);

        // Also call set_home to zero out local position residue
        const homeSrv = new ROSLIB.Service({
          ros,
          name: '/mavros/cmd/set_home',
          serviceType: 'mavros_msgs/srv/CommandHome',
        });
        const homeReq = new ROSLIB.ServiceRequest({
          current_gps: true,
          latitude: 0.0,
          longitude: 0.0,
          altitude: 0.0,
        });
        homeSrv.callService(homeReq, (hRes) => {
          if (hRes.success) {
            addToast('✅ EKF Local Origin Zeroed at Drone Pose', 'success', 3500);
          }
        });
      } else {
        addToast(`⚠️ Gyro Calib command returned result code: ${res.result}`, 'warning', 4000);
      }

      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[FLUSH_GYRO] Result: ${res.result || (res.success ? 'Success' : 'Failed')}`,
          success: res.success,
        },
        ...logs.slice(0, 19),
      ]);
    },
    (err) => {
      addToast(`Gyro Calib Service Error: ${err}`, 'error', 4000);
    }
  );
}

// Option 2: Full Soft-Reboot of Autopilot / EKF3 (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 246)
export function callRebootAutopilot() {
  if (!ros) {
    addToast('Cannot reboot autopilot: ROS is not connected!', 'error', 4000);
    return;
  }

  addToast('Rebooting ArduPilot Autopilot & EKF3...', 'warning', 3000);

  const srv = new ROSLIB.Service({
    ros,
    name: '/mavros/cmd/command',
    serviceType: 'mavros_msgs/srv/CommandLong',
  });

  const req = new ROSLIB.ServiceRequest({
    broadcast: false,
    command: 246, // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    confirmation: 0,
    param1: 1.0,  // Reboot autopilot
    param2: 0.0,
    param3: 0.0,
    param4: 0.0,
    param5: 0.0,
    param6: 0.0,
    param7: 0.0,
  });

  srv.callService(
    req,
    (res) => {
      if (res.success) {
        addToast('⚡ Autopilot reboot command sent! EKF3 reinitializing...', 'success', 5000);
      } else {
        addToast(`⚠️ Reboot command returned result code: ${res.result}`, 'warning', 4000);
      }

      serviceResponseLog.update((logs) => [
        {
          time: new Date().toLocaleTimeString(),
          text: `[REBOOT_AUTOPILOT] Result: ${res.result || (res.success ? 'Success' : 'Failed')}`,
          success: res.success,
        },
        ...logs.slice(0, 19),
      ]);
    },
    (err) => {
      addToast(`Autopilot Reboot Error: ${err}`, 'error', 4000);
    }
  );
}

