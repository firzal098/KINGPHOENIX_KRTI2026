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

// Active Target Gate Index (0 = Gate #1, 1 = Gate #2, etc.)
export const targetGateIndex = writable(0);
export const targetGateLabel = derived(targetGateIndex, ($idx) => {
  if ($idx >= 5) return 'ALL GATES CLEARED (5/5)';
  return `GATE #${$idx + 1} (${$idx + 1}/5)`;
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

// Raw 40D Observation Array
export const rawObservation = writable(new Array(40).fill(0.0));

// Structured Observation Derived Store
export const structuredObservation = derived(rawObservation, ($obs) => {
  if (!$obs || $obs.length < 40) {
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

  // 3. Active Gate 15D [6:21]
  const active_gate = {
    tl: [$obs[6] || 0.0, $obs[7] || 0.0, $obs[8] || 0.0],
    tr: [$obs[9] || 0.0, $obs[10] || 0.0, $obs[11] || 0.0],
    bl: [$obs[12] || 0.0, $obs[13] || 0.0, $obs[14] || 0.0],
    br: [$obs[15] || 0.0, $obs[16] || 0.0, $obs[17] || 0.0],
    center: [$obs[18] || 0.0, $obs[19] || 0.0, $obs[20] || 0.0],
    dist: Math.hypot($obs[18] || 0, $obs[19] || 0, $obs[20] || 0),
  };

  // 4. Next Gate 15D [21:36]
  const isZero = $obs.slice(21, 36).every((v) => Math.abs(v) < 1e-5);
  const next_gate = {
    has_next: !isZero,
    tl: [$obs[21] || 0.0, $obs[22] || 0.0, $obs[23] || 0.0],
    tr: [$obs[24] || 0.0, $obs[25] || 0.0, $obs[26] || 0.0],
    bl: [$obs[27] || 0.0, $obs[28] || 0.0, $obs[29] || 0.0],
    br: [$obs[30] || 0.0, $obs[31] || 0.0, $obs[32] || 0.0],
    center: [$obs[33] || 0.0, $obs[34] || 0.0, $obs[35] || 0.0],
    dist: Math.hypot($obs[33] || 0, $obs[34] || 0, $obs[35] || 0),
  };

  // 5. Previous Action [36:40]
  const prev_action = {
    vfwd: $obs[36] || 0.0,
    vleft: $obs[37] || 0.0,
    vup: $obs[38] || 0.0,
    yawRate: $obs[39] || 0.0,
    yawRateDeg: ($obs[39] || 0.0) * (180.0 / Math.PI),
  };

  return { vel_B, grav_B, active_gate, next_gate, prev_action };
});

// Raw 4D Action Array
export const rawAction = writable([0.0, 0.0, 0.0, 0.0]);

// Structured Action Derived Store
export const structuredAction = derived(rawAction, ($act) => {
  const vfwd = $act[0] || 0.0;
  const vleft = $act[1] || 0.0;
  const vup = $act[2] || 0.0;
  const yawRate = $act[3] || 0.0;
  return {
    vfwd,
    vleft,
    vup,
    yawRate,
    yawRateDeg: yawRate * (180.0 / Math.PI),
    totalSpeed: Math.hypot(vfwd, vleft, vup),
  };
});

export const serviceResponseLog = writable([]);

// ROS instance and topics
let ros = null;
let reconnectTimer = null;
let imageFrameCount = 0;
let lastFpsTime = performance.now();

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
    connectionError.set(error?.message || `Failed to connect to ${targetUrl}`);
  });

  ros.on('close', () => {
    connectionStatus.set('disconnected');
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        initRosConnection(targetUrl);
      }, 3000);
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
      controllerFsmState.set(msg.data.toUpperCase());
    }
  });

  // 1b. Active Target Gate Index (/controller/target_gate_index)
  const targetGateSub = new ROSLIB.Topic({
    ros,
    name: '/controller/target_gate_index',
    messageType: 'std_msgs/msg/Int32',
  });
  targetGateSub.subscribe((msg) => {
    if (msg && typeof msg.data === 'number') {
      targetGateIndex.set(msg.data);
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

  // 3. Drone Pose
  const poseSub = new ROSLIB.Topic({
    ros,
    name: '/mavros/local_position/pose',
    messageType: 'geometry_msgs/msg/PoseStamped',
  });
  poseSub.subscribe((msg) => {
    const { x, y, z } = msg.pose.position;
    const { x: qx, y: qy, z: qz, w: qw } = msg.pose.orientation;
    const { roll, pitch, yaw } = quaternionToEuler(qx, qy, qz, qw);
    dronePose.set({
      x,
      y,
      z,
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

  // 6. Policy Observation (40D)
  const obsSub = new ROSLIB.Topic({
    ros,
    name: '/policy/observation',
    messageType: 'std_msgs/msg/Float64MultiArray',
  });
  obsSub.subscribe((msg) => {
    if (msg.data && msg.data.length >= 40) {
      rawObservation.set(msg.data);
      structuredObservation.set(parseObservation40D(msg.data));
    }
  });

  // 7. Policy Action (4D)
  const actSub = new ROSLIB.Topic({
    ros,
    name: '/policy/action',
    messageType: 'std_msgs/msg/Float64MultiArray',
  });
  actSub.subscribe((msg) => {
    if (msg.data && msg.data.length >= 4) {
      rawAction.set(msg.data);
    }
  });

  // Subscribe to perception stream if active
  let active;
  isPerceptionStreamActive.subscribe((v) => (active = v))();
  if (active) {
    subscribePerceptionStream();
  }
}

let perceptionSub = null;
export const isPerceptionStreamActive = writable(true);

export function subscribePerceptionStream() {
  if (!ros || perceptionSub) return;

  perceptionSub = new ROSLIB.Topic({
    ros,
    name: '/perception/debug_image/compressed',
    messageType: 'sensor_msgs/msg/CompressedImage',
  });

  perceptionSub.subscribe((msg) => {
    imageFrameCount++;
    const now = performance.now();
    let currentFps = 0;
    if (now - lastFpsTime >= 1000) {
      currentFps = Math.round((imageFrameCount * 1000) / (now - lastFpsTime));
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
    subscribePerceptionStream();
    addToast('Perception FPV stream resumed (subscribed)', 'success', 2000);
  }
}

// Call change_state service with interactive notifications
export function callChangeState(stateName) {
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
