<script>
  import {
    connectionStatus,
    connectionError,
    rosbridgeUrl,
    controllerFsmState,
    targetGateLabel,
    fcuState,
    dronePose,
    droneVel,
    callChangeState,
    initRosConnection,
    serviceResponseLog
  } from '../ros.js';

  let showConfig = false;
  let customUrl = '';
  let clickedAction = null;

  // RUN Lap Timer
  let runElapsedSec = 0;
  let runTimerInterval = null;
  let lastFsmState = 'OFF';
  let runStartTime = null;

  $: handleFsmStateChange($controllerFsmState);

  function handleFsmStateChange(newState) {
    if (newState === lastFsmState) return;

    if (newState === 'RUN') {
      // Switching to RUN state: reset previous timer to 0 and start counting
      runElapsedSec = 0;
      runStartTime = performance.now();
      if (runTimerInterval) clearInterval(runTimerInterval);
      runTimerInterval = setInterval(() => {
        runElapsedSec = (performance.now() - runStartTime) / 1000.0;
      }, 50);
    } else {
      // Switched out of RUN state: stop timer, DO NOT reset (freeze final lap time)
      if (runTimerInterval) {
        clearInterval(runTimerInterval);
        runTimerInterval = null;
        if (runStartTime && lastFsmState === 'RUN') {
          runElapsedSec = (performance.now() - runStartTime) / 1000.0;
        }
      }
    }
    lastFsmState = newState;
  }

  function formatTime(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = (seconds % 60).toFixed(2).padStart(5, '0');
    return `${mins.toString().padStart(2, '0')}:${secs}`;
  }

  $: if ($rosbridgeUrl && !customUrl) {
    customUrl = $rosbridgeUrl;
  }

  function handleConnect() {
    const urlToUse = customUrl || $rosbridgeUrl;
    rosbridgeUrl.set(urlToUse);
    initRosConnection(urlToUse);
    showConfig = false;
  }

  function triggerState(name) {
    clickedAction = name;
    callChangeState(name);
    setTimeout(() => {
      clickedAction = null;
    }, 800);
  }

  function getStateClass(fsm) {
    switch (fsm) {
      case 'RUN':
        return 'fsm-run';
      case 'HOVER':
        return 'fsm-hover';
      case 'HOME':
        return 'fsm-home';
      case 'FREE':
        return 'fsm-free';
      case 'CLIMBING':
      case 'TAKEOFF':
      case 'ARMING':
      case 'SET_MODE_GUIDED':
        return 'fsm-climb';
      case 'LANDING':
        return 'fsm-landing';
      case 'OFF':
      default:
        return 'fsm-off';
    }
  }
</script>

<header class="gcs-card header-container">
  <!-- Brand & Active State Badge -->
  <div class="brand-section">
    <div class="brand-icon">
      <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <polygon points="12 2 2 7 12 12 22 7 12 2" />
        <polyline points="2 17 12 22 22 17" />
        <polyline points="2 12 12 17 22 12" />
      </svg>
    </div>
    <div>
      <div class="brand-title font-hud">KINGPHOENIX <span class="brand-sub">GCS</span></div>
      <div class="brand-version font-mono">KRTI 2026 // GATE RUNNER</div>
    </div>

    <!-- Prominent Active FSM State Indicator -->
    <div class="fsm-badge-wrapper">
      <span class="fsm-pill {getStateClass($controllerFsmState)} font-hud">
        <span class="fsm-pulse"></span>
        <span class="fsm-title">STATE:</span>
        <span class="fsm-name">{$controllerFsmState}</span>
      </span>
    </div>
  </div>

  <!-- Telemetry Quick Badges -->
  <div class="telemetry-bar">
    <!-- WS Link -->
    <button class="ws-pill" on:click={() => (showConfig = !showConfig)} title="Click to configure WebSocket">
      <span class="badge {$connectionStatus === 'connected' ? 'badge-cyan' : $connectionStatus === 'connecting' ? 'badge-amber' : 'badge-red'}">
        <span class="badge-dot"></span>
        WS: {$connectionStatus}
      </span>
    </button>

    <!-- FCU State -->
    <div class="telemetry-item">
      <span class="label">FCU:</span>
      <span class="badge {$fcuState.connected ? 'badge-green' : 'badge-red'}">
        {$fcuState.connected ? 'LINK OK' : 'NO LINK'}
      </span>
    </div>

    <!-- Arming State -->
    <div class="telemetry-item">
      <span class="label">MOTORS:</span>
      <span class="badge {$fcuState.armed ? 'badge-amber' : 'badge-cyan'}">
        {$fcuState.armed ? 'ARMED' : 'DISARMED'}
      </span>
    </div>

    <!-- Flight Mode -->
    <div class="telemetry-item">
      <span class="label">MODE:</span>
      <span class="mode-pill font-mono">{$fcuState.mode}</span>
    </div>

    <!-- Active Target Gate Indicator -->
    <div class="telemetry-item">
      <span class="label">TARGET:</span>
      <span class="target-gate-badge font-hud">
        <span class="gate-icon">🎯</span>
        {$targetGateLabel}
      </span>
    </div>

    <!-- RUN Lap Timer -->
    <div class="telemetry-item lap-timer-item">
      <span class="label">LAP:</span>
      <span class="lap-timer-badge font-hud {$controllerFsmState === 'RUN' ? 'timer-running' : runElapsedSec > 0 ? 'timer-stopped' : ''}">
        <span class="timer-icon">⏱️</span>
        <span class="font-mono timer-value">{formatTime(runElapsedSec)}</span>
      </span>
    </div>

    <!-- Quick Altitude & Speed -->
    <div class="quick-stats">
      <div class="stat-box">
        <span class="stat-lbl">ALT (AGL)</span>
        <span class="stat-val font-mono">{($dronePose.rel_alt !== undefined ? $dronePose.rel_alt : $dronePose.z).toFixed(2)} <small>m</small></span>
      </div>
      <div class="stat-box">
        <span class="stat-lbl">SPEED</span>
        <span class="stat-val font-mono">{$droneVel.speed.toFixed(2)} <small>m/s</small></span>
      </div>
    </div>
  </div>

  <!-- Command Actions (change_state) -->
  <div class="actions-section">
    <!-- HOVER Button -->
    <button
      class="btn-action btn-hover {$controllerFsmState === 'HOVER' ? 'btn-current-active' : ''} {clickedAction === 'HOVER' ? 'btn-clicking' : ''}"
      on:click={() => triggerState('HOVER')}
      title="Takeoff to 1.0m and maintain hover"
    >
      <span class="btn-glow"></span>
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <path d="M12 19V5M5 12l7-7 7 7"/>
      </svg>
      <span>HOVER</span>
    </button>

    <!-- RUN RL Button -->
    <button
      class="btn-action btn-run {$controllerFsmState === 'RUN' ? 'btn-current-active' : ''} {clickedAction === 'RUN' ? 'btn-clicking' : ''}"
      on:click={() => triggerState('RUN')}
      title="Engage neural policy gate traversal"
    >
      <span class="btn-glow"></span>
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <polygon points="5 3 19 12 5 21 5 3"/>
      </svg>
      <span>RUN RL</span>
    </button>

    <!-- HOME Button -->
    <button
      class="btn-action btn-home {$controllerFsmState === 'HOME' ? 'btn-current-active' : ''} {clickedAction === 'HOME' ? 'btn-clicking' : ''}"
      on:click={() => triggerState('HOME')}
      title="Fly to Home Waypoint"
    >
      <span class="btn-glow"></span>
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/>
        <polyline points="9 22 9 12 15 12 15 22"/>
      </svg>
      <span>HOME</span>
    </button>

    <!-- FREE / RC Manual Button -->
    <button
      class="btn-action btn-free {$controllerFsmState === 'FREE' ? 'btn-current-active' : ''} {clickedAction === 'FREE' ? 'btn-clicking' : ''}"
      on:click={() => triggerState('FREE')}
      title="Release full control to manual RC (no auto arm/disarm/setpoints)"
    >
      <span class="btn-glow"></span>
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <rect x="2" y="6" width="20" height="12" rx="3"/>
        <circle cx="6" cy="12" r="2"/>
        <circle cx="18" cy="12" r="2"/>
        <path d="M12 9v6M9 12h6"/>
      </svg>
      <span>FREE (RC)</span>
    </button>

    <!-- OFF / LAND Button -->
    <button
      class="btn-action btn-off {$controllerFsmState === 'OFF' ? 'btn-current-active' : ''} {clickedAction === 'OFF' ? 'btn-clicking' : ''}"
      on:click={() => triggerState('OFF')}
      title="Emergency Land / Force Disarm"
    >
      <span class="btn-glow"></span>
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <rect x="3" y="3" width="18" height="18" rx="2" ry="2"/>
      </svg>
      <span>OFF / LAND</span>
    </button>
  </div>
</header>

<!-- Connection Config Modal -->
{#if showConfig}
  <div class="modal-backdrop" on:click|self={() => (showConfig = false)}>
    <div class="gcs-card modal-box">
      <h3 class="font-hud">ROSBridge WebSocket Settings</h3>
      <p class="modal-desc">Configure the WebSocket address for ROS 2 rosbridge_server (default: ws://localhost:9090).</p>
      
      <div class="input-group">
        <label for="ws-url">WebSocket URL</label>
        <input id="ws-url" type="text" bind:value={customUrl} class="font-mono" placeholder="ws://localhost:9090" />
      </div>

      {#if $connectionError}
        <div class="error-banner font-mono">{$connectionError}</div>
      {/if}

      <div class="modal-actions">
        <button class="btn-secondary" on:click={() => (showConfig = false)}>Cancel</button>
        <button class="btn-primary" on:click={handleConnect}>Connect</button>
      </div>

      {#if $serviceResponseLog.length > 0}
        <div class="service-log-box">
          <div class="log-title font-hud">Service Activity Log</div>
          <div class="log-entries font-mono">
            {#each $serviceResponseLog.slice(0, 5) as log}
              <div class="log-row {log.success ? 'log-ok' : 'log-err'}">
                <span class="log-time">[{log.time}]</span> {log.text}
              </div>
            {/each}
          </div>
        </div>
      {/if}
    </div>
  </div>
{/if}

<style>
  .header-container {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 12px 20px;
    margin-bottom: 16px;
    gap: 16px;
    flex-wrap: wrap;
  }

  .brand-section {
    display: flex;
    align-items: center;
    gap: 12px;
  }

  .brand-icon {
    width: 38px;
    height: 38px;
    border-radius: 8px;
    background: linear-gradient(135deg, rgba(0, 240, 255, 0.2), rgba(168, 85, 247, 0.2));
    border: 1px solid var(--border-glow);
    display: flex;
    align-items: center;
    justify-content: center;
    color: var(--accent-cyan);
    box-shadow: 0 0 12px var(--accent-cyan-glow);
  }

  .brand-title {
    font-size: 1.15rem;
    font-weight: 700;
    color: var(--text-primary);
  }

  .brand-sub {
    color: var(--accent-cyan);
  }

  .brand-version {
    font-size: 0.68rem;
    color: var(--text-muted);
    letter-spacing: 0.08em;
  }

  /* FSM Badge */
  .fsm-badge-wrapper {
    margin-left: 10px;
    border-left: 1px solid var(--border-subtle);
    padding-left: 14px;
  }

  .fsm-pill {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 5px 12px;
    border-radius: 6px;
    font-size: 0.82rem;
    font-weight: 800;
    letter-spacing: 0.06em;
    border: 1px solid transparent;
    transition: all 0.2s ease;
  }

  .fsm-pulse {
    width: 8px;
    height: 8px;
    border-radius: 50%;
  }

  .fsm-title {
    opacity: 0.65;
    font-size: 0.7rem;
  }

  .fsm-run {
    background: rgba(0, 240, 255, 0.18);
    color: var(--accent-cyan);
    border-color: rgba(0, 240, 255, 0.5);
    box-shadow: 0 0 18px rgba(0, 240, 255, 0.35);
  }
  .fsm-run .fsm-pulse {
    background: var(--accent-cyan);
    box-shadow: 0 0 8px var(--accent-cyan);
    animation: blink 1s infinite alternate;
  }

  .fsm-hover {
    background: rgba(16, 185, 129, 0.18);
    color: var(--accent-green);
    border-color: rgba(16, 185, 129, 0.5);
    box-shadow: 0 0 18px rgba(16, 185, 129, 0.35);
  }
  .fsm-hover .fsm-pulse {
    background: var(--accent-green);
    box-shadow: 0 0 8px var(--accent-green);
  }

  .fsm-climb {
    background: rgba(245, 158, 11, 0.18);
    color: var(--accent-amber);
    border-color: rgba(245, 158, 11, 0.5);
    box-shadow: 0 0 18px rgba(245, 158, 11, 0.35);
  }
  .fsm-climb .fsm-pulse {
    background: var(--accent-amber);
    box-shadow: 0 0 8px var(--accent-amber);
    animation: blink 0.6s infinite alternate;
  }

  .fsm-landing {
    background: rgba(239, 68, 68, 0.18);
    color: var(--accent-red);
    border-color: rgba(239, 68, 68, 0.5);
    box-shadow: 0 0 18px rgba(239, 68, 68, 0.35);
  }
  .fsm-landing .fsm-pulse {
    background: var(--accent-red);
    box-shadow: 0 0 8px var(--accent-red);
    animation: blink 0.5s infinite alternate;
  }

  .fsm-home {
    background: rgba(59, 130, 246, 0.18);
    color: #60a5fa;
    border-color: rgba(59, 130, 246, 0.5);
    box-shadow: 0 0 18px rgba(59, 130, 246, 0.35);
  }
  .fsm-home .fsm-pulse {
    background: #60a5fa;
    box-shadow: 0 0 8px #60a5fa;
    animation: blink 0.8s infinite alternate;
  }

  .fsm-free {
    background: rgba(168, 85, 247, 0.18);
    color: #c084fc;
    border-color: rgba(168, 85, 247, 0.5);
    box-shadow: 0 0 18px rgba(168, 85, 247, 0.35);
  }
  .fsm-free .fsm-pulse {
    background: #c084fc;
    box-shadow: 0 0 8px #c084fc;
  }

  .fsm-off {
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-muted);
    border-color: var(--border-subtle);
  }
  .fsm-off .fsm-pulse {
    background: var(--text-muted);
  }

  @keyframes blink {
    from { opacity: 0.3; transform: scale(0.85); }
    to { opacity: 1; transform: scale(1.15); }
  }

  /* Telemetry Bar */
  .telemetry-bar {
    display: flex;
    align-items: center;
    gap: 14px;
    background: rgba(0, 0, 0, 0.3);
    padding: 6px 14px;
    border-radius: 8px;
    border: 1px solid var(--border-subtle);
  }

  .ws-pill {
    background: transparent;
    border: none;
    cursor: pointer;
    padding: 0;
  }

  .telemetry-item {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.78rem;
  }

  .label {
    color: var(--text-muted);
    font-weight: 600;
  }

  .mode-pill {
    background: rgba(255, 255, 255, 0.06);
    padding: 2px 8px;
    border-radius: 4px;
    font-weight: 700;
    color: var(--accent-cyan);
    font-size: 0.75rem;
    border: 1px solid rgba(0, 240, 255, 0.2);
  }

  .target-gate-badge {
    display: inline-flex;
    align-items: center;
    gap: 4px;
    background: rgba(16, 185, 129, 0.15);
    color: #34d399;
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 0.74rem;
    font-weight: 800;
    letter-spacing: 0.5px;
    border: 1px solid rgba(16, 185, 129, 0.35);
    box-shadow: 0 0 10px rgba(16, 185, 129, 0.15);
    transition: all 0.2s ease;
  }

  .gate-icon {
    font-size: 0.8rem;
  }

  /* Lap Timer */
  .lap-timer-item {
    margin-left: 2px;
  }

  .lap-timer-badge {
    display: inline-flex;
    align-items: center;
    gap: 5px;
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-secondary);
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 0.74rem;
    font-weight: 700;
    border: 1px solid var(--border-subtle);
    transition: all 0.2s ease;
  }

  .lap-timer-badge.timer-running {
    background: rgba(0, 240, 255, 0.15);
    color: var(--accent-cyan);
    border-color: rgba(0, 240, 255, 0.5);
    box-shadow: 0 0 14px rgba(0, 240, 255, 0.35);
    animation: pulse-timer 1.5s infinite ease-in-out;
  }

  .lap-timer-badge.timer-stopped {
    background: rgba(245, 158, 11, 0.15);
    color: #fbbf24;
    border-color: rgba(245, 158, 11, 0.4);
    box-shadow: 0 0 10px rgba(245, 158, 11, 0.2);
  }

  .timer-icon {
    font-size: 0.78rem;
  }

  .timer-value {
    letter-spacing: 0.05em;
    font-size: 0.78rem;
  }

  @keyframes pulse-timer {
    0%, 100% { box-shadow: 0 0 8px rgba(0, 240, 255, 0.25); }
    50% { box-shadow: 0 0 16px rgba(0, 240, 255, 0.55); }
  }

  .quick-stats {
    display: flex;
    align-items: center;
    gap: 12px;
    border-left: 1px solid var(--border-subtle);
    padding-left: 12px;
  }

  .stat-box {
    display: flex;
    flex-direction: column;
  }

  .stat-lbl {
    font-size: 0.62rem;
    color: var(--text-muted);
    font-weight: 600;
  }

  .stat-val {
    font-size: 0.85rem;
    font-weight: 700;
    color: var(--accent-cyan);
  }

  .stat-val small {
    font-size: 0.65rem;
    color: var(--text-secondary);
  }

  /* Actions */
  .actions-section {
    display: flex;
    align-items: center;
    gap: 10px;
  }

  .btn-action {
    position: relative;
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 9px 18px;
    border-radius: 8px;
    font-family: var(--font-hud);
    font-size: 0.88rem;
    font-weight: 700;
    letter-spacing: 0.04em;
    cursor: pointer;
    border: 1px solid transparent;
    transition: all 0.18s ease;
    overflow: hidden;
  }

  .btn-action:hover {
    transform: translateY(-1px);
  }

  .btn-clicking {
    transform: scale(0.94) !important;
  }

  .btn-hover {
    background: rgba(16, 185, 129, 0.12);
    color: var(--accent-green);
    border-color: rgba(16, 185, 129, 0.35);
  }
  .btn-hover:hover, .btn-hover.btn-current-active {
    background: rgba(16, 185, 129, 0.28);
    border-color: var(--accent-green);
    box-shadow: 0 0 16px var(--accent-green-glow);
  }

  .btn-run {
    background: rgba(0, 240, 255, 0.12);
    color: var(--accent-cyan);
    border-color: rgba(0, 240, 255, 0.35);
  }
  .btn-run:hover, .btn-run.btn-current-active {
    background: rgba(0, 240, 255, 0.28);
    border-color: var(--accent-cyan);
    box-shadow: 0 0 16px var(--accent-cyan-glow);
  }

  .btn-home {
    background: rgba(59, 130, 246, 0.12);
    color: #60a5fa;
    border-color: rgba(59, 130, 246, 0.35);
  }
  .btn-home:hover, .btn-home.btn-current-active {
    background: rgba(59, 130, 246, 0.28);
    border-color: #60a5fa;
    box-shadow: 0 0 16px rgba(59, 130, 246, 0.4);
  }

  .btn-free {
    background: rgba(168, 85, 247, 0.12);
    color: #c084fc;
    border-color: rgba(168, 85, 247, 0.35);
  }
  .btn-free:hover, .btn-free.btn-current-active {
    background: rgba(168, 85, 247, 0.28);
    border-color: #c084fc;
    box-shadow: 0 0 16px rgba(168, 85, 247, 0.4);
  }

  .btn-off {
    background: rgba(239, 68, 68, 0.12);
    color: var(--accent-red);
    border-color: rgba(239, 68, 68, 0.35);
  }
  .btn-off:hover, .btn-off.btn-current-active {
    background: rgba(239, 68, 68, 0.28);
    border-color: var(--accent-red);
    box-shadow: 0 0 16px var(--accent-red-glow);
  }

  /* Modal */
  .modal-backdrop {
    position: fixed;
    inset: 0;
    background: rgba(0, 0, 0, 0.7);
    backdrop-filter: blur(8px);
    z-index: 1000;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .modal-box {
    width: 480px;
    max-width: 90vw;
    padding: 24px;
    background: #111827;
    border: 1px solid var(--border-glow);
  }

  .modal-desc {
    font-size: 0.85rem;
    color: var(--text-secondary);
    margin: 8px 0 16px;
  }

  .input-group {
    display: flex;
    flex-direction: column;
    gap: 6px;
    margin-bottom: 16px;
  }

  .input-group label {
    font-size: 0.75rem;
    color: var(--text-muted);
    font-weight: 600;
  }

  .input-group input {
    background: rgba(0, 0, 0, 0.4);
    border: 1px solid var(--border-subtle);
    padding: 8px 12px;
    border-radius: 6px;
    color: var(--accent-cyan);
    font-size: 0.9rem;
    outline: none;
  }
  .input-group input:focus {
    border-color: var(--accent-cyan);
    box-shadow: 0 0 8px var(--accent-cyan-glow);
  }

  .modal-actions {
    display: flex;
    justify-content: flex-end;
    gap: 10px;
  }

  .btn-primary {
    background: var(--accent-cyan);
    color: #000;
    border: none;
    padding: 8px 18px;
    border-radius: 6px;
    font-weight: 700;
    cursor: pointer;
  }

  .btn-secondary {
    background: transparent;
    color: var(--text-secondary);
    border: 1px solid var(--border-subtle);
    padding: 8px 18px;
    border-radius: 6px;
    cursor: pointer;
  }

  .error-banner {
    background: rgba(239, 68, 68, 0.15);
    color: var(--accent-red);
    border: 1px solid rgba(239, 68, 68, 0.3);
    padding: 8px;
    border-radius: 6px;
    font-size: 0.78rem;
    margin-bottom: 14px;
  }

  .service-log-box {
    margin-top: 18px;
    border-top: 1px solid var(--border-subtle);
    padding-top: 12px;
  }

  .log-title {
    font-size: 0.8rem;
    color: var(--text-muted);
    margin-bottom: 6px;
  }

  .log-entries {
    font-size: 0.72rem;
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .log-row {
    padding: 3px 6px;
    border-radius: 4px;
  }

  .log-ok {
    background: rgba(16, 185, 129, 0.08);
    color: var(--accent-green);
  }

  .log-err {
    background: rgba(239, 68, 68, 0.08);
    color: var(--accent-red);
  }

  .log-time {
    color: var(--text-muted);
  }
</style>
