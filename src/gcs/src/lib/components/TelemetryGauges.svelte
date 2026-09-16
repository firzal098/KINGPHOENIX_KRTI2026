<script>
  import { dronePose, droneVel, gripperOpened, gripperPending, callSetGripperState, gate3UngripDelay, setGate3UngripDelay } from '../ros.js';

  function fmt(val, dec = 2) {
    if (val === undefined || isNaN(val)) return '0.00';
    return (val >= 0 ? '+' : '') + val.toFixed(dec);
  }
</script>

<div class="gcs-card telemetry-card">
  <div class="card-header">
    <div class="header-title font-hud">
      <span class="dot-green"></span>
      <span>MAVROS STATE & ACTUATORS</span>
    </div>
    <span class="topic-tag font-mono">/mavros/local_position/* & /gripper/*</span>
  </div>

  <div class="telemetry-content">
    <!-- 1. Attitude Readout -->
    <div class="attitude-block">
      <div class="block-title font-hud">ORIENTATION (DEG)</div>
      <div class="att-row">
        <div class="att-item">
          <span class="att-lbl">ROLL</span>
          <span class="att-val font-mono">{fmt($dronePose.roll, 1)}°</span>
        </div>
        <div class="att-item">
          <span class="att-lbl">PITCH</span>
          <span class="att-val font-mono">{fmt($dronePose.pitch, 1)}°</span>
        </div>
        <div class="att-item">
          <span class="att-lbl">YAW</span>
          <span class="att-val font-mono">{fmt($dronePose.yaw, 1)}°</span>
        </div>
      </div>
    </div>

    <!-- 2. Position Readout -->
    <div class="position-block">
      <div class="block-title font-hud">LOCAL POSITION (M)</div>
      <div class="pos-row font-mono">
        <div class="pos-item">
          <span class="pos-axis">X:</span>
          <span class="pos-val">{fmt($dronePose.x, 2)}</span>
        </div>
        <div class="pos-item">
          <span class="pos-axis">Y:</span>
          <span class="pos-val">{fmt($dronePose.y, 2)}</span>
        </div>
        <div class="pos-item highlight-alt">
          <span class="pos-axis">Z:</span>
          <span class="pos-val">{fmt($dronePose.z, 2)}</span>
        </div>
      </div>
    </div>

    <!-- 3. Local Velocity -->
    <div class="velocity-block">
      <div class="block-title font-hud">LOCAL VELOCITY (M/S)</div>
      <div class="pos-row font-mono">
        <div class="pos-item">
          <span class="pos-axis">Vx:</span>
          <span class="pos-val">{fmt($droneVel.vx, 2)}</span>
        </div>
        <div class="pos-item">
          <span class="pos-axis">Vy:</span>
          <span class="pos-val">{fmt($droneVel.vy, 2)}</span>
        </div>
        <div class="pos-item">
          <span class="pos-axis">Vz:</span>
          <span class="pos-val">{fmt($droneVel.vz, 2)}</span>
        </div>
      </div>
    </div>

    <!-- 4. Gripper Actuator -->
    <div class="gripper-block">
      <div class="block-title font-hud">GRIPPER (UDP 5504)</div>
      <div class="gripper-row">
        <div class="gripper-status-badge {$gripperOpened ? 'open' : 'closed'}">
          <span class="status-dot"></span>
          <span class="font-mono">{$gripperOpened ? 'OPEN' : 'CLOSED'}</span>
        </div>
        <div class="gripper-btn-group">
          <button 
            class="grp-btn {$gripperOpened ? 'active-open' : ''}" 
            on:click={() => callSetGripperState(true)}
            disabled={$gripperPending}
            title="Open Gripper"
          >
            OPEN
          </button>
          <button 
            class="grp-btn {!$gripperOpened ? 'active-closed' : ''}" 
            on:click={() => callSetGripperState(false)}
            disabled={$gripperPending}
            title="Close Gripper"
          >
            CLOSE
          </button>
        </div>
      </div>

      <!-- Gate 3.2 Auto-Ungrip Delay Control -->
      <div class="delay-row">
        <div class="delay-info">
          <span class="delay-lbl font-hud">GATE 3.2 UNGRIP DELAY</span>
          <span class="delay-num font-mono">{$gate3UngripDelay.toFixed(2)}s</span>
        </div>
        <div class="delay-input-group">
          <input
            type="range"
            min="0.0"
            max="5.0"
            step="0.1"
            value={$gate3UngripDelay}
            on:input={(e) => setGate3UngripDelay(e.target.value)}
            class="range-slider"
            title="Configure delay before ungripping after passing Gate 3.2"
          />
          <div class="delay-presets font-mono">
            {#each [0.0, 0.5, 1.0, 2.0] as preset}
              <button
                class="delay-btn {Math.abs($gate3UngripDelay - preset) < 0.05 ? 'active-preset' : ''}"
                on:click={() => setGate3UngripDelay(preset)}
              >
                {preset === 0.0 ? '0s' : `${preset}s`}
              </button>
            {/each}
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<style>
  .telemetry-card {
    padding: 14px;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .card-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding-bottom: 6px;
    border-bottom: 1px solid var(--border-subtle);
  }

  .header-title {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.8rem;
    font-weight: 700;
    color: var(--text-secondary);
  }

  .dot-green {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: var(--accent-green);
    box-shadow: 0 0 6px var(--accent-green);
  }

  .topic-tag {
    font-size: 0.65rem;
    color: var(--text-muted);
  }

  .telemetry-content {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 10px;
  }

  .block-title {
    font-size: 0.68rem;
    color: var(--text-muted);
    font-weight: 600;
    margin-bottom: 6px;
  }

  .attitude-block, .position-block, .velocity-block, .gripper-block {
    background: rgba(0, 0, 0, 0.25);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 8px 10px;
  }

  .att-row, .pos-row, .gripper-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 6px;
  }

  .gripper-status-badge {
    display: inline-flex;
    align-items: center;
    gap: 5px;
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 0.75rem;
    font-weight: 700;
  }

  .gripper-status-badge.open {
    background: rgba(245, 158, 11, 0.15);
    color: #fbbf24;
    border: 1px solid rgba(245, 158, 11, 0.4);
    box-shadow: 0 0 8px rgba(245, 158, 11, 0.2);
  }
  .gripper-status-badge.open .status-dot {
    background: #fbbf24;
    box-shadow: 0 0 6px #fbbf24;
  }

  .gripper-status-badge.closed {
    background: rgba(0, 240, 255, 0.12);
    color: var(--accent-cyan);
    border: 1px solid rgba(0, 240, 255, 0.35);
  }
  .gripper-status-badge.closed .status-dot {
    background: var(--accent-cyan);
    box-shadow: 0 0 6px var(--accent-cyan);
  }

  .status-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
  }

  .gripper-btn-group {
    display: flex;
    gap: 4px;
  }

  .grp-btn {
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-secondary);
    border: 1px solid var(--border-subtle);
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 0.7rem;
    font-weight: 700;
    cursor: pointer;
    transition: all 0.15s ease;
  }

  .grp-btn:hover {
    background: rgba(255, 255, 255, 0.12);
    color: var(--text-primary);
  }

  .grp-btn.active-open {
    background: rgba(245, 158, 11, 0.25);
    color: #fbbf24;
    border-color: #fbbf24;
    box-shadow: 0 0 8px rgba(245, 158, 11, 0.35);
  }

  .grp-btn.active-closed {
    background: rgba(0, 240, 255, 0.22);
    color: var(--accent-cyan);
    border-color: var(--accent-cyan);
    box-shadow: 0 0 8px var(--accent-cyan-glow);
  }

  .grp-btn:disabled {
    opacity: 0.5;
    cursor: wait;
  }

  .att-item {
    display: flex;
    flex-direction: column;
  }

  .att-lbl {
    font-size: 0.6rem;
    color: var(--text-muted);
    font-weight: 600;
  }

  .att-val {
    font-size: 0.85rem;
    font-weight: 700;
    color: var(--accent-cyan);
  }

  .pos-item {
    display: flex;
    align-items: center;
    gap: 4px;
    font-size: 0.82rem;
  }

  .pos-axis {
    color: var(--text-muted);
    font-size: 0.7rem;
  }

  .pos-val {
    color: var(--text-primary);
    font-weight: 600;
  }

  .highlight-alt .pos-val {
    color: var(--accent-cyan);
    font-weight: 700;
  }

  .delay-row {
    margin-top: 8px;
    padding-top: 8px;
    border-top: 1px solid rgba(255, 255, 255, 0.07);
    display: flex;
    flex-direction: column;
    gap: 5px;
  }

  .delay-info {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .delay-lbl {
    font-size: 0.62rem;
    color: var(--text-muted);
    letter-spacing: 0.5px;
  }

  .delay-num {
    font-size: 0.75rem;
    font-weight: 700;
    color: var(--accent-cyan);
    background: rgba(0, 240, 255, 0.08);
    padding: 1px 5px;
    border-radius: 3px;
    border: 1px solid rgba(0, 240, 255, 0.2);
  }

  .delay-input-group {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .range-slider {
    flex: 1;
    height: 4px;
    appearance: none;
    background: rgba(255, 255, 255, 0.15);
    border-radius: 2px;
    outline: none;
    cursor: pointer;
  }

  .range-slider::-webkit-slider-thumb {
    appearance: none;
    width: 12px;
    height: 12px;
    border-radius: 50%;
    background: var(--accent-cyan);
    box-shadow: 0 0 6px var(--accent-cyan);
    cursor: pointer;
    transition: transform 0.1s ease;
  }

  .range-slider::-webkit-slider-thumb:hover {
    transform: scale(1.2);
  }

  .delay-presets {
    display: flex;
    gap: 3px;
  }

  .delay-btn {
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-secondary);
    border: 1px solid var(--border-subtle);
    padding: 2px 5px;
    border-radius: 3px;
    font-size: 0.62rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.15s ease;
  }

  .delay-btn:hover {
    background: rgba(255, 255, 255, 0.15);
    color: var(--text-primary);
  }

  .delay-btn.active-preset {
    background: rgba(0, 240, 255, 0.2);
    color: var(--accent-cyan);
    border-color: var(--accent-cyan);
    box-shadow: 0 0 6px var(--accent-cyan-glow);
  }
</style>

