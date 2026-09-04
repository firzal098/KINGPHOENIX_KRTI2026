<script>
  import { 
    structuredAction, 
    rawAction, 
    policyActionHz, 
    maxActionMagnitude, 
    maxYawRateDeg, 
    setMaxActionMagnitude, 
    setMaxYawRateDeg 
  } from '../ros.js';

  function fmt(val, dec = 2) {
    if (val === undefined || isNaN(val)) return '0.00';
    return (val >= 0 ? '+' : '') + val.toFixed(dec);
  }

  // Helper to compute percentage for centered or asymmetric range
  function getPercent(val, min, max) {
    const clamped = Math.max(min, Math.min(max, val || 0));
    return ((clamped - min) / (max - min)) * 100;
  }

  function getBiPercent(val, maxAbs) {
    const clamped = Math.max(-maxAbs, Math.min(maxAbs, val || 0));
    // Center is 50%
    return 50 + (clamped / (2 * maxAbs)) * 100;
  }
</script>

<div class="gcs-card action-container">
  <div class="panel-header">
    <div class="header-title">
      <span class="icon-chip">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/>
        </svg>
      </span>
      <span class="font-hud">POLICY ACTION SPACE (3D)</span>
    </div>
    <span class="topic-tag font-mono">
      /policy/action
      {#if $policyActionHz > 0}
        <span class="hz-tag">{$policyActionHz} Hz</span>
      {/if}
    </span>
  </div>

  <div class="actions-grid">
    <!-- 1. Forward Velocity (v_fwd) -->
    <div class="action-card">
      <div class="act-header">
        <div class="act-title font-hud">
          <span class="act-dot cyan"></span>
          <span>FORWARD VELOCITY (v_fwd)</span>
        </div>
        <div class="act-value font-mono {($structuredAction?.vfwd || 0) >= 0 ? 'val-pos' : 'val-neg'}">
          {fmt($structuredAction?.vfwd, 2)} <small>m/s</small>
        </div>
      </div>

      <!-- Gauge Bar -->
      <div class="gauge-bar-wrapper">
        <div class="gauge-track">
          <!-- 0.0 position is at (4 / 20) * 100 = 20% -->
          <div class="zero-marker" style="left: 20%;"></div>
          {#if ($structuredAction?.vfwd || 0) >= 0}
            <div
              class="gauge-fill fill-cyan"
              style="left: 20%; width: {Math.min(80, (($structuredAction?.vfwd || 0) / 16.0) * 80)}%;"
            ></div>
          {:else}
            <div
              class="gauge-fill fill-amber"
              style="right: 80%; width: {Math.min(20, (Math.abs($structuredAction?.vfwd || 0) / 4.0) * 20)}%;"
            ></div>
          {/if}
        </div>
        <div class="gauge-labels font-mono">
          <span>-4.0 m/s</span>
          <span class="zero-lbl">0.0</span>
          <span>+16.0 m/s</span>
        </div>
      </div>
    </div>

    <!-- 2. Lateral Velocity (v_left) -->
    <div class="action-card">
      <div class="act-header">
        <div class="act-title font-hud">
          <span class="act-dot purple"></span>
          <span>LATERAL VELOCITY (v_left)</span>
        </div>
        <div class="act-value font-mono {Math.abs($structuredAction?.vleft || 0) < 0.05 ? 'val-zero' : ($structuredAction?.vleft || 0) > 0 ? 'val-pos' : 'val-neg'}">
          {fmt($structuredAction?.vleft, 2)} <small>m/s</small>
        </div>
      </div>

      <!-- Bi-directional Bar -->
      <div class="gauge-bar-wrapper">
        <div class="gauge-track">
          <div class="zero-marker" style="left: 50%;"></div>
          {#if ($structuredAction?.vleft || 0) >= 0}
            <div
              class="gauge-fill fill-purple"
              style="left: 50%; width: {Math.min(50, (($structuredAction?.vleft || 0) / 8.0) * 50)}%;"
            ></div>
          {:else}
            <div
              class="gauge-fill fill-purple"
              style="right: 50%; width: {Math.min(50, (Math.abs($structuredAction?.vleft || 0) / 8.0) * 50)}%;"
            ></div>
          {/if}
        </div>
        <div class="gauge-labels font-mono">
          <span>-8.0 (Right)</span>
          <span class="zero-lbl">0.0</span>
          <span>+8.0 (Left)</span>
        </div>
      </div>
    </div>

    <!-- 3. Yaw Rate -->
    <div class="action-card">
      <div class="act-header">
        <div class="act-title font-hud">
          <span class="act-dot amber"></span>
          <span>YAW ROTATION RATE (psi_dot)</span>
        </div>
        <div class="act-value font-mono {Math.abs($structuredAction?.yawRate || 0) < 0.05 ? 'val-zero' : ($structuredAction?.yawRate || 0) > 0 ? 'val-pos' : 'val-neg'}">
          {fmt($structuredAction?.yawRate, 2)} <small>rad/s</small>
          <span class="deg-sub font-mono">({fmt($structuredAction?.yawRateDeg, 0)}°/s)</span>
        </div>
      </div>

      <!-- Bi-directional Bar -->
      <div class="gauge-bar-wrapper">
        <div class="gauge-track">
          <div class="zero-marker" style="left: 50%;"></div>
          {#if ($structuredAction?.yawRate || 0) >= 0}
            <div
              class="gauge-fill fill-amber"
              style="left: 50%; width: {Math.min(50, (($structuredAction?.yawRate || 0) / 15.71) * 50)}%;"
            ></div>
          {:else}
            <div
              class="gauge-fill fill-amber"
              style="right: 50%; width: {Math.min(50, (Math.abs($structuredAction?.yawRate || 0) / 15.71) * 50)}%;"
            ></div>
          {/if}
        </div>
        <div class="gauge-labels font-mono">
          <span>-900°/s (CW)</span>
          <span class="zero-lbl">0.0</span>
          <span>+900°/s (CCW)</span>
        </div>
      </div>
    </div>
  </div>

  <!-- 4. Action Space Ceiling Sliders (RL Maximum Commanded Velocity & Yaw Rate) -->
  <div class="limits-section">
    <div class="limits-header">
      <div class="limits-title font-hud">
        <span class="limits-icon">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
            <path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/>
          </svg>
        </span>
        <span>ACTION SPACE MAXIMUM CEILING</span>
      </div>
      <span class="limits-hint font-mono">Real-time RL Constraints</span>
    </div>

    <div class="limits-grid">
      <!-- 1. Max Combined Horizontal Velocity (Forward + Lateral) -->
      <div class="limit-control-card">
        <div class="control-header">
          <div class="control-label font-hud">
            <span class="limit-dot cyan"></span>
            <span>MAX HORIZONTAL VELOCITY</span>
          </div>
          <div class="control-badge font-mono badge-cyan">
            {$maxActionMagnitude.toFixed(1)} <small>m/s</small>
          </div>
        </div>

        <div class="slider-wrapper">
          <input
            type="range"
            min="1.0"
            max="16.0"
            step="0.5"
            value={$maxActionMagnitude}
            on:input={(e) => setMaxActionMagnitude(e.target.value)}
            class="range-slider slider-cyan"
            aria-label="Maximum Combined Velocity Magnitude"
          />
          <div class="slider-ticks font-mono">
            <span>1.0 m/s</span>
            <span>4.0</span>
            <span>8.0</span>
            <span>12.0</span>
            <span>16.0 m/s</span>
          </div>
        </div>

        <!-- Quick Presets -->
        <div class="presets-row">
          {#each [4.0, 8.0, 12.0, 16.0] as preset}
            <button
              class="preset-btn font-mono {Math.abs($maxActionMagnitude - preset) < 0.2 ? 'active-cyan' : ''}"
              on:click={() => setMaxActionMagnitude(preset)}
            >
              {preset}m/s
            </button>
          {/each}
        </div>
      </div>

      <!-- 2. Max Yaw Rate -->
      <div class="limit-control-card">
        <div class="control-header">
          <div class="control-label font-hud">
            <span class="limit-dot amber"></span>
            <span>MAX YAW ROTATION RATE</span>
          </div>
          <div class="control-badge font-mono badge-amber">
            {$maxYawRateDeg}°/s <small class="rad-badge">({(($maxYawRateDeg * Math.PI) / 180).toFixed(2)} rad/s)</small>
          </div>
        </div>

        <div class="slider-wrapper">
          <input
            type="range"
            min="20"
            max="360"
            step="5"
            value={$maxYawRateDeg}
            on:input={(e) => setMaxYawRateDeg(e.target.value)}
            class="range-slider slider-amber"
            aria-label="Maximum Yaw Rotation Rate"
          />
          <div class="slider-ticks font-mono">
            <span>20°/s</span>
            <span>90°</span>
            <span>180°</span>
            <span>270°</span>
            <span>360°/s</span>
          </div>
        </div>

        <!-- Quick Presets -->
        <div class="presets-row">
          {#each [45, 90, 180, 360] as preset}
            <button
              class="preset-btn font-mono {Math.abs($maxYawRateDeg - preset) < 3 ? 'active-amber' : ''}"
              on:click={() => setMaxYawRateDeg(preset)}
            >
              {preset}°/s
            </button>
          {/each}
        </div>
      </div>
    </div>
  </div>
</div>

<style>
  .action-container {
    padding: 16px;
    display: flex;
    flex-direction: column;
    gap: 12px;
  }

  .panel-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding-bottom: 8px;
    border-bottom: 1px solid var(--border-subtle);
  }

  .header-title {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 0.95rem;
    font-weight: 700;
    color: var(--accent-cyan);
  }

  .icon-chip {
    color: var(--accent-amber);
  }

  .topic-tag {
    font-size: 0.68rem;
    color: var(--text-muted);
    background: rgba(255, 255, 255, 0.04);
    padding: 2px 6px;
    border-radius: 4px;
    display: inline-flex;
    align-items: center;
    gap: 5px;
  }

  .hz-tag {
    color: var(--accent-cyan, #00f0ff);
    font-weight: 700;
    background: rgba(0, 240, 255, 0.1);
    padding: 1px 4px;
    border-radius: 3px;
  }

  .actions-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 12px;
  }

  .action-card {
    background: rgba(0, 0, 0, 0.28);
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    padding: 12px;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .act-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .act-title {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.76rem;
    font-weight: 700;
    color: var(--text-secondary);
  }

  .act-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
  }
  .act-dot.cyan { background: var(--accent-cyan); box-shadow: 0 0 6px var(--accent-cyan); }
  .act-dot.purple { background: var(--accent-purple); box-shadow: 0 0 6px var(--accent-purple); }
  .act-dot.green { background: var(--accent-green); box-shadow: 0 0 6px var(--accent-green); }
  .act-dot.amber { background: var(--accent-amber); box-shadow: 0 0 6px var(--accent-amber); }

  .act-value {
    font-size: 0.95rem;
    font-weight: 700;
  }

  .act-value small {
    font-size: 0.7rem;
    color: var(--text-muted);
  }

  .deg-sub {
    font-size: 0.72rem;
    color: var(--text-muted);
    margin-left: 4px;
  }

  .val-pos { color: var(--accent-green); }
  .val-neg { color: var(--accent-amber); }
  .val-zero { color: var(--text-muted); }

  /* Gauge Bar */
  .gauge-bar-wrapper {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .gauge-track {
    position: relative;
    height: 8px;
    background: rgba(255, 255, 255, 0.06);
    border-radius: 4px;
    overflow: hidden;
    border: 1px solid rgba(255, 255, 255, 0.05);
  }

  .zero-marker {
    position: absolute;
    top: 0;
    bottom: 0;
    width: 2px;
    background: rgba(255, 255, 255, 0.3);
    z-index: 2;
  }

  .gauge-fill {
    position: absolute;
    top: 0;
    bottom: 0;
    border-radius: 3px;
    transition: width 0.1s ease-out, left 0.1s ease-out, right 0.1s ease-out;
  }

  .fill-cyan {
    background: linear-gradient(90deg, var(--accent-cyan), #38bdf8);
    box-shadow: 0 0 8px var(--accent-cyan-glow);
  }
  .fill-purple {
    background: linear-gradient(90deg, var(--accent-purple), #c084fc);
    box-shadow: 0 0 8px var(--accent-purple-glow);
  }
  .fill-green {
    background: linear-gradient(90deg, var(--accent-green), #34d399);
    box-shadow: 0 0 8px var(--accent-green-glow);
  }
  .fill-amber {
    background: linear-gradient(90deg, var(--accent-amber), #fbbf24);
    box-shadow: 0 0 8px var(--accent-amber-glow);
  }

  .gauge-labels {
    display: flex;
    justify-content: space-between;
    font-size: 0.62rem;
    color: var(--text-muted);
  }

  .zero-lbl {
    color: var(--text-secondary);
  }

  /* --- Action Space Limits Section --- */
  .limits-section {
    margin-top: 2px;
    padding: 10px 12px;
    background: rgba(10, 15, 24, 0.65);
    border: 1px solid rgba(0, 240, 255, 0.12);
    border-radius: 6px;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .limits-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding-bottom: 6px;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  }

  .limits-title {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.76rem;
    font-weight: 700;
    letter-spacing: 0.05em;
    color: var(--text-primary);
  }

  .limits-icon {
    color: var(--accent-cyan);
    display: flex;
    align-items: center;
  }

  .limits-hint {
    font-size: 0.62rem;
    color: var(--text-muted);
    letter-spacing: 0.02em;
  }

  .limits-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
  }

  @media (max-width: 900px) {
    .limits-grid {
      grid-template-columns: 1fr;
    }
  }

  .limit-control-card {
    background: rgba(0, 0, 0, 0.35);
    border: 1px solid rgba(255, 255, 255, 0.06);
    border-radius: 5px;
    padding: 8px 10px;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .control-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .control-label {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.70rem;
    font-weight: 600;
    color: var(--text-secondary);
  }

  .limit-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
  }

  .limit-dot.cyan {
    background: var(--accent-cyan);
    box-shadow: 0 0 6px var(--accent-cyan-glow);
  }

  .limit-dot.amber {
    background: var(--accent-amber);
    box-shadow: 0 0 6px var(--accent-amber-glow);
  }

  .control-badge {
    font-size: 0.78rem;
    font-weight: 700;
    padding: 2px 7px;
    border-radius: 4px;
    display: flex;
    align-items: baseline;
    gap: 3px;
  }

  .control-badge small {
    font-size: 0.62rem;
    font-weight: 500;
    opacity: 0.8;
  }

  .control-badge .rad-badge {
    color: var(--text-muted);
    font-size: 0.62rem;
    margin-left: 2px;
  }

  .badge-cyan {
    background: rgba(0, 240, 255, 0.12);
    color: var(--accent-cyan);
    border: 1px solid rgba(0, 240, 255, 0.3);
    box-shadow: 0 0 10px rgba(0, 240, 255, 0.15);
  }

  .badge-amber {
    background: rgba(245, 158, 11, 0.12);
    color: var(--accent-amber);
    border: 1px solid rgba(245, 158, 11, 0.3);
    box-shadow: 0 0 10px rgba(245, 158, 11, 0.15);
  }

  .slider-wrapper {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .range-slider {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 6px;
    border-radius: 3px;
    background: rgba(255, 255, 255, 0.08);
    outline: none;
    cursor: pointer;
    transition: background 0.2s;
  }

  .range-slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 16px;
    height: 16px;
    border-radius: 50%;
    border: 2px solid #000;
    cursor: pointer;
    transition: transform 0.15s ease, box-shadow 0.15s ease;
  }

  .range-slider:active::-webkit-slider-thumb {
    transform: scale(1.2);
  }

  .slider-cyan::-webkit-slider-thumb {
    background: var(--accent-cyan);
    box-shadow: 0 0 8px var(--accent-cyan-glow);
  }

  .slider-amber::-webkit-slider-thumb {
    background: var(--accent-amber);
    box-shadow: 0 0 8px var(--accent-amber-glow);
  }

  .slider-ticks {
    display: flex;
    justify-content: space-between;
    font-size: 0.58rem;
    color: var(--text-muted);
    padding: 0 2px;
  }

  .presets-row {
    display: flex;
    gap: 6px;
    margin-top: 2px;
  }

  .preset-btn {
    flex: 1;
    padding: 3px 0;
    font-size: 0.64rem;
    font-weight: 600;
    border-radius: 3px;
    border: 1px solid rgba(255, 255, 255, 0.08);
    background: rgba(255, 255, 255, 0.03);
    color: var(--text-secondary);
    cursor: pointer;
    transition: all 0.15s ease;
  }

  .preset-btn:hover {
    background: rgba(255, 255, 255, 0.08);
    color: var(--text-primary);
    border-color: rgba(255, 255, 255, 0.18);
  }

  .preset-btn.active-cyan {
    background: rgba(0, 240, 255, 0.2);
    border-color: var(--accent-cyan);
    color: #fff;
    box-shadow: 0 0 8px var(--accent-cyan-glow);
  }

  .preset-btn.active-amber {
    background: rgba(245, 158, 11, 0.2);
    border-color: var(--accent-amber);
    color: #fff;
    box-shadow: 0 0 8px var(--accent-amber-glow);
  }
</style>
