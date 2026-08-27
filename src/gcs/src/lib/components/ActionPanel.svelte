<script>
  import { structuredAction, rawAction } from '../ros.js';

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
      <span class="font-hud">POLICY ACTION SPACE (4D)</span>
    </div>
    <span class="topic-tag font-mono">/policy/action</span>
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
          <div
            class="gauge-fill fill-cyan"
            style="left: 20%; width: {Math.max(0, (($structuredAction?.vfwd || 0) / 16.0) * 80)}%;"
          ></div>
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

    <!-- 3. Vertical Velocity (v_up) -->
    <div class="action-card">
      <div class="act-header">
        <div class="act-title font-hud">
          <span class="act-dot green"></span>
          <span>VERTICAL VELOCITY (v_up)</span>
        </div>
        <div class="act-value font-mono {Math.abs($structuredAction?.vup || 0) < 0.05 ? 'val-zero' : ($structuredAction?.vup || 0) > 0 ? 'val-pos' : 'val-neg'}">
          {fmt($structuredAction?.vup, 2)} <small>m/s</small>
        </div>
      </div>

      <!-- Bi-directional Bar -->
      <div class="gauge-bar-wrapper">
        <div class="gauge-track">
          <div class="zero-marker" style="left: 50%;"></div>
          {#if ($structuredAction?.vup || 0) >= 0}
            <div
              class="gauge-fill fill-green"
              style="left: 50%; width: {Math.min(50, (($structuredAction?.vup || 0) / 6.0) * 50)}%;"
            ></div>
          {:else}
            <div
              class="gauge-fill fill-amber"
              style="right: 50%; width: {Math.min(50, (Math.abs($structuredAction?.vup || 0) / 6.0) * 50)}%;"
            ></div>
          {/if}
        </div>
        <div class="gauge-labels font-mono">
          <span>-6.0 (Sink)</span>
          <span class="zero-lbl">0.0</span>
          <span>+6.0 (Climb)</span>
        </div>
      </div>
    </div>

    <!-- 4. Yaw Rate -->
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
              style="left: 50%; width: {Math.min(50, (($structuredAction?.yawRate || 0) / 9.42) * 50)}%;"
            ></div>
          {:else}
            <div
              class="gauge-fill fill-amber"
              style="right: 50%; width: {Math.min(50, (Math.abs($structuredAction?.yawRate || 0) / 9.42) * 50)}%;"
            ></div>
          {/if}
        </div>
        <div class="gauge-labels font-mono">
          <span>-540°/s (CW)</span>
          <span class="zero-lbl">0.0</span>
          <span>+540°/s (CCW)</span>
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
</style>
