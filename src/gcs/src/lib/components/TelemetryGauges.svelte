<script>
  import { dronePose, droneVel } from '../ros.js';

  function fmt(val, dec = 2) {
    if (val === undefined || isNaN(val)) return '0.00';
    return (val >= 0 ? '+' : '') + val.toFixed(dec);
  }
</script>

<div class="gcs-card telemetry-card">
  <div class="card-header">
    <div class="header-title font-hud">
      <span class="dot-green"></span>
      <span>MAVROS STATE & ATTITUDE</span>
    </div>
    <span class="topic-tag font-mono">/mavros/local_position/*</span>
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

  .attitude-block, .position-block, .velocity-block {
    background: rgba(0, 0, 0, 0.25);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 8px 10px;
  }

  .att-row, .pos-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 6px;
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
</style>
