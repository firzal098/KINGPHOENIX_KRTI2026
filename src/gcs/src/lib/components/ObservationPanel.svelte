<script>
  import { structuredObservation, rawObservation, targetGateLabel, targetGateIndex } from '../ros.js';

  let showRaw = false;

  function fmt(val) {
    if (val === undefined || isNaN(val)) return '0.000';
    return (val >= 0 ? '+' : '') + val.toFixed(3);
  }

  function getValClass(val) {
    if (Math.abs(val) < 0.005) return 'zero';
    return val > 0 ? 'pos' : 'neg';
  }
</script>

<div class="gcs-card obs-container">
  <!-- Section Title -->
  <div class="panel-header">
    <div class="header-title">
      <span class="icon-chip">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <rect x="4" y="4" width="16" height="16" rx="2"/>
          <rect x="9" y="9" width="6" height="6"/>
          <line x1="9" y1="1" x2="9" y2="4"/>
          <line x1="15" y1="1" x2="15" y2="4"/>
          <line x1="9" y1="20" x2="9" y2="23"/>
          <line x1="15" y1="20" x2="15" y2="23"/>
        </svg>
      </span>
      <span class="font-hud">OBSERVATION SPACE (40D)</span>
    </div>

    <div class="header-tools">
      <button class="raw-toggle {showRaw ? 'active' : ''}" on:click={() => (showRaw = !showRaw)}>
        {showRaw ? 'GROUPED VIEW' : 'RAW 40D ARRAY'}
      </button>
    </div>
  </div>

  {#if showRaw}
    <!-- Raw Vector Table -->
    <div class="raw-array-view font-mono">
      {#each $rawObservation as val, i}
        <div class="raw-cell">
          <span class="idx">[{i < 10 ? '0' + i : i}]</span>
          <span class="val {getValClass(val)}">{fmt(val)}</span>
        </div>
      {/each}
    </div>
  {:else if $structuredObservation}
    <div class="obs-grid">
      <!-- 1. Body Velocity -->
      <div class="sub-card">
        <div class="sub-header font-hud">
          <span class="sub-dot cyan"></span>
          <span>1. BODY VELOCITY (v_B)</span>
          <span class="dim-tag font-mono">3D [0:3]</span>
        </div>
        <div class="coords-row">
          <div class="coord-item">
            <span class="coord-lbl">v_x (Fwd)</span>
            <span class="val-pill {getValClass($structuredObservation.vel_B.vx)}">
              {fmt($structuredObservation.vel_B.vx)} <small>m/s</small>
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">v_y (Left)</span>
            <span class="val-pill {getValClass($structuredObservation.vel_B.vy)}">
              {fmt($structuredObservation.vel_B.vy)} <small>m/s</small>
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">v_z (Up)</span>
            <span class="val-pill {getValClass($structuredObservation.vel_B.vz)}">
              {fmt($structuredObservation.vel_B.vz)} <small>m/s</small>
            </span>
          </div>
        </div>
      </div>

      <!-- 2. Projected Gravity -->
      <div class="sub-card">
        <div class="sub-header font-hud">
          <span class="sub-dot purple"></span>
          <span>2. PROJECTED GRAVITY (g_B)</span>
          <span class="dim-tag font-mono">3D [3:6]</span>
        </div>
        <div class="coords-row">
          <div class="coord-item">
            <span class="coord-lbl">g_x (r20)</span>
            <span class="val-pill {getValClass($structuredObservation.grav_B.gx)}">
              {fmt($structuredObservation.grav_B.gx)}
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">g_y (r21)</span>
            <span class="val-pill {getValClass($structuredObservation.grav_B.gy)}">
              {fmt($structuredObservation.grav_B.gy)}
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">g_z (r22)</span>
            <span class="val-pill {getValClass($structuredObservation.grav_B.gz)}">
              {fmt($structuredObservation.grav_B.gz)}
            </span>
          </div>
        </div>
      </div>

      <!-- 3. Active Gate 15D Features -->
      <div class="sub-card gate-card active-gate-border">
        <div class="sub-header font-hud">
          <div class="gate-header-left">
            <span class="sub-dot green"></span>
            <span>3. ACTIVE TARGET: {$targetGateLabel}</span>
            <span class="dim-tag font-mono">15D [6:21]</span>
          </div>
          <span class="dist-badge font-mono">
            Dist: {$structuredObservation.active_gate.dist.toFixed(2)}m
          </span>
        </div>

        <div class="gate-points-list font-mono">
          <!-- Gate Center -->
          <div class="gate-point-row highlight-center">
            <span class="point-name font-hud">GATE CENTER [18:21]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.center[0])}">{fmt($structuredObservation.active_gate.center[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.center[1])}">{fmt($structuredObservation.active_gate.center[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.center[2])}">{fmt($structuredObservation.active_gate.center[2])}</span>
            </div>
          </div>

          <!-- Top-Left Corner -->
          <div class="gate-point-row">
            <span class="point-name">Top-Left Corner [6:9]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.tl[0])}">{fmt($structuredObservation.active_gate.tl[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.tl[1])}">{fmt($structuredObservation.active_gate.tl[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.tl[2])}">{fmt($structuredObservation.active_gate.tl[2])}</span>
            </div>
          </div>

          <!-- Top-Right Corner -->
          <div class="gate-point-row">
            <span class="point-name">Top-Right Corner [9:12]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.tr[0])}">{fmt($structuredObservation.active_gate.tr[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.tr[1])}">{fmt($structuredObservation.active_gate.tr[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.tr[2])}">{fmt($structuredObservation.active_gate.tr[2])}</span>
            </div>
          </div>

          <!-- Bottom-Left Corner -->
          <div class="gate-point-row">
            <span class="point-name">Bottom-Left Corner [12:15]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.bl[0])}">{fmt($structuredObservation.active_gate.bl[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.bl[1])}">{fmt($structuredObservation.active_gate.bl[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.bl[2])}">{fmt($structuredObservation.active_gate.bl[2])}</span>
            </div>
          </div>

          <!-- Bottom-Right Corner -->
          <div class="gate-point-row">
            <span class="point-name">Bottom-Right Corner [15:18]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.br[0])}">{fmt($structuredObservation.active_gate.br[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.br[1])}">{fmt($structuredObservation.active_gate.br[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.br[2])}">{fmt($structuredObservation.active_gate.br[2])}</span>
            </div>
          </div>
        </div>
      </div>

      <!-- 4. Next Gate Preview 15D Features -->
      <div class="sub-card gate-card">
        <div class="sub-header font-hud">
          <div class="gate-header-left">
            <span class="sub-dot amber"></span>
            <span>4. NEXT PREVIEW: {$targetGateIndex < 4 ? `GATE #${$targetGateIndex + 2}` : 'NONE (FINAL GATE)'}</span>
            <span class="dim-tag font-mono">15D [21:36]</span>
          </div>
          {#if $structuredObservation.next_gate.has_next}
            <span class="badge badge-green">IN VIEW</span>
          {:else}
            <span class="badge badge-amber">FINAL GATE / NONE</span>
          {/if}
        </div>

        <div class="gate-points-list font-mono {$structuredObservation.next_gate.has_next ? '' : 'faded'}">
          <!-- Next Center -->
          <div class="gate-point-row highlight-center">
            <span class="point-name font-hud">NEXT CENTER [33:36]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.center[0])}">{fmt($structuredObservation.next_gate.center[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.center[1])}">{fmt($structuredObservation.next_gate.center[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.center[2])}">{fmt($structuredObservation.next_gate.center[2])}</span>
            </div>
          </div>

          <!-- Next TL -->
          <div class="gate-point-row">
            <span class="point-name">Next Top-Left [21:24]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.tl[0])}">{fmt($structuredObservation.next_gate.tl[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.tl[1])}">{fmt($structuredObservation.next_gate.tl[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.tl[2])}">{fmt($structuredObservation.next_gate.tl[2])}</span>
            </div>
          </div>

          <!-- Next TR -->
          <div class="gate-point-row">
            <span class="point-name">Next Top-Right [24:27]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.tr[0])}">{fmt($structuredObservation.next_gate.tr[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.tr[1])}">{fmt($structuredObservation.next_gate.tr[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.tr[2])}">{fmt($structuredObservation.next_gate.tr[2])}</span>
            </div>
          </div>

          <!-- Next BL -->
          <div class="gate-point-row">
            <span class="point-name">Next Bottom-Left [27:30]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.bl[0])}">{fmt($structuredObservation.next_gate.bl[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.bl[1])}">{fmt($structuredObservation.next_gate.bl[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.bl[2])}">{fmt($structuredObservation.next_gate.bl[2])}</span>
            </div>
          </div>

          <!-- Next BR -->
          <div class="gate-point-row">
            <span class="point-name">Next Bottom-Right [30:33]</span>
            <div class="xyz-box">
              <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.br[0])}">{fmt($structuredObservation.next_gate.br[0])}</span>
              <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.br[1])}">{fmt($structuredObservation.next_gate.br[1])}</span>
              <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.br[2])}">{fmt($structuredObservation.next_gate.br[2])}</span>
            </div>
          </div>
        </div>
      </div>

      <!-- 5. Previous Action -->
      <div class="sub-card prev-act-card">
        <div class="sub-header font-hud">
          <span class="sub-dot blue"></span>
          <span>5. LATCHED PREVIOUS ACTION (a_prev)</span>
          <span class="dim-tag font-mono">4D [36:40]</span>
        </div>
        <div class="coords-row">
          <div class="coord-item">
            <span class="coord-lbl">v_fwd</span>
            <span class="val-pill {getValClass($structuredObservation.prev_action.vfwd)}">
              {fmt($structuredObservation.prev_action.vfwd)} <small>m/s</small>
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">v_left</span>
            <span class="val-pill {getValClass($structuredObservation.prev_action.vleft)}">
              {fmt($structuredObservation.prev_action.vleft)} <small>m/s</small>
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">v_up</span>
            <span class="val-pill {getValClass($structuredObservation.prev_action.vup)}">
              {fmt($structuredObservation.prev_action.vup)} <small>m/s</small>
            </span>
          </div>
          <div class="coord-item">
            <span class="coord-lbl">yaw_rate</span>
            <span class="val-pill {getValClass($structuredObservation.prev_action.yawRate)}">
              {fmt($structuredObservation.prev_action.yawRate)} <small>rad/s</small>
            </span>
          </div>
        </div>
      </div>
    </div>
  {/if}
</div>

<style>
  .obs-container {
    padding: 16px;
    display: flex;
    flex-direction: column;
    gap: 14px;
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
    color: var(--accent-cyan);
  }

  .raw-toggle {
    background: rgba(255, 255, 255, 0.06);
    border: 1px solid var(--border-subtle);
    color: var(--text-secondary);
    font-family: var(--font-hud);
    font-size: 0.72rem;
    padding: 4px 10px;
    border-radius: 4px;
    cursor: pointer;
    transition: all 0.15s ease;
  }
  .raw-toggle:hover, .raw-toggle.active {
    background: rgba(0, 240, 255, 0.15);
    border-color: var(--accent-cyan);
    color: var(--accent-cyan);
  }

  .obs-grid {
    display: flex;
    flex-direction: column;
    gap: 12px;
  }

  .sub-card {
    background: rgba(0, 0, 0, 0.28);
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    padding: 12px;
  }

  .active-gate-border {
    border-color: rgba(16, 185, 129, 0.3);
    background: rgba(16, 185, 129, 0.03);
  }

  .sub-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    font-size: 0.8rem;
    font-weight: 700;
    color: var(--text-secondary);
    margin-bottom: 10px;
  }

  .gate-header-left {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .sub-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
  }
  .sub-dot.cyan { background: var(--accent-cyan); box-shadow: 0 0 6px var(--accent-cyan); }
  .sub-dot.purple { background: var(--accent-purple); box-shadow: 0 0 6px var(--accent-purple); }
  .sub-dot.green { background: var(--accent-green); box-shadow: 0 0 6px var(--accent-green); }
  .sub-dot.amber { background: var(--accent-amber); box-shadow: 0 0 6px var(--accent-amber); }
  .sub-dot.blue { background: var(--accent-blue); box-shadow: 0 0 6px var(--accent-blue); }

  .dim-tag {
    font-size: 0.65rem;
    color: var(--text-muted);
    background: rgba(255, 255, 255, 0.05);
    padding: 1px 6px;
    border-radius: 4px;
  }

  .dist-badge {
    font-size: 0.72rem;
    color: var(--accent-green);
    background: rgba(16, 185, 129, 0.15);
    padding: 2px 8px;
    border-radius: 4px;
    border: 1px solid rgba(16, 185, 129, 0.3);
    font-weight: 700;
  }

  .coords-row {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
    gap: 8px;
  }

  .coord-item {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .coord-lbl {
    font-size: 0.65rem;
    color: var(--text-muted);
    font-weight: 600;
  }

  /* Gate Points List */
  .gate-points-list {
    display: flex;
    flex-direction: column;
    gap: 6px;
  }

  .gate-points-list.faded {
    opacity: 0.45;
  }

  .gate-point-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: rgba(0, 0, 0, 0.3);
    padding: 6px 10px;
    border-radius: 6px;
    border: 1px solid rgba(255, 255, 255, 0.03);
    font-size: 0.75rem;
  }

  .highlight-center {
    background: rgba(0, 240, 255, 0.08);
    border-color: rgba(0, 240, 255, 0.2);
  }

  .highlight-center .point-name {
    color: var(--accent-cyan);
    font-weight: 700;
  }

  .point-name {
    color: var(--text-secondary);
    font-size: 0.74rem;
  }

  .xyz-box {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .axis {
    color: var(--text-muted);
    font-size: 0.65rem;
  }

  .num {
    font-weight: 600;
    min-width: 54px;
    text-align: right;
  }
  .num.pos { color: var(--accent-green); }
  .num.neg { color: var(--accent-amber); }
  .num.zero { color: var(--text-muted); }

  /* Raw 40D Array */
  .raw-array-view {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(110px, 1fr));
    gap: 6px;
    background: rgba(0, 0, 0, 0.4);
    padding: 10px;
    border-radius: 8px;
    max-height: 400px;
    overflow-y: auto;
  }

  .raw-cell {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: rgba(255, 255, 255, 0.03);
    padding: 3px 6px;
    border-radius: 4px;
    font-size: 0.72rem;
  }

  .idx {
    color: var(--text-muted);
    font-size: 0.65rem;
  }
</style>
