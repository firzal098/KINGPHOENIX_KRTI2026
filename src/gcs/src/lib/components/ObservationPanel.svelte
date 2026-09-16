<script>
  import { onMount } from 'svelte';
  import {
    structuredObservation,
    rawObservation,
    targetGateLabel,
    targetGateIndex,
    previewGateLabel,
    setTargetGate,
    debugImage,
    debugImageSrc,
    isPerceptionStreamActive,
    togglePerceptionStream
  } from '../ros.js';

  let activeTab = 'obs'; // 'obs' | 'fpv'
  let showRaw = false;

  // FPV Stream variables
  let canvasEl;
  let ctx;
  let showCrosshair = true;
  let fitMode = 'contain'; // 'contain' | 'cover'
  let isFullscreen = false;
  let containerEl;

  onMount(() => {
    if (canvasEl) {
      ctx = canvasEl.getContext('2d', { willReadFrequently: false });
    }
  });

  // Fallback canvas renderer if raw bytes are streamed instead of JPEG
  $: if (
    activeTab === 'fpv' &&
    $isPerceptionStreamActive &&
    !$debugImageSrc &&
    canvasEl &&
    $debugImage.data &&
    $debugImage.width > 0 &&
    $debugImage.height > 0
  ) {
    renderRawImage($debugImage);
  }

  function renderRawImage(img) {
    if (!canvasEl) return;
    if (!ctx) ctx = canvasEl.getContext('2d');

    const width = img.width;
    const height = img.height;

    if (canvasEl.width !== width || canvasEl.height !== height) {
      canvasEl.width = width;
      canvasEl.height = height;
    }

    try {
      let rawBytes;
      if (typeof img.data === 'string') {
        const binaryStr = atob(img.data);
        rawBytes = new Uint8Array(binaryStr.length);
        for (let i = 0; i < binaryStr.length; i++) {
          rawBytes[i] = binaryStr.charCodeAt(i);
        }
      } else if (img.data instanceof Uint8Array) {
        rawBytes = img.data;
      } else if (Array.isArray(img.data)) {
        rawBytes = new Uint8Array(img.data);
      }

      if (!rawBytes || rawBytes.length === 0) return;

      const imgData = ctx.createImageData(width, height);
      const data32 = new Uint32Array(imgData.data.buffer);
      const isBgr = img.encoding.includes('bgr');

      let srcIdx = 0;
      const numPixels = width * height;

      for (let i = 0; i < numPixels; i++) {
        let r, g, b;
        if (isBgr) {
          b = rawBytes[srcIdx];
          g = rawBytes[srcIdx + 1];
          r = rawBytes[srcIdx + 2];
        } else {
          r = rawBytes[srcIdx];
          g = rawBytes[srcIdx + 1];
          b = rawBytes[srcIdx + 2];
        }
        srcIdx += 3;
        data32[i] = (255 << 24) | (b << 16) | (g << 8) | r;
      }

      ctx.putImageData(imgData, 0, 0);
    } catch (err) {
      console.error('Failed to render ROS Image on canvas:', err);
    }
  }

  function toggleFullscreen() {
    if (!containerEl) return;
    if (!document.fullscreenElement) {
      containerEl.requestFullscreen().catch((err) => console.error(err));
      isFullscreen = true;
    } else {
      document.exitFullscreen();
      isFullscreen = false;
    }
  }

  function fmt(val) {
    if (val === undefined || isNaN(val)) return '0.000';
    return (val >= 0 ? '+' : '') + val.toFixed(3);
  }

  function getValClass(val) {
    if (Math.abs(val) < 0.005) return 'zero';
    return val > 0 ? 'pos' : 'neg';
  }
</script>

<div class="gcs-card obs-container {activeTab === 'fpv' ? 'fpv-mode' : ''}" bind:this={containerEl}>
  <!-- Section Title & Tab Switcher -->
  <div class="panel-header">
    <div class="header-left">
      <div class="tab-switcher font-hud">
        <!-- Observation Space Tab Button -->
        <button
          class="tab-btn {activeTab === 'obs' ? 'active' : ''}"
          on:click={() => (activeTab = 'obs')}
          title="Switch to 42D Observation Space View"
        >
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <rect x="4" y="4" width="16" height="16" rx="2"/>
            <rect x="9" y="9" width="6" height="6"/>
            <line x1="9" y1="1" x2="9" y2="4"/>
            <line x1="15" y1="1" x2="15" y2="4"/>
            <line x1="9" y1="20" x2="9" y2="23"/>
            <line x1="15" y1="20" x2="15" y2="23"/>
          </svg>
          <span>OBSERVATION SPACE (42D)</span>
        </button>

        <!-- FPV Camera Tab Button -->
        <button
          class="tab-btn {activeTab === 'fpv' ? 'active' : ''}"
          on:click={() => (activeTab = 'fpv')}
          title="Switch to FPV Camera Stream"
        >
          <span class="tab-dot {$isPerceptionStreamActive && ($debugImageSrc || $debugImage.data) ? 'live' : ''}"></span>
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/>
            <circle cx="12" cy="13" r="4"/>
          </svg>
          <span>FPV CAMERA</span>
        </button>
      </div>
    </div>

    <div class="header-tools">
      {#if activeTab === 'obs'}
        <button class="raw-toggle {showRaw ? 'active' : ''}" on:click={() => (showRaw = !showRaw)}>
          {showRaw ? 'GROUPED VIEW' : 'RAW 42D ARRAY'}
        </button>
      {:else if activeTab === 'fpv'}
        {#if $isPerceptionStreamActive}
          {#if $debugImageSrc || $debugImage.data}
            <span class="info-pill font-mono">
              {$debugImage.fps > 0 ? `${$debugImage.fps} FPS` : 'STREAMING'} &bull; JPEG
            </span>
          {:else}
            <span class="info-pill font-mono waiting">AWAITING STREAM</span>
          {/if}
        {:else}
          <span class="info-pill font-mono paused">STREAM OFF</span>
        {/if}

        <!-- Visibility Toggle Eye Button -->
        <button
          class="ctrl-btn {$isPerceptionStreamActive ? 'active' : 'btn-eye-off'}"
          on:click={togglePerceptionStream}
          title={$isPerceptionStreamActive ? 'Hide & Unsubscribe Stream' : 'Show & Subscribe Stream'}
        >
          {#if $isPerceptionStreamActive}
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/>
              <circle cx="12" cy="12" r="3"/>
            </svg>
          {:else}
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"/>
              <line x1="1" y1="1" x2="23" y2="23"/>
            </svg>
          {/if}
        </button>

        {#if $isPerceptionStreamActive}
          <button
            class="ctrl-btn {showCrosshair ? 'active' : ''}"
            on:click={() => (showCrosshair = !showCrosshair)}
            title="Toggle FPV Crosshairs"
          >
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <circle cx="12" cy="12" r="10"/>
              <line x1="22" y1="12" x2="18" y2="12"/>
              <line x1="6" y1="12" x2="2" y2="12"/>
              <line x1="12" y1="6" x2="12" y2="2"/>
              <line x1="12" y1="22" x2="12" y2="18"/>
            </svg>
          </button>

          <button
            class="ctrl-btn {fitMode === 'cover' ? 'active' : ''}"
            on:click={() => (fitMode = fitMode === 'contain' ? 'cover' : 'contain')}
            title="Toggle Aspect Fit / Fill"
          >
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <rect x="3" y="3" width="18" height="18" rx="2"/>
              <path d="M9 3v18"/>
            </svg>
          </button>
        {/if}

        <button class="ctrl-btn" on:click={toggleFullscreen} title="Toggle Fullscreen">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <polyline points="15 3 21 3 21 9"/>
            <polyline points="9 21 3 21 3 15"/>
            <line x1="21" y1="3" x2="14" y2="10"/>
            <line x1="3" y1="21" x2="10" y2="14"/>
          </svg>
        </button>
      {/if}
    </div>
  </div>

  <!-- Tab Contents -->
  {#if activeTab === 'obs'}
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

        <!-- 3. Body Angular Velocity (omega_B) -->
        <div class="sub-card">
          <div class="sub-header font-hud">
            <span class="sub-dot amber"></span>
            <span>3. BODY ANGULAR VELOCITY (omega_B)</span>
            <span class="dim-tag font-mono">3D [6:9]</span>
          </div>
          <div class="coords-row">
            <div class="coord-item">
              <span class="coord-lbl">w_x (Roll Rate / p)</span>
              <span class="val-pill {getValClass($structuredObservation.omega_B?.wx)}">
                {fmt($structuredObservation.omega_B?.wx)} <small>rad/s</small>
              </span>
            </div>
            <div class="coord-item">
              <span class="coord-lbl">w_y (Pitch Rate / q)</span>
              <span class="val-pill {getValClass($structuredObservation.omega_B?.wy)}">
                {fmt($structuredObservation.omega_B?.wy)} <small>rad/s</small>
              </span>
            </div>
            <div class="coord-item">
              <span class="coord-lbl">w_z (Yaw Rate / r)</span>
              <span class="val-pill {getValClass($structuredObservation.omega_B?.wz)}">
                {fmt($structuredObservation.omega_B?.wz)} <small>rad/s</small>
              </span>
            </div>
          </div>
        </div>

        <!-- 4. Active Gate 15D Features -->
        <div class="sub-card gate-card active-gate-border">
          <div class="sub-header font-hud">
            <div class="gate-header-left">
              <span class="sub-dot green"></span>
              <span>4. ACTIVE: {$targetGateLabel}</span>
              <span class="dim-tag font-mono">15D [9:24]</span>
            </div>

            <div class="gate-quick-switcher font-hud">
              {#each [0, 1, 2, 3, 4] as gIdx}
                <button
                  class="gate-btn { $targetGateIndex === gIdx ? 'active' : '' }"
                  on:click={() => setTargetGate(gIdx)}
                  title="Target Gate #{gIdx + 1}"
                >
                  G{gIdx + 1}
                </button>
              {/each}
            </div>

            <span class="dist-badge font-mono">
              Dist: {$structuredObservation.active_gate.dist.toFixed(2)}m
            </span>
          </div>

          <div class="gate-points-list font-mono">
            <!-- Gate Center -->
            <div class="gate-point-row highlight-center">
              <span class="point-name font-hud">GATE CENTER [21:24]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.center[0])}">{fmt($structuredObservation.active_gate.center[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.center[1])}">{fmt($structuredObservation.active_gate.center[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.center[2])}">{fmt($structuredObservation.active_gate.center[2])}</span>
              </div>
            </div>

            <!-- Top-Left Corner -->
            <div class="gate-point-row">
              <span class="point-name">Top-Left Corner [9:12]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.tl[0])}">{fmt($structuredObservation.active_gate.tl[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.tl[1])}">{fmt($structuredObservation.active_gate.tl[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.tl[2])}">{fmt($structuredObservation.active_gate.tl[2])}</span>
              </div>
            </div>

            <!-- Top-Right Corner -->
            <div class="gate-point-row">
              <span class="point-name">Top-Right Corner [12:15]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.tr[0])}">{fmt($structuredObservation.active_gate.tr[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.tr[1])}">{fmt($structuredObservation.active_gate.tr[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.tr[2])}">{fmt($structuredObservation.active_gate.tr[2])}</span>
              </div>
            </div>

            <!-- Bottom-Left Corner -->
            <div class="gate-point-row">
              <span class="point-name">Bottom-Left Corner [15:18]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.bl[0])}">{fmt($structuredObservation.active_gate.bl[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.bl[1])}">{fmt($structuredObservation.active_gate.bl[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.bl[2])}">{fmt($structuredObservation.active_gate.bl[2])}</span>
              </div>
            </div>

            <!-- Bottom-Right Corner -->
            <div class="gate-point-row">
              <span class="point-name">Bottom-Right Corner [18:21]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.active_gate.br[0])}">{fmt($structuredObservation.active_gate.br[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.active_gate.br[1])}">{fmt($structuredObservation.active_gate.br[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.active_gate.br[2])}">{fmt($structuredObservation.active_gate.br[2])}</span>
              </div>
            </div>
          </div>
        </div>

        <!-- 5. Next Gate Preview 15D Features -->
        <div class="sub-card gate-card">
          <div class="sub-header font-hud">
            <div class="gate-header-left">
              <span class="sub-dot amber"></span>
              <span>5. NEXT PREVIEW: {$previewGateLabel}</span>
              <span class="dim-tag font-mono">15D [24:39]</span>
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
              <span class="point-name font-hud">NEXT CENTER [36:39]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.center[0])}">{fmt($structuredObservation.next_gate.center[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.center[1])}">{fmt($structuredObservation.next_gate.center[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.center[2])}">{fmt($structuredObservation.next_gate.center[2])}</span>
              </div>
            </div>

            <!-- Next TL -->
            <div class="gate-point-row">
              <span class="point-name">Next Top-Left [24:27]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.tl[0])}">{fmt($structuredObservation.next_gate.tl[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.tl[1])}">{fmt($structuredObservation.next_gate.tl[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.tl[2])}">{fmt($structuredObservation.next_gate.tl[2])}</span>
              </div>
            </div>

            <!-- Next TR -->
            <div class="gate-point-row">
              <span class="point-name">Next Top-Right [27:30]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.tr[0])}">{fmt($structuredObservation.next_gate.tr[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.tr[1])}">{fmt($structuredObservation.next_gate.tr[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.tr[2])}">{fmt($structuredObservation.next_gate.tr[2])}</span>
              </div>
            </div>

            <!-- Next BL -->
            <div class="gate-point-row">
              <span class="point-name">Next Bottom-Left [30:33]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.bl[0])}">{fmt($structuredObservation.next_gate.bl[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.bl[1])}">{fmt($structuredObservation.next_gate.bl[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.bl[2])}">{fmt($structuredObservation.next_gate.bl[2])}</span>
              </div>
            </div>

            <!-- Next BR -->
            <div class="gate-point-row">
              <span class="point-name">Next Bottom-Right [33:36]</span>
              <div class="xyz-box">
                <span class="axis">X:</span> <span class="num {getValClass($structuredObservation.next_gate.br[0])}">{fmt($structuredObservation.next_gate.br[0])}</span>
                <span class="axis">Y:</span> <span class="num {getValClass($structuredObservation.next_gate.br[1])}">{fmt($structuredObservation.next_gate.br[1])}</span>
                <span class="axis">Z:</span> <span class="num {getValClass($structuredObservation.next_gate.br[2])}">{fmt($structuredObservation.next_gate.br[2])}</span>
              </div>
            </div>
          </div>
        </div>

        <!-- 6. Previous Action -->
        <div class="sub-card prev-act-card">
          <div class="sub-header font-hud">
            <span class="sub-dot blue"></span>
            <span>6. LATCHED PREVIOUS ACTION (a_prev)</span>
            <span class="dim-tag font-mono">3D [39:42]</span>
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
              <span class="coord-lbl">yaw_rate</span>
              <span class="val-pill {getValClass($structuredObservation.prev_action.yawRate)}">
                {fmt($structuredObservation.prev_action.yawRate)} <small>rad/s</small>
              </span>
            </div>
          </div>
        </div>
      </div>
    {/if}
  {:else if activeTab === 'fpv'}
    <!-- FPV Video Viewport -->
    {#if $isPerceptionStreamActive}
      <div class="viewport-wrapper">
        {#if $debugImageSrc}
          <img
            src={$debugImageSrc}
            alt="Perception Stream"
            class="video-feed {fitMode}"
          />
        {:else}
          <canvas
            bind:this={canvasEl}
            class="video-canvas {fitMode}"
            style="display: {$debugImage.data ? 'block' : 'none'};"
          ></canvas>
        {/if}

        <!-- Radar Searching Animation when no stream is present -->
        {#if !$debugImageSrc && !$debugImage.data}
          <div class="placeholder-box">
            <div class="radar-scan">
              <div class="radar-circle"></div>
              <div class="radar-sweep"></div>
              <svg class="drone-wireframe" width="64" height="64" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
                <polygon points="12 2 2 7 12 12 22 7 12 2"/>
                <polyline points="2 17 12 22 22 17"/>
                <polyline points="2 12 12 17 22 12"/>
              </svg>
            </div>
            <div class="placeholder-text font-hud">CONNECTING TO PERCEPTION NODE</div>
            <div class="placeholder-sub font-mono">Subscribed: /perception/debug_image/compressed</div>
          </div>
        {/if}

        <!-- FPV Crosshair Overlay -->
        {#if showCrosshair && ($debugImageSrc || $debugImage.data)}
          <div class="crosshair-overlay">
            <div class="crosshair-center"></div>
            <div class="pitch-ladder">
              <div class="pitch-line p-pos">+10°</div>
              <div class="pitch-line p-zero">── 0° ──</div>
              <div class="pitch-line p-neg">-10°</div>
            </div>
            <div class="hud-tag font-hud">CAM: +25° TILT &bull; ONNX IPPE</div>
            <div class="hud-target font-hud">
              <span class="hud-target-dot"></span>
              <span>{$targetGateLabel}</span>
            </div>
          </div>
        {/if}
      </div>
    {:else}
      <div class="stream-off-box font-hud">
        <svg width="36" height="36" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
          <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"/>
          <line x1="1" y1="1" x2="23" y2="23"/>
        </svg>
        <span class="stream-off-title">FPV STREAM PAUSED</span>
        <span class="stream-off-sub font-mono">Camera subscription is turned off to save bandwidth</span>
        <button class="resume-stream-btn font-hud" on:click={togglePerceptionStream}>
          RESUME FPV STREAM
        </button>
      </div>
    {/if}
  {/if}
</div>

<style>
  .obs-container {
    padding: 14px;
    display: flex;
    flex-direction: column;
    gap: 12px;
    height: 100%;
    min-height: 480px;
  }

  .obs-container.fpv-mode {
    height: auto;
    min-height: auto;
  }

  .panel-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding-bottom: 8px;
    border-bottom: 1px solid var(--border-subtle);
    gap: 8px;
    flex-wrap: wrap;
  }

  .header-left {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  /* Tab Switcher */
  .tab-switcher {
    display: flex;
    align-items: center;
    background: rgba(7, 11, 18, 0.85);
    padding: 3px;
    border-radius: 6px;
    border: 1px solid rgba(0, 240, 255, 0.2);
    gap: 4px;
  }

  .tab-btn {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    background: transparent;
    border: none;
    color: var(--text-secondary, #94a3b8);
    font-size: 0.74rem;
    font-weight: 700;
    padding: 4px 10px;
    border-radius: 4px;
    cursor: pointer;
    letter-spacing: 0.04em;
    transition: all 0.2s ease;
  }

  .tab-btn:hover {
    color: var(--accent-cyan, #00f0ff);
    background: rgba(0, 240, 255, 0.08);
  }

  .tab-btn.active {
    color: #070b12;
    background: var(--accent-cyan, #00f0ff);
    font-weight: 800;
    box-shadow: 0 0 10px rgba(0, 240, 255, 0.35);
  }

  .tab-dot {
    width: 7px;
    height: 7px;
    border-radius: 50%;
    background: #64748b;
    transition: all 0.3s ease;
  }

  .tab-dot.live {
    background: #00f0ff;
    box-shadow: 0 0 8px #00f0ff;
    animation: blink 1.2s infinite alternate;
  }

  .tab-btn.active .tab-dot {
    background: #070b12;
    box-shadow: none;
  }

  @keyframes blink {
    from { opacity: 0.4; }
    to { opacity: 1; }
  }

  .header-tools {
    display: flex;
    align-items: center;
    gap: 6px;
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

  .info-pill {
    background: rgba(0, 240, 255, 0.1);
    color: var(--accent-cyan);
    border: 1px solid rgba(0, 240, 255, 0.25);
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 0.72rem;
    font-weight: 600;
  }

  .info-pill.waiting {
    background: rgba(245, 158, 11, 0.1);
    color: var(--accent-amber);
    border-color: rgba(245, 158, 11, 0.3);
  }

  .info-pill.paused {
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-muted);
    border-color: rgba(255, 255, 255, 0.1);
  }

  .ctrl-btn {
    background: rgba(255, 255, 255, 0.05);
    border: 1px solid var(--border-subtle);
    color: var(--text-secondary);
    padding: 4px 8px;
    border-radius: 4px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.15s ease;
  }

  .ctrl-btn:hover {
    color: var(--accent-cyan);
    border-color: var(--accent-cyan);
    background: rgba(0, 240, 255, 0.1);
  }

  .ctrl-btn.active {
    color: var(--accent-cyan);
    background: rgba(0, 240, 255, 0.2);
    border-color: var(--accent-cyan);
  }

  .btn-eye-off {
    color: var(--text-muted);
    border-color: rgba(255, 255, 255, 0.1);
  }
  .btn-eye-off:hover {
    color: var(--accent-amber);
    border-color: var(--accent-amber);
    background: rgba(245, 158, 11, 0.1);
  }

  /* FPV Viewport */
  .viewport-wrapper {
    position: relative;
    width: 100%;
    height: 420px;
    max-height: 420px;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #05070c;
    border-radius: 8px;
    border: 1px solid rgba(255, 255, 255, 0.06);
    overflow: hidden;
  }

  .video-feed, .video-canvas {
    width: 100%;
    height: 100%;
    max-height: 420px;
    display: block;
  }

  .video-feed.contain, .video-canvas.contain {
    object-fit: contain;
  }

  .video-feed.cover, .video-canvas.cover {
    object-fit: cover;
  }

  :fullscreen .obs-container.fpv-mode,
  :-webkit-full-screen .obs-container.fpv-mode {
    height: 100vh !important;
    max-height: 100vh !important;
    padding: 10px;
  }

  :fullscreen .viewport-wrapper,
  :-webkit-full-screen .viewport-wrapper {
    height: calc(100vh - 65px) !important;
    max-height: none !important;
  }

  :fullscreen .video-feed,
  :fullscreen .video-canvas,
  :-webkit-full-screen .video-feed,
  :-webkit-full-screen .video-canvas {
    max-height: none !important;
  }

  /* Placeholder */
  .placeholder-box {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 12px;
    color: var(--text-muted);
  }

  .radar-scan {
    position: relative;
    width: 100px;
    height: 100px;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .radar-circle {
    position: absolute;
    inset: 0;
    border-radius: 50%;
    border: 1px solid rgba(0, 240, 255, 0.2);
    box-shadow: inset 0 0 20px rgba(0, 240, 255, 0.05);
  }

  .radar-sweep {
    position: absolute;
    inset: 0;
    border-radius: 50%;
    border-top: 2px solid var(--accent-cyan);
    animation: rotate-sweep 3s linear infinite;
  }

  @keyframes rotate-sweep {
    from { transform: rotate(0deg); }
    to { transform: rotate(360deg); }
  }

  .drone-wireframe {
    color: rgba(0, 240, 255, 0.4);
    animation: float-pulse 2s ease-in-out infinite alternate;
  }

  @keyframes float-pulse {
    from { transform: translateY(-3px); }
    to { transform: translateY(3px); }
  }

  .placeholder-text {
    font-size: 0.95rem;
    font-weight: 700;
    color: var(--text-secondary);
    letter-spacing: 0.1em;
  }

  .placeholder-sub {
    font-size: 0.72rem;
    color: var(--text-muted);
  }

  /* Crosshair Overlay */
  .crosshair-overlay {
    position: absolute;
    inset: 0;
    pointer-events: none;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .crosshair-center {
    width: 24px;
    height: 24px;
    border: 1px solid rgba(0, 240, 255, 0.5);
    border-radius: 50%;
    position: relative;
  }

  .crosshair-center::before, .crosshair-center::after {
    content: '';
    position: absolute;
    background: rgba(0, 240, 255, 0.6);
  }

  .crosshair-center::before {
    left: 50%; top: -8px; bottom: -8px; width: 1px; transform: translateX(-50%);
  }

  .crosshair-center::after {
    top: 50%; left: -8px; right: -8px; height: 1px; transform: translateY(-50%);
  }

  .pitch-ladder {
    position: absolute;
    display: flex;
    flex-direction: column;
    gap: 30px;
    align-items: center;
    color: rgba(0, 240, 255, 0.4);
    font-family: var(--font-mono);
    font-size: 0.65rem;
    font-weight: 600;
  }

  .hud-tag {
    position: absolute;
    bottom: 12px;
    left: 14px;
    font-size: 0.68rem;
    color: var(--accent-cyan);
    background: rgba(0, 0, 0, 0.6);
    padding: 3px 8px;
    border-radius: 4px;
    border: 1px solid rgba(0, 240, 255, 0.2);
    letter-spacing: 0.06em;
  }

  .hud-target {
    position: absolute;
    top: 12px;
    right: 14px;
    display: inline-flex;
    align-items: center;
    gap: 6px;
    font-size: 0.72rem;
    font-weight: 800;
    color: #34d399;
    background: rgba(0, 0, 0, 0.7);
    padding: 4px 10px;
    border-radius: 4px;
    border: 1px solid rgba(16, 185, 129, 0.4);
    box-shadow: 0 0 10px rgba(16, 185, 129, 0.25);
    letter-spacing: 0.06em;
  }

  .hud-target-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: #34d399;
    box-shadow: 0 0 6px #34d399;
    animation: pulse 1.5s infinite;
  }

  @keyframes pulse {
    0%, 100% { opacity: 1; transform: scale(1); }
    50% { opacity: 0.4; transform: scale(0.85); }
  }

  .stream-off-box {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 10px;
    height: 420px;
    min-height: 420px;
    background: rgba(0, 0, 0, 0.3);
    border: 1px dashed rgba(255, 255, 255, 0.1);
    border-radius: 8px;
    color: var(--text-muted);
  }

  .stream-off-title {
    font-size: 0.95rem;
    font-weight: 700;
    color: #f1f5f9;
  }

  .stream-off-sub {
    font-size: 0.72rem;
    color: var(--text-muted);
  }

  .resume-stream-btn {
    margin-top: 8px;
    background: rgba(0, 240, 255, 0.15);
    border: 1px solid var(--accent-cyan);
    color: var(--accent-cyan);
    font-weight: 700;
    font-size: 0.75rem;
    padding: 6px 14px;
    border-radius: 4px;
    cursor: pointer;
    transition: all 0.2s ease;
  }
  .resume-stream-btn:hover {
    background: var(--accent-cyan);
    color: #070b12;
  }

  /* Observation Space Styles */
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

  .gate-quick-switcher {
    display: inline-flex;
    align-items: center;
    gap: 3px;
    background: rgba(0, 0, 0, 0.4);
    padding: 2px 4px;
    border-radius: 5px;
    border: 1px solid rgba(255, 255, 255, 0.08);
  }

  .gate-btn {
    background: transparent;
    border: 1px solid transparent;
    color: #94a3b8;
    font-size: 0.62rem;
    font-weight: 700;
    padding: 2px 6px;
    border-radius: 3px;
    cursor: pointer;
    transition: all 0.15s ease;
  }

  .gate-btn:hover {
    color: #34d399;
    background: rgba(16, 185, 129, 0.15);
  }

  .gate-btn.active {
    background: #10b981;
    color: #070b12;
    font-weight: 800;
    border-color: #34d399;
    box-shadow: 0 0 6px rgba(16, 185, 129, 0.4);
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

  /* Raw 42D Array */
  .raw-array-view {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(110px, 1fr));
    gap: 6px;
    background: rgba(0, 0, 0, 0.4);
    padding: 10px;
    border-radius: 8px;
    max-height: 480px;
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

  .badge {
    font-size: 0.65rem;
    font-weight: 700;
    padding: 2px 6px;
    border-radius: 3px;
  }
  .badge-green {
    background: rgba(16, 185, 129, 0.15);
    color: #10b981;
    border: 1px solid rgba(16, 185, 129, 0.3);
  }
  .badge-amber {
    background: rgba(245, 158, 11, 0.15);
    color: #f59e0b;
    border: 1px solid rgba(245, 158, 11, 0.3);
  }
</style>
