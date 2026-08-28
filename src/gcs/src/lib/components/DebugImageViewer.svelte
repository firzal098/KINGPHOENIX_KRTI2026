<script>
  import { onMount } from 'svelte';
  import {
    debugImage,
    debugImageSrc,
    isPerceptionStreamActive,
    togglePerceptionStream,
    targetGateLabel
  } from '../ros.js';
  import World3DViewer from './World3DViewer.svelte';

  let activeTab = 'fpv'; // 'fpv' | '3d'
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
</script>

<div class="gcs-card viewer-card {activeTab === 'fpv' && !$isPerceptionStreamActive ? 'collapsed-card' : 'glow-cyan'}" bind:this={containerEl}>
  <!-- Card Header with FPV / 3D Tab Switcher -->
  <div class="viewer-header">
    <div class="header-left">
      <div class="tab-switcher font-hud">
        <!-- FPV Tab Button -->
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

        <!-- 3D Arena Tab Button -->
        <button
          class="tab-btn {activeTab === '3d' ? 'active' : ''}"
          on:click={() => (activeTab = '3d')}
          title="Switch to Interactive 3D Arena World"
        >
          <span class="tab-dot live-emerald"></span>
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <circle cx="12" cy="12" r="10"/>
            <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"/>
            <path d="M2 12h20"/>
          </svg>
          <span>3D ARENA</span>
        </button>
      </div>
    </div>

    <div class="header-controls">
      {#if activeTab === 'fpv'}
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
            <!-- Eye Open -->
            <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/>
              <circle cx="12" cy="12" r="3"/>
            </svg>
          {:else}
            <!-- Eye Slash -->
            <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
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
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
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
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <rect x="3" y="3" width="18" height="18" rx="2"/>
              <path d="M9 3v18"/>
            </svg>
          </button>
        {/if}
      {:else}
        <!-- 3D Tab Active Pill -->
        <span class="info-pill font-mono live-3d">
          THREE.JS &bull; 60 FPS
        </span>
      {/if}

      <!-- Fullscreen Button (shared) -->
      <button class="ctrl-btn" on:click={toggleFullscreen} title="Toggle Fullscreen">
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <polyline points="15 3 21 3 21 9"/>
          <polyline points="9 21 3 21 3 15"/>
          <line x1="21" y1="3" x2="14" y2="10"/>
          <line x1="3" y1="21" x2="10" y2="14"/>
        </svg>
      </button>
    </div>
  </div>

  <!-- Content Viewport -->
  {#if activeTab === 'fpv'}
    <!-- 1. FPV Video Viewport -->
    {#if $isPerceptionStreamActive}
      <div class="viewport-wrapper">
        <!-- Native High-Speed JPEG Image Stream -->
        {#if $debugImageSrc}
          <img
            src={$debugImageSrc}
            alt="Perception Stream"
            class="video-feed {fitMode}"
          />
        {:else}
          <!-- Fallback Canvas for raw Image bytes -->
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
    {/if}
  {:else if activeTab === '3d'}
    <!-- 2. Interactive 3D Arena World View -->
    <div class="viewport-wrapper viewport-3d">
      <World3DViewer />
    </div>
  {/if}
</div>

<style>
  .viewer-card {
    display: flex;
    flex-direction: column;
    overflow: hidden;
    height: 100%;
    min-height: 400px;
    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  }

  .collapsed-card {
    min-height: auto !important;
    height: auto !important;
    border-color: rgba(255, 255, 255, 0.08);
  }

  .viewer-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 12px;
    background: rgba(0, 0, 0, 0.4);
    border-bottom: 1px solid var(--border-subtle);
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

  .tab-dot.live-emerald {
    background: #10b981;
    box-shadow: 0 0 8px #10b981;
  }

  .tab-btn.active .tab-dot {
    background: #070b12;
    box-shadow: none;
  }

  .topic-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--text-muted);
    transition: all 0.3s ease;
  }

  .topic-dot.live {
    background: var(--accent-cyan);
    box-shadow: 0 0 10px var(--accent-cyan);
    animation: blink 1.2s infinite alternate;
  }

  .header-controls {
    display: flex;
    align-items: center;
    gap: 6px;
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

  .viewport-wrapper {
    position: relative;
    flex: 1;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #05070c;
    overflow: hidden;
    min-height: 340px;
  }

  .video-feed, .video-canvas {
    width: 100%;
    height: 100%;
    max-height: 540px;
    display: block;
  }

  .video-feed.contain, .video-canvas.contain {
    object-fit: contain;
  }

  .video-feed.cover, .video-canvas.cover {
    object-fit: cover;
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

  .viewport-3d {
    height: 100%;
    min-height: 400px;
    background: #070b12;
    display: flex;
    position: relative;
  }

  .info-pill.live-3d {
    background: rgba(16, 185, 129, 0.15);
    color: #34d399;
    border: 1px solid rgba(16, 185, 129, 0.3);
  }
</style>
