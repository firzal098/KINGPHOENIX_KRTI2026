<script>
  import World3DViewer from './World3DViewer.svelte';
  import { targetGateLabel } from '../ros.js';

  let containerEl;
  let isFullscreen = false;

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

<div class="gcs-card arena-card glow-cyan" bind:this={containerEl}>
  <!-- Card Header -->
  <div class="arena-header">
    <div class="header-left">
      <span class="live-dot"></span>
      <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <circle cx="12" cy="12" r="10"/>
        <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"/>
        <path d="M2 12h20"/>
      </svg>
      <span class="font-hud header-title">3D ARENA WORLD</span>
    </div>

    <div class="header-right font-mono">
      <span class="info-pill target-pill font-hud">
        {$targetGateLabel}
      </span>

      <span class="info-pill live-pill">
        THREE.JS &bull; 60 FPS
      </span>

      <button class="ctrl-btn" on:click={toggleFullscreen} title="Toggle Arena Fullscreen">
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <polyline points="15 3 21 3 21 9"/>
          <polyline points="9 21 3 21 3 15"/>
          <line x1="21" y1="3" x2="14" y2="10"/>
          <line x1="3" y1="21" x2="10" y2="14"/>
        </svg>
      </button>
    </div>
  </div>

  <!-- 3D Arena Canvas Container -->
  <div class="arena-viewport">
    <World3DViewer />
  </div>
</div>

<style>
  .arena-card {
    display: flex;
    flex-direction: column;
    overflow: hidden;
    height: 480px;
    max-height: 480px;
    background: rgba(10, 16, 26, 0.75);
    border: 1px solid rgba(0, 240, 255, 0.2);
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5);
    border-radius: 8px;
  }

  :fullscreen.arena-card,
  :-webkit-full-screen.arena-card {
    height: 100vh !important;
    max-height: 100vh !important;
  }

  :fullscreen .arena-viewport,
  :-webkit-full-screen .arena-viewport {
    height: calc(100vh - 50px) !important;
    max-height: none !important;
  }

  .arena-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 14px;
    background: rgba(0, 0, 0, 0.45);
    border-bottom: 1px solid var(--border-subtle, rgba(255, 255, 255, 0.08));
  }

  .header-left {
    display: flex;
    align-items: center;
    gap: 8px;
    color: var(--accent-cyan, #00f0ff);
  }

  .header-title {
    font-size: 0.85rem;
    font-weight: 700;
    letter-spacing: 0.06em;
    color: #f1f5f9;
  }

  .live-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #10b981;
    box-shadow: 0 0 8px #10b981;
  }

  .header-right {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .info-pill {
    font-size: 0.7rem;
    padding: 3px 8px;
    border-radius: 4px;
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-secondary, #94a3b8);
    border: 1px solid rgba(255, 255, 255, 0.08);
  }

  .target-pill {
    color: var(--accent-green, #10b981);
    background: rgba(16, 185, 129, 0.12);
    border-color: rgba(16, 185, 129, 0.3);
    font-weight: 700;
  }

  .live-pill {
    color: var(--accent-cyan, #00f0ff);
    background: rgba(0, 240, 255, 0.08);
    border-color: rgba(0, 240, 255, 0.2);
  }

  .ctrl-btn {
    background: rgba(255, 255, 255, 0.06);
    border: 1px solid rgba(255, 255, 255, 0.1);
    color: var(--text-secondary, #94a3b8);
    border-radius: 4px;
    padding: 4px 6px;
    cursor: pointer;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    transition: all 0.15s ease;
  }

  .ctrl-btn:hover {
    color: #fff;
    background: rgba(0, 240, 255, 0.15);
    border-color: var(--accent-cyan, #00f0ff);
  }

  .arena-viewport {
    flex: 1;
    position: relative;
    width: 100%;
    min-height: 440px;
    overflow: hidden;
  }
</style>
