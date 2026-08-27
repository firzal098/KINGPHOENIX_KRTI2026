<script>
  import { toasts, removeToast } from '../ros.js';
</script>

<div class="toast-stack">
  {#each $toasts as toast (toast.id)}
    <div class="toast-item {toast.type}" on:click={() => removeToast(toast.id)}>
      <div class="toast-icon">
        {#if toast.type === 'success'}
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
            <polyline points="20 6 9 17 4 12"/>
          </svg>
        {:else if toast.type === 'error'}
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
            <circle cx="12" cy="12" r="10"/>
            <line x1="15" y1="9" x2="9" y2="15"/>
            <line x1="9" y1="9" x2="15" y2="15"/>
          </svg>
        {:else if toast.type === 'warning'}
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
            <path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/>
            <line x1="12" y1="9" x2="12" y2="13"/>
            <line x1="12" y1="17" x2="12.01" y2="17"/>
          </svg>
        {:else}
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
            <circle cx="12" cy="12" r="10"/>
            <line x1="12" y1="16" x2="12" y2="12"/>
            <line x1="12" y1="8" x2="12.01" y2="8"/>
          </svg>
        {/if}
      </div>
      <div class="toast-message font-mono">{toast.message}</div>
      <button class="toast-close" on:click|stopPropagation={() => removeToast(toast.id)}>&times;</button>
    </div>
  {/each}
</div>

<style>
  .toast-stack {
    position: fixed;
    bottom: 24px;
    right: 24px;
    display: flex;
    flex-direction: column-reverse;
    gap: 10px;
    z-index: 9999;
    max-width: 440px;
    pointer-events: none;
  }

  .toast-item {
    pointer-events: auto;
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 12px 16px;
    border-radius: 8px;
    backdrop-filter: blur(16px);
    box-shadow: 0 8px 30px rgba(0, 0, 0, 0.6);
    animation: slide-up 0.25s cubic-bezier(0.16, 1, 0.3, 1);
    cursor: pointer;
    border: 1px solid transparent;
  }

  @keyframes slide-up {
    from {
      opacity: 0;
      transform: translateY(20px) scale(0.95);
    }
    to {
      opacity: 1;
      transform: translateY(0) scale(1);
    }
  }

  .toast-item.info {
    background: rgba(15, 23, 42, 0.92);
    border-color: rgba(0, 240, 255, 0.4);
    color: #e2e8f0;
  }
  .toast-item.info .toast-icon {
    color: var(--accent-cyan);
  }

  .toast-item.success {
    background: rgba(6, 44, 27, 0.92);
    border-color: rgba(16, 185, 129, 0.5);
    color: #ecfdf5;
  }
  .toast-item.success .toast-icon {
    color: #34d399;
  }

  .toast-item.warning {
    background: rgba(45, 26, 3, 0.92);
    border-color: rgba(245, 158, 11, 0.5);
    color: #fffbeb;
  }
  .toast-item.warning .toast-icon {
    color: #fbbf24;
  }

  .toast-item.error {
    background: rgba(50, 10, 10, 0.94);
    border-color: rgba(239, 68, 68, 0.5);
    color: #fef2f2;
  }
  .toast-item.error .toast-icon {
    color: #f87171;
  }

  .toast-message {
    flex: 1;
    font-size: 0.82rem;
    font-weight: 500;
    line-height: 1.4;
  }

  .toast-close {
    background: transparent;
    border: none;
    color: var(--text-muted);
    font-size: 1.2rem;
    cursor: pointer;
    padding: 0 4px;
    line-height: 1;
    transition: color 0.15s ease;
  }
  .toast-close:hover {
    color: #ffffff;
  }
</style>
