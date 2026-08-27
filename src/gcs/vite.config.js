import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
  plugins: [svelte()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    strictPort: true,
    proxy: {
      '/rosbridge': {
        target: 'ws://127.0.0.1:9090',
        ws: true,
        rewrite: (path) => path.replace(/^\/rosbridge/, ''),
      },
    },
  },
});
