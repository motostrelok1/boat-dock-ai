import { defineConfig } from 'vite';
import { resolve } from 'node:path';

export default defineConfig({
  base: '/boat-dock-ai/',
  build: {
    rollupOptions: {
      input: {
        main: resolve(import.meta.dirname, 'index.html'),
        benchmark: resolve(import.meta.dirname, 'benchmark.html')
      }
    }
  },
  server: {
    port: 7100,
    strictPort: true,
    host: true
  }
});
