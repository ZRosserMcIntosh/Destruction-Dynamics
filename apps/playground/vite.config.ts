import { defineConfig } from 'vite';

export default defineConfig({
  // Relative base so the build can be hosted from any static path.
  base: './',
  build: {
    target: 'es2022',
    chunkSizeWarningLimit: 4096,
  },
  server: { port: 5173 },
  preview: { port: 4173 },
});
