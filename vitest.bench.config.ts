import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['benchmarks/**/*.bench.ts'],
    testTimeout: 120_000,
    fileParallelism: false,
  },
});
