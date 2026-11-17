import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: [
      'packages/*/test/**/*.test.ts',
      'apps/*/test/**/*.test.ts',
      'scenarios/**/*.test.ts',
      'examples/**/*.test.ts',
    ],
    benchmark: { include: ['benchmarks/**/*.bench.ts'] },
  },
});
