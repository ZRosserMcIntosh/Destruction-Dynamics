# Dependencies

The goal is few runtime dependencies, each of them well maintained.

## Runtime

| Package | Version | License | Why | Alternatives considered |
|---|---|---|---|---|
| `three` | ^0.186 | MIT | Scene graph, WebGL2 renderer, instancing, shadows. The integration target named in the brief. | Babylon.js (heavier, a different ecosystem), raw WebGL (too much to rebuild) |
| `@dimforge/rapier3d-compat` | ^0.21 | Apache-2.0 | Rigid bodies, kinematic bodies, sleeping, CCD, collision groups | Jolt, cannon-es, Ammo (see RESEARCH.md) |

That is the complete runtime list. Everything else is our own code: event
schema validation (no JSON-schema library), RNG, math, particles, water,
audio synthesis and UI (plain DOM).

## Development

| Package | Purpose |
|---|---|
| `typescript` ~5.9 | Strict type checking. TypeScript 7 is out, but typescript-eslint supports up to 6.0. |
| `vite` ^8 | Dev server and static build |
| `vitest` ^5 | Unit tests, the headless integration test and the benchmark harness |
| `@playwright/test` 1.56.1 | Browser end-to-end tests. Pinned to the Chromium build installed in the development container. |
| `eslint` ^9, `typescript-eslint`, `@eslint/js`, `globals` | Linting |
| `prettier` ^3 | Formatting |

## Bundle

The production playground is one JS chunk of about 5.0 MB (1.85 MB gzipped).
Most of it is Rapier's WASM, embedded as base64 by the compat build. Planned:
load Rapier through a dynamic import, and later through the non-compat build
with a separately fetched `.wasm` file for better compression and caching.

## Policy

- No paid services, API keys or network calls at runtime.
- No third-party assets. Geometry and sound are procedural.
- New runtime dependencies need an entry here and, for significant ones, an
  ADR.
