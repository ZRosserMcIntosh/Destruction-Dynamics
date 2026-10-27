# Performance targets and measurements

## Targets (not yet verified on GPU hardware)

| Tier | Target | Budget highlights |
|---|---|---|
| Potato ("Dorito Mode") | 30 FPS at reduced resolution | 0.6 render scale, no shadows, 96² fluid at 30 Hz, 40 debris bodies, 1,500 particles |
| Balanced | 60 FPS on mainstream desktop hardware | 0.85 render scale, 1024 shadows, 160² fluid, 150 bodies, 6,000 particles |
| High | 60 FPS on capable gaming hardware | full resolution, 2048 shadows, 256² fluid, 400 bodies, 15,000 particles |
| Cinematic | highest practical fidelity; non-real-time capture acceptable | 4096 shadows, 384² fluid, 120 Hz physics, 1,000 bodies, 40,000 particles |

The full list of budgets is `QUALITY_PRESETS` in `packages/core/src/quality.ts`.
The e2e suite checks that switching presets changes them in the running app.

**None of the FPS targets has been measured on a real GPU.** The only browser
runs so far are the automated ones, which use SwiftShader (software
rasterisation on the CPU) at 1–4 FPS. That shows the pipeline works and says
nothing about GPU frame rates. Measuring on representative hardware is the
first task of Milestone 7.

## Measured CPU costs

`npm run bench`, Node 22.22, Linux, 4 vCPU Intel Xeon @ 2.10 GHz (a cloud
container), 2026-10-23. Each figure is the mean (and p95) time per call.

| Subsystem, per frame | Potato | Balanced | High | Cinematic |
|---|---|---|---|---|
| Near-field shallow water, one 1/60 s step | 0.53 ms (0.69) | 1.39 ms (2.29) | 3.97 ms (4.75) | 8.64 ms (10.29) |
| Hybrid ocean step with an active landmark surge (solver plus composite texture) | 0.89 ms (1.27) | 2.32 ms (3.50) | 4.99 ms (6.36) | 12.98 ms (18.91) |
| Particle update at full budget | 0.04 ms | 0.15 ms | 0.33 ms | 0.93 ms |
| Rapier step at the debris budget (bodies fall, then settle) | 0.02 ms | 0.06 ms | 0.97 ms | 1.49 ms |
| Buoyancy for all floating props | 0.05 ms | 0.13 ms | 0.25 ms | 0.41 ms |

| Authored structure | Cost |
|---|---|
| Tower section poses at an event time | 0.002 ms |
| Facade plan build (2,720 panels) | 0.94 ms, once per seed or preset change |
| Director update | below 0.001 ms |
| Floor-layer precompute (once per seed or preset, in a Web Worker in the browser) | Streamed: playback starts after the first chunk, 9–29 ms. Full solve: 12 layers 0.10 s · 26 layers 0.23 s · 77 floors 0.70 s · 77 floors + cheese wedge 1.13 s (warm, Node) |

The raw output is written to `benchmarks/results/latest.json`. CI runs the
benchmark job and uploads it as an artifact.

### What this says

- **The near-field fluid is the dominant CPU cost.** At High it uses about
  30% of a 16.7 ms frame, which is too much next to rendering. It is the
  first candidate for optimisation: a WebGPU compute shader, Rust/WASM SIMD,
  or moving the solver to a worker. This is the profiling evidence ADR 0004
  asked for before any WASM migration.
- Debris physics is cheap at these budgets because Rapier sleeps settled
  bodies. Awake worst cases (everything moving at once) still need
  measuring.
- The authored layers (director, poses, facade plan) cost almost nothing.
  That is the payoff of authoring the macro motion.

## Instrumentation in the app

The playground HUD shows FPS, frame time and per-category CPU time (director,
physics, fluid, particles, render submit), averaged with an EMA by
`FrameProfiler`. It also shows body counts (dynamic, sleeping, kinematic,
static), the debris budget and refusals, the particle count and limit, the
fluid grid and substeps, the current and peak wave height, draw calls and
triangles, the pixel ratio, the JS heap where the browser exposes it, and the
budget governor's degradation level. `window.__DD.state()` exposes the same
data to automation.

GPU timing (`EXT_disjoint_timer_query_webgl2`) is not wired up yet. Planned
for Milestone 7.

## Degradation policy

When the EMA frame time stays more than 15% over target for 30 frames, the
governor moves one level down. It moves back up after 120 frames under 70%
of target.

| Level | Particles | Debris bodies | Fluid solver |
|---|---|---|---|
| 0 | 100% | 100% | every frame |
| 1 | 60% | 70% | every frame |
| 2 | 35% | 45% | every 2nd frame |
| 3 | 20% | 25% | every 3rd frame |

The director, authored motion, collision states and audio cues are never
degraded. `apps/playground/test/simulation.test.ts` checks that the event
still completes correctly at level 3.
