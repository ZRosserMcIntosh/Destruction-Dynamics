# ADR 0002: Ship on WebGL2 now; add a WebGPU path through Three's node materials

Status: accepted (Milestone 1)

## Context

The brief names WebGPU as the primary API, with a practical WebGL2 fallback.
In Three.js r186 the WebGPU renderer (`three/webgpu`) needs node materials
(TSL). Hand-written GLSL `ShaderMaterial`s and `onBeforeCompile` patches,
which the ocean, sky, caustics, particles and underwater pass use, do not
run on it. WebGPU support is also still uneven across desktop browsers and
in headless CI.

## Decision

- Milestone 1–3 render through `WebGLRenderer` (WebGL2) only. The HUD
  reports when `navigator.gpu` is available, so it is clear that WebGPU is
  present but unused.
- Simulation stays renderer-agnostic, so moving to WebGPU affects only
  `@destruction-dynamics/rendering`.
- Milestone 7: port the custom materials to TSL. Then WebGPU becomes the
  default and WebGL2 (which TSL also targets) the fallback. Move the
  near-field solver to a compute shader on WebGPU (see ADR 0004).

## Consequences

- No compute shaders yet. The near-field water runs on the CPU and is the
  largest CPU cost measured (PERFORMANCE_TARGETS.md).
- The shader code written now will be rewritten in TSL. It is kept small
  and in one place (`shaderChunks.ts`, `ocean.ts`) to limit that cost.
