# ADR 0004: TypeScript first; Rust/WASM only where profiling shows it is needed

Status: accepted. The first migration target was identified on 2026-10-23.

## Context

The brief suggests Rust compiled to WASM for performance-critical code, but
warns that WASM is not automatically faster. Crossing between JS and WASM
costs time, data has to be marshalled, and a second toolchain makes
contributing harder.

## Decision

- Write everything in TypeScript with allocation-free inner loops
  (structure-of-arrays, typed arrays).
- Measure with `npm run bench` and the in-app profiler before moving
  anything.
- Move a subsystem only when it is a top cost at its preset's budget **and**
  its data can stay on the WASM side (or the GPU) across frames.
- Do not create the `crates/simulation` directory until there is code for
  it.

## Evidence (2026-10-23)

The near-field shallow-water step with an active surge costs 2.3 ms at
Balanced, 5.0 ms at High and 13.0 ms at Cinematic per 1/60 s on a 2.1 GHz
Xeon vCPU. That is the largest CPU cost by about 5×. Particles, buoyancy and
the director are each under 1 ms at their budgets.

## Consequences

- Next: compare the WebGPU compute version of the solver (ADR 0002) with a
  Rust/WASM SIMD version that runs in a worker and shares the heightfield
  through a SharedArrayBuffer. Choose by measurement.
  Cross-origin isolation is needed for SharedArrayBuffer and will be
  documented for hosts.
