# Architecture decision records

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-hierarchical-authored-destruction.md) | Landmark destruction is authored motion with bounded runtime physics | Accepted |
| [0002](0002-webgl2-first-webgpu-later.md) | Ship on WebGL2 now; add a WebGPU path through Three's node materials | Accepted |
| [0003](0003-rapier-behind-an-adapter.md) | Rapier for rigid bodies, behind `PhysicsAdapter` | Accepted |
| [0004](0004-typescript-first-wasm-when-measured.md) | TypeScript first; Rust/WASM only where profiling shows it is needed | Accepted (first target identified) |
| [0005](0005-frames.md) | Scene frame (Y-up) inside the simulation, ENU at world boundaries | Accepted |
| [0006](0006-server-authoritative-events.md) | Server-authoritative event state; clients reconstruct the visuals | Accepted |
| [0007](0007-hybrid-ocean.md) | Gerstner far field, shallow-water near field, authored surges | Accepted |
| [0008](0008-procedural-placeholder-audio.md) | Procedural placeholder audio behind `AudioEngine` | Accepted |
| [0009](0009-floor-layers.md) | Floor layers chosen by fidelity, released into a precomputed solver at impact | Accepted |
