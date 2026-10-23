# ADR 0003: Rapier for rigid bodies, behind `PhysicsAdapter`

Status: accepted (Milestone 1)

## Context

Debris, floating props and kinematic tower collision need a robust
rigid-body engine. The brief says not to build one, and not to tie
destruction code to engine internals.

## Decision

- `@dimforge/rapier3d-compat`. It is maintained and has sleeping, CCD,
  kinematic position-based bodies and collision groups. The compat build
  embeds its WASM, so it runs unchanged in browsers, workers and Node, which
  lets the unit and integration tests use the real engine.
- Everything goes through `PhysicsAdapter`. A second backend,
  `SimplePhysicsAdapter` (dependency-free, sphere-vs-box), passes the same
  contract tests. It is the fallback if WASM fails to load, and it keeps the
  contract honest.

## Consequences

- Jolt or a host game's physics can be swapped in by implementing one
  interface and passing the contract suite.
- The embedded base64 WASM inflates the bundle (about 5 MB, 1.85 MB
  gzipped). A separate `.wasm` file and lazy loading are planned
  (Milestone 7).
