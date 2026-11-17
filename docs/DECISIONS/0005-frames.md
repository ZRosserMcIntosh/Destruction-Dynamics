# ADR 0005: Scene frame (Y-up) inside the simulation, ENU at world boundaries

Status: accepted (Milestone 1)

## Context

The Brazil world and Aviation Dynamics use ENU (x east, y north, z up)
metres, with 500 m tiles centred on multiples of 500 m. Three.js is Y-up.
World coordinates hundreds of kilometres from the origin lose precision as
`float32` on the GPU.

## Decision

- The simulation packages use a **scene frame**: x east, y up, z south,
  metres, relative to an event origin. It is right-handed and maps to
  Three.js without conversion.
- Conversion to ENU (`(e, n, u) → (e, u, −n)`, a proper rotation) happens
  only at integration boundaries, through `@destruction-dynamics/core`
  `frames.ts`.
- Event bounds in definitions are in ENU world metres, because tile
  activation is a world concern.

## Consequences

- Event-local positions stay small and precise. A host with a floating
  origin only has to move the event origin.
- Integrators must convert at the boundary. INTEGRATION.md documents it.
