# ADR 0007: Gerstner far field, shallow-water near field, authored surges

Status: accepted (Milestone 1, extended in Milestone 4)

## Context

The ocean must react to a skyscraper impact with a big, readable wave, foam,
spray and floating debris, and must also stretch to the horizon, all within
a frame budget. Full CFD over a coastline is out of the question.

## Decision

- **Far field.** Up to 8 Gerstner waves from a seeded sea state, evaluated
  identically on GPU (vertex shader) and CPU (buoyancy, cameras). FFT waves
  will replace them on WebGPU without changing the interface.
- **Near field.** Linearised shallow-water equations on a C-grid over an
  840 m square around the event. Speed follows bathymetry. Land cells are
  walls. A sponge absorbs at the edges. Foam is advected. Bodies push
  through a pressure head.
- **Authored surge.** For landmark impacts, an art-directed wave packet
  expands from the footprint's long axis. It is faded at the domain edge.
  This guarantees the money shot whatever the grid resolution.
- **Coupling.** Everything goes through `FluidAdapter` and
  `WaterImpactEvent`. The tower never touches solver internals.

## Consequences

- Linear equations do not break waves or run up the beach. Flooding the
  promenade needs a nonlinear or wet/dry extension (Milestone 4 remainder).
- Ripples from before a seek or load are not reconstructed. Surges are,
  because they are analytic in time.
- Outside the near-field square only the swell exists. The surge fades
  before the edge, so this is invisible at the default framing.
