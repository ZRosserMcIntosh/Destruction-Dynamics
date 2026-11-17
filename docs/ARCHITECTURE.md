# Architecture

## Principles

1. **Simulate what affects gameplay.** That means collision states, debris
   you can touch, buoyant objects, and water height wherever something floats.
2. **Approximate what affects visuals.** Spray, mist, dust and glass are
   particles. Foam and surges are cheap fields.
3. **Author what needs cinematic precision.** The macro motion of a landmark
   is a function of event time, not of an unconstrained rigid-body solve.
4. **Bound everything.** Every expensive subsystem has a budget from the
   quality preset. A governor degrades secondary work under load.
5. **Make it deterministic where it matters.** State and timing are exact
   functions of event time and seed. Secondary physics is allowed to differ
   between clients.

## Layers

```
            ┌──────────────────────────────── apps/playground ────────────────────────────────┐
            │  DeepEndSimulation (no rendering)    SceneView (Three.js)    CameraRig    UI/HUD │
            └───────┬──────────────┬───────────────┬──────────────┬───────────────┬───────────┘
                    │              │               │              │               │
        @dd/events  │  @dd/destruction   @dd/fluids │   @dd/rendering   @dd/audio  │
   EventDirector ───┘  TowerCollapse     HybridOcean    OceanView ...    AudioCueRouter
   EventAuthority      FacadePlan        NearFieldSolver TowerView        ProceduralAudioEngine
   AuthoritySync       DebrisManager     FarFieldOcean   ParticleView
   EventRecorder       TowerCollision    surges, buoyancy ParticleSim
                    │              │               │
                    └────────── @dd/physics (PhysicsAdapter: Rapier | Simple) ──────────┘
                                   │
                              @dd/core: math, RNG, frames, quality, budgets, tiles
```

Dependencies only point downward. `events` knows nothing about rendering,
physics or audio. `destruction` and `fluids` know nothing about Three.js.
`rendering` reads simulation state and never changes it. Each of physics,
water and audio sits behind a contract (`PhysicsAdapter`, `FluidAdapter`,
`AudioEngine`) with at least two implementations, so the contracts are tested
in practice and not only on paper.

## The event director

`EventDirector` owns one event instance:

- **Definition.** A versioned JSON document (`dd-event/1`) with a
  preparation window, then ordered committed stages (`INITIAL_FAILURE`,
  `ACTIVE_DESTRUCTION`, `WATER_IMPACT`, `SETTLING`, `AFTERMATH`), each with
  timed cues on channels: audio, camera, fluid, particles, physics,
  collision, world, custom. `validateEventDefinition` reports every problem
  with its path and never throws. The Deep End is
  `scenarios/the-deep-end/event.json`.
- **Event time.** Measured in seconds from the commit instant. Preparation
  occupies `[-prep, 0)`. `update(dt)` walks from boundary to boundary, so
  each transition and cue happens at its authored time whatever the frame
  step. Long frames fire cues late, and `lateBy` lets subsystems drop
  one-shot effects that are no longer meaningful.
- **Lifecycle.** `INTACT → PREPARING → TRIGGERED → stages… → AFTERMATH →
  RESTORATION → INTACT`. `cancel()` is legal only while PREPARING; once
  committed, the event cannot be cancelled. Every invalid control throws
  `InvalidTransitionError`.
- **Restoration.** Real-world duration (24 h by default, on an injected wall
  clock), event-time duration, or manual. During RESTORATION the collapse
  plays backwards to intact.
- **Determinism.** `seed` drives a named random stream for each cue
  (`deriveSeed(seed, cue.id)`) and `rng(stream)` for subsystems.
  `EventRecorder` plus `replay()` reproduce the observable timeline
  exactly.
- **Persistence and networking.** `save()` and `load()` hold a few scalars.
  `EventAuthority` (server) and `AuthoritySync` (client) exchange
  canonical instants, never fragments. See
  [ADR 0006](DECISIONS/0006-server-authoritative-events.md).

Subsystems use two mechanisms. **Cues** are one-shot events: a sound, a
particle burst, an impact on the water. **Continuous state** is sampled each
frame from `(state, time, restorationProgress)`: tower poses, facade
attachment, collision mode, turbidity. Continuous state is what makes
seeking, loading and late joining correct. Cues are for flourishes.

## Destruction at three scales

| Scale | Example | Mechanism |
|---|---|---|
| Macro | the 85-storey tower | `TowerCollapse`: 1 stump plus N tipping sections. An inverted-pendulum tip (θ'' = 3g/2L · sin θ, integrated with RK4) is time-normalised so impact lands exactly on the WATER_IMPACT stage. Sections bend relative to each other for lag and whip. The wreck pose is seeded and sits on the seabed. Restoration rewinds the motion. |
| Meso | facade panels, concrete chunks | `FacadePlan`: a seeded detach time per panel, earlier near the break and on the compression face, most of the rest at impact. Detached panels become Rapier bodies through `DebrisManager`, within the body budget. When the budget refuses a spawn, the panel turns into particles. |
| Micro | glass, sparks, dust, spray | `ParticleSim`: structure-of-arrays storage with fixed capacity, a cap set by the governor, and per-kind water behaviour |

**Floor layers.** The number of tipping layers comes from the quality preset:
5 at Potato, 12 at Balanced, 26 at High and 77 (one per floor) at Cinematic.
The fall is authored for every preset. At impact, the layered presets release
each layer into `LayerSolver`, an XPBD rigid-body solver with breakable slab
joints, flooding buoyancy, seabed friction and layer-to-layer contact. It
runs ahead of time, in a Web Worker in the browser, and stores keyframes, so
poses stay a function of event time. See
[ADR 0009](DECISIONS/0009-floor-layers.md).

`TowerCollision` switches gameplay collision from static boxes (intact) to
kinematic boxes that follow the authored motion, which sweeps debris and any
player out of the way, and then to static boxes at the wreck. The mode comes
from event state and time (`towerCollisionModeAt`). Each change bumps
`navigationVersion`, the hook where a game would rebuild navigation.

## Water

`HybridOcean` implements `FluidAdapter`:

- **Far field.** A sum of Gerstner waves. The same parameters drive the
  vertex shader and CPU sampling, so buoyant props ride the waves you see.
- **Near field.** `NearFieldSolver`, the linearised shallow-water equations
  on a staggered grid around the event (840 m square, 96² to 384² cells by
  preset). Wave speed follows depth (√(gD)), land cells are walls, a sponge
  layer absorbs at the edges, and foam is advected by the current. Bodies
  push on it through a pressure head; directional pushes add momentum.
- **Authored surges.** For a landmark impact, an expanding wave packet starts
  from the footprint's long axis. The result is a stadium-shaped front,
  biased in the direction of the fall, that fades out across the domain
  edge.
- **Input.** `WaterImpactEvent` carries position, footprint, intensity,
  duration, direction and a visual profile. It is an artistic input, not a
  hydrodynamic calculation.

Rendering uses two ocean meshes. A camera-following geometric grid covers
the far field. A uniform patch covers the near field and samples a float
texture holding solver height, surges and foam. See
[ADR 0007](DECISIONS/0007-hybrid-ocean.md).

## Budgets

`QualitySettings` carry real budgets: render scale, pixel ratio, shadows and
their resolution, ocean grid, fluid grid and rate, physics rate, debris
bodies, particles, facade density, city density, sediment and floating props.
`BudgetGovernor` watches frame time and moves through four degradation
levels, scaling particles, debris and the fluid step divisor. The director,
the authored motion and gameplay collision are never degraded.

## Frames

Simulation packages work in a Y-up **scene frame** (x east, y up, z south,
metres) relative to an event origin. The world uses **ENU**, the frame of
the Brazil tiles and Aviation Dynamics. `@dd/core/frames` converts between
them. See [ADR 0005](DECISIONS/0005-frames.md).

## What runs where each frame (playground)

1. `director.update(dt)`: cues fire, which emit particles, apply water
   impacts and write world overrides.
2. Tower poses at the effective time. Facade panels detached in
   `[prev, now)` spawn debris and glass.
3. Collision mode and kinematic targets.
4. Fixed-step physics (up to 6 substeps), with buoyancy and drag applied to
   props and submerged debris, then splash detection.
5. Ocean step (near-field substeps chosen by CFL), then the composite
   texture.
6. Continuous effects (impact spray, wreck bubbles) and the particle update.
7. Governor update, then render.
