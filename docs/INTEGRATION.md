# Integration guide

How to use Destruction Dynamics outside the playground: in the Brazil open
world, in another Three.js application, or as the reference for an engine
port.

## 1. Embedding the packages

Every package is ESM TypeScript with a single entry point (`src/index.ts`).
Use them from a bundler (Vite, esbuild, webpack) inside this workspace, or
copy them. Publishing to npm is planned (semantic versioning, 0.x until the
APIs settle).

[`examples/headless-event.ts`](../examples/headless-event.ts) is the
smallest complete integration: director, authored tower, Rapier collision
and the ocean, driven from a plain loop with no rendering. A test runs it.

```ts
const director = new EventDirector(definitionJson, { seed });   // validates the definition
director.on('cue', (c) => { /* sounds, particles, water impacts, camera, world writes */ });
director.arm();               // or trigger() to skip the preparation window
// each frame:
director.update(dt);
const poses = tower.posesAt(director.time, director.restorationProgress);
collision.update(towerCollisionModeAt(director.state, director.time, timeline), poses);
```

To render with Three.js, use the views in `@destruction-dynamics/rendering`
(`TowerView`, `OceanView`, `ParticleView`, `DebrisView`, `UnderwaterView`).
`apps/playground/src/sceneView.ts` shows the wiring.

## 2. Coordinate frames and world tiles

- The world frame is **ENU** (x east, y north, z up, metres), shared with
  Aviation Dynamics (`avd::world`) and the Brazil tiles.
- Simulation code uses the **scene frame** (x east, y up, z south) relative
  to an event origin. Use `sceneFromWorld(origin, enu)` and
  `worldFromScene(origin, v)` from `@destruction-dynamics/core` at the
  boundary. Positions near the event therefore stay small and keep
  `float32` precision on the GPU.
- Tiles are 500 m squares centred on multiples of 500 m: `tileAt(east,
  north)` gives `T_i_j`, as in Aviation Dynamics. An event declares its ENU
  `bounds`, and `tilesForBounds` returns every tile it touches, so cross-tile
  events work.

What the host world streamer provides:

```ts
interface TileActivationHost {
  acquire(tiles: readonly TileCoord[], reason: string): TileLease; // keep these tiles loaded until release()
  isActive(tile: TileCoord): boolean;
}
```

The playground uses `InMemoryTileHost` (reference counting). The event
acquires a lease when it arms or triggers and releases it on return to
INTACT. Nothing else needs the whole world loaded.

**World-state overrides.** `WorldStateStore` keeps per-tile records such as
`{ eventId, state: 'wreck', restoreAtMs }`. They are saved with the game
(`toJSON` and `fromJSON`) and expired by `expire(now)`. When a tile streams
in, the host asks `forTile(tile)` and loads the wreck variant if an override
is present, even if the event itself is not loaded.

**Collision and navigation.** `TowerCollision` changes the physics
representation and the playground bumps `navigationVersion` on every change.
Hook navmesh rebuilds or link toggles for the affected tiles there.

## 3. Physics

Destruction code only sees `PhysicsAdapter`. To use another engine (Jolt,
the host game's own physics), implement the interface and run
`packages/physics/test/contract.test.ts` against it. Add it to the
`backends` table in that file. Collision groups use the
`group`/`mask` 16-bit pair.

## 4. Water and the `liquid` engine

There is no existing fluid engine to integrate yet: the `liquid` repository
is empty (see RESEARCH.md). When one exists:

1. Implement `FluidAdapter`: `step`, `heightAt`, `velocityAt`, `sample`,
   `applyImpact(WaterImpactEvent)`, `applyDisturbance`, `reset`.
2. Run the fluid contract suite in `packages/fluids/test/fluids.test.ts`
   (add it to `adapters`).
3. Buoyancy (`computeBuoyancy`), the particles' water behaviour and the
   cameras then work unchanged.

Rendering is separate. `OceanView` currently reads `HybridOcean`'s composite
texture. Another engine would provide its own view, or export a heightfield
texture with the same layout (RG float: height, foam).

## 5. Audio

Destruction code never plays sound. Director cues on the `audio` channel go
through `AudioCueRouter` to an `AudioEngine`. Implement `AudioEngine` to
target game middleware (Wwise, FMOD, Unreal MetaSounds). The
`ProceduralAudioEngine` placeholder shows what spatialisation and the
underwater state need.

## 6. Multiplayer

The server runs `EventAuthority` (arm, trigger, cancel, tick) on a monotonic
clock and broadcasts `AuthoritySnapshot`s. These are small JSON objects with
a sequence number. Each client runs `AuthoritySync.apply(snapshot,
serverNowMs)`, where `serverNowMs` is the client's estimate of server time
from your clock sync. The director seeks to the canonical event time, so
late joiners arrive in the correct stage with the correct seed.

| Deterministic (identical on every client) | Not deterministic (cosmetic, local) |
|---|---|
| Lifecycle state, stage, event time | Debris trajectories after spawn (Rapier, frame-rate-dependent stepping) |
| Tower section poses and the wreck pose | Particles |
| Facade detach times and which panels detach | Near-field ripples from debris splashes |
| Collision mode and wreck colliders | Floating props' exact drift |
| Surge timing and shape | Camera shake |
| Per-cue seeds | |

Gameplay should depend only on the left column.

## 7. Unreal Engine and other engines

The engine-neutral parts are the event format (`dd-event/1`, JSON), the
lifecycle semantics, the authority snapshot and the authored-motion maths
(`TowerCollapse` is a few hundred lines of pure functions of time). Planned approach: an
Unreal plugin that reads the same event JSON, uses a C++ port of the director
or a Rust core shared through WASM and native builds, and drives Chaos
Destruction for meso pieces. The browser playground stays the reference
implementation and test oracle: recorded timelines (`EventRecording`) can be
replayed on both and compared.
