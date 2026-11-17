# Roadmap

Statuses use the README's vocabulary: implemented, tested, demonstrated,
planned, experimental.

## Milestone 1: Repository and runnable foundation. Done.
Monorepo, browser playground, 3D scene, cameras, ocean, Rapier integration,
CI, tests, docs.

## Milestone 2: Event director. Done.
Data-driven lifecycle, timeline, triggers, reset, saveable state, debug UI,
facade fracture and debris.

## Milestone 3: Tower collapse prototype. Done.
Authored tip, secondary debris, collision transitions, impact effects.

## Milestone 4: Interactive water. Partly done.
- Done: local impact disturbances, wave propagation, foam, spray, floating
  objects with buoyancy, underwater camera, CPU instrumentation.
- Remaining:
  - GPU compute path for the near field (WebGPU);
  - waves running up onto the beach and promenade (flooding);
  - wake and splash coupling for every moving body, not only debris crossing
    the surface;
  - GPU timing queries.

## Milestone 5: Cinematic refinement
- Higher-detail tower: mullions, balconies, interior floors visible through
  broken glass, a damaged stump top.
- Materials: environment-map reflections, a wet-surface darkening pass.
- Progressive facade deformation (vertex bending ahead of detachment).
- Soft particles using the depth buffer, lit dust and mist volumes.
- Licensed or recorded sound layers behind `AudioEngine`, with a convolution
  reverb for the city.
- Authored camera rails with easing, depth of field in Cinematic.

## Milestone 6: Underwater salvage (standalone scenario)
- Flooded interior: one or two navigable floors of the wreck.
- Diver controller: momentum, buoyancy control, stamina.
- Diver propulsion vehicle.
- Cargo pods with lift bags; surface recovery zones.
- Can be played without running the collapse first: it loads into the
  AFTERMATH state through `director.load()`.

## Milestone 7: Performance
- Real-GPU frame-time measurements on four reference machines.
- WebGPU renderer path (ADR 0002). Near-field solver on compute or in a
  Rust/WASM crate (ADR 0004).
- Rapier loaded asynchronously from its own `.wasm` file. Code splitting.
- Fracture-asset streaming (`FractureAssetRef`). LOD for the tower.
  Impostors for the city.
- Shader warm-up to avoid compile hitches when the event starts.

## Milestone 8: São Paulo bridge (reusability proof)
- A fictionalised cable-stayed bridge. Cable tension visualisation.
  Cable-snap and hum audio (the layers exist in `SOUND_TYPES`).
- Authored deck deformation, roadway collision states, traffic and
  navigation updates through `navigationVersion`, restoration.
- A new event definition and layout. The director, physics, debris, water
  and audio stay unchanged. Any change it forces in them is a finding to
  record.

## Cross-cutting
- Multiplayer transport over the existing `EventAuthority` model, plus a
  monotonic clock-sync service.
- Integration samples for the Brazil world streamer, once its repository is
  available.
- An Unreal Engine port of the event format and director (the JSON schema is
  engine-neutral).
