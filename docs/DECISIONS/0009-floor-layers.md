# ADR 0009: Floor layers chosen by fidelity, released into a precomputed solver at impact

Status: accepted

## Context

The first version split the tower into five authored sections that blended
into an authored wreck. At higher fidelity we want the structure to come
apart floor by floor: each floor its own body, connected to its neighbours
until the slam overloads the connection. That has to stay compatible with
scrubbing, save/load, restoration rewind and multiplayer reconstruction
(ADR 0006), all of which rely on the collapse being a function of event
time.

## Decision

- `QualitySettings.structuralLayers` sets how many layers sit above the
  stump: Potato 5, Balanced 12, High 26 (about three floors each),
  Cinematic 77 (one per floor).
- The fall stays authored, so impact lands exactly on WATER_IMPACT. The bend
  along the tower is a continuous function of height, so the fall has the
  same shape at 5 layers and at 77.
- At impact, every layer is released with the authored velocity into
  `LayerSolver`, an XPBD rigid-body solver. The solver handles:
  - slab joints at the four corners of each floor interface, each with a
    seeded breaking stretch;
  - buoyancy that fades as each layer floods, at a seeded rate;
  - water drag;
  - seabed contact with static friction;
  - contact between neighbouring layers once their joints have failed;
  - sleeping, so settled layers cost nothing.
- The solver runs **ahead of time** and stores keyframes at 30 Hz. Poses
  after impact are interpolated from them, so the collapse is still a pure
  function of event time and seed.
- In the browser the precompute runs in a Web Worker, started as soon as the
  tower is built. Until it finishes, the tower uses the authored wreck path.
  A late result is attached only before impact, so the tower never jumps.
- Collision stays kinematic, following the layers, until they settle (or
  SETTLING ends), then switches to static wreck boxes.
- Potato keeps the authored wreck (`layerSimulation: false`).

## Amendments (Day 1 challenge: the indestructible cheese)

- Static convex obstacles; layers release at first obstacle contact.
- Joints break by carried force (multiplier / h²), not stretch.
- New penetration is resolved in full; old overlap leaks at ≤ 6 m/s; contacts
  never push bodies apart faster than that. Crushing contacts lose 18% of
  velocity. Friction against obstacles.
- Adaptive substeps (2–6) and a cached ground grid.
- Details and measurements: [LESSONS.md](../LESSONS.md).

## Consequences

- Precompute cost, measured in Node on a 2.1 GHz Xeon vCPU (warm, `npm run
  bench`): 47 ms for 12 layers, 0.22 s for 26, 0.52 s for 77, 1.16 s for 77
  with the cheese wedge. It runs off the main thread in the browser.
- The step uses only arithmetic and `sqrt`, so a given engine reproduces it
  exactly. Across browsers it also depends on the scenario's ground function
  (`Math.pow`), so cross-engine bit-equality is not guaranteed. Gameplay
  should use the collision boxes, which every client derives from the same
  keyframes.
- Layer-to-layer contact covers neighbours up to two indices apart. Layers
  further apart can overlap in rare piles. A full broad phase was tried and
  needs the velocity contact pass first (LESSONS.md).
- The playground draws each layer as its own group (core, slabs, panels).
  At 77 layers that is roughly 230 draw calls for the tower. Merging layers
  into one instanced draw is part of Milestone 7.

## Amendment: velocity pass and streaming

- Contacts now get a velocity pass after the position solve (restitution 0.1
  above a 2gh threshold, dynamic friction μ 0.8). Layers sleep only while in
  contact and still for 1 s. This replaced the crush damping and separation
  clamp.
- The worker streams keyframes in 0.5 s chunks. `TowerCollapse.attachLayers`
  takes the number of frames ready and `extendLayers` adds more; poses hold at
  the newest ready keyframe. Play waits only for the first chunk (9–29 ms
  measured in Node, warm). Full solve: 12 layers 0.10 s, 26 layers 0.23 s,
  77 floors 0.70 s, 77 floors with the cheese 1.13 s.
