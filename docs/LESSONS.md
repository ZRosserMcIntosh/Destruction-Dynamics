# Viewer challenges → engine lessons

The rule: every absurd challenge a subscriber sets gets built, and whatever it
teaches goes into the engine, not into a one-off scenario hack. This ledger
records each challenge and what the engine gained from it.

## Day 1: "A giant block of cheese in the center forever (make it indestructible pls)"

Requested by @Fartman32. Built as the `cheese` variant of The Deep End
(`?scenario=cheese`): a 60 m-tall wedge of Swiss cheese, cut from a 200 m
wheel, sits in the shallows on the tower's fall line. The tower breaks over
it, and it survives every collapse, reset and restoration.

What the engine gained (all in `@destruction-dynamics/destruction`,
`physics` and `scenarios`, usable by any future event):

| Lesson | What changed in the engine |
|---|---|
| Structures need to hit things that are not ground | **Static convex obstacles** in the layer solver (`StaticObstacle`, `ConvexShape`, `boxShape`, `wedgeShape`), plus convex-hull colliders in `PhysicsAdapter` (Rapier `convexHull`; the simple backend uses the bounding box) |
| An authored path cannot pass through an obstacle | **Release on first contact**: layers leave the authored fall the instant it touches an obstacle (dense sampling plus bisection to 1 ms). Without obstacles they still release at impact. |
| Coarse floor checks missed the overlap between check times, so floors were already deep in the cheese when released, and the solver fired them back out (chunks thrown 50–90 m into the air) | Release detection samples densely; contact sampling covers both directions (layer surface points inside the obstacle, and obstacle edges cutting through a layer) |
| A capped push-out applied every substep turns into velocity: a catapult | **New overlap versus old overlap**: penetration created by this substep's motion is resolved in full; overlap that already existed leaks out at no more than 6 m/s; and contacts may stop a body but never push it apart faster than that |
| Stiff position-based joints barely stretch, so a stretch threshold never fails them. The tower acted as a rigid lever over the cheese edge | **Joints break by force**: the constraint multiplier over h² is the force carried, and capacity is seeded as a multiple of the lighter layer's weight, so a joint means the same at 5 layers or 77 |
| Concrete hitting cheese should not bounce | **Crushing energy loss**: a hard push-out keeps 82% of velocity |
| Floors slid off the cheese like ice | **Friction against obstacles** (static and kinetic) |
| Starting before the floors were solved let the tower ghost through the cheese | **Play is held until the floor solve is ready**; scenarios with obstacles always simulate their layers, even at Potato |
| Precompute too slow for "every floor its own body" | **Adaptive substeps** (2–6 per step, by the fastest awake layer's speed versus slab thickness) and **a cached ground grid**: 77 floors 1.9 s → 0.52 s, with the cheese 3.3 s → 1.16 s (warm, `npm run bench`) |

### Tried, measured and not kept (yet)

- **Exact convex contacts (separating-axis test, single point or vertex
  manifold) with position-only correction.** They catch every crossing slab,
  but a pile of 77 slabs never came to rest: it jittered at 4–27 m/s for the
  whole 30 s and took 7–10 s. What's missing is a velocity pass (restitution
  and dynamic friction at velocity level, as in Müller et al. 2020). That pass
  is the next engine task; with it the pile can sleep, which should also cut
  most of the remaining solve time.
- **Sweep-and-prune over all floor pairs with sphere bounds.** Correct, but a
  slab's bounding sphere (44 m across) overlaps half the pile, so it was 3×
  slower. Tight bounding boxes (from the rotation matrix) are the way to do
  it, together with the velocity pass.

### Next optimisations, in expected order of payoff

1. A velocity-level contact pass, so piles settle and sleep. Today no layer
   ever sleeps; the whole 30 s is simulated for every floor.
2. Stream keyframes from the worker as they are computed, so playback can
   start immediately and "Solving floors…" disappears.
3. Island splitting across several workers (separate piles are independent).
4. The contact and joint loops in Rust/WASM SIMD or a WebGPU compute pass
   (ADR 0004 now has the profile to justify it).
