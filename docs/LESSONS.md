# Viewer challenges → engine lessons

The rule: every absurd challenge a subscriber sets gets built, and whatever it
teaches goes into the engine, not into a one-off scenario hack. This ledger
records each challenge and what the engine gained from it.

## Day 2: Impact Lab (car into a wall, everyday objects into panels)

Proposed on the channel as the follow-up to Day 1: crash a test car into a
brick wall, and fire different things at different materials. Built as a
separate app, `apps/impact-lab`. The cheese is back as an indestructible wall
and target.

| Lesson | What changed in the engine |
|---|---|
| Things must break where they are hit, not along pre-cut seams | **Support graph + Voronoi shatter at the impact point** (`BondGraph`, `voronoiBox`): joints with capacity and accumulated impulse damage; boxes shatter radially (plates) or around the hit (blocks), cells inherit joints across the faces they touch (ADR 0010) |
| A rigid solver stops a fast car or ball dead against a fixed wall before the wall can break | **Momentum sharing on break-through**: when an impact frees pieces, the impactor and the freed pieces leave together with the impactor's pre-step momentum |
| Against a rigid wall a car's front crushes at full speed, so at 120 km/h it reached the cabin before the wall failed, and the wall felt the same force at every speed | **Riera crush model**: the wall feels the crush force plus μv², the momentum flux of the front coming to rest, so fast hits load it much harder |
| Damage centred on one point popped single bricks but the ring of joints around the car's footprint held | **A wide impactor strikes every piece it touches**, each on its own joints, applied as one batch with a single rebuild |
| Bricks only fell where they were hit, leaving a cartoon car-shaped hole | **Gravity sag**: pieces with nothing below wear down their remaining joints by weight, giving the stepped arch of real masonry |
| A small, fast golf ball must not pass through 6 mm glass | **Continuous collision** on projectiles; a test fires one at 80 m/s into plywood |
| Hundreds of brick chips colliding with each other made the solver the bottleneck | **Chip collision group**: loose pieces under 6 cm only collide with the ground and impactors |
| Every contact event recomputed every pose in a 700-brick wall (0.5 ms per event) | **Fixed bodies skip pose refresh**, the impact search uses an index queue and stamp arrays instead of `shift()` and Sets: about 0.1 ms per event |
| Rapier panics (WASM `unreachable`) when a body removed by a fracture is read later in the same step | **Check bodies still exist** before reading them across a rebuild |

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
| Concrete hitting cheese should not bounce | ~~Crushing energy loss (a hard push-out keeps 82% of velocity)~~, replaced by the **velocity contact pass** below |
| Floors slid off the cheese like ice | **Friction against obstacles** (static and kinetic) |
| Starting before the floors were solved let the tower ghost through the cheese | **Play waits for the first streamed chunk of the floor solve** (about 30 ms, see below); scenarios with obstacles always simulate their layers, even at Potato |
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

### Follow-up: settling and streaming (optimisations 1 and 2)

| Problem | Engine change |
|---|---|
| Piles jittered forever because position-only contacts leave random velocity behind | **Velocity contact pass** (Müller et al. 2020): every contact (ground, obstacles, neighbouring layers) is recorded with its normal multiplier; after the position solve, restitution (0.1, only above a 2gh threshold so resting contacts do not bounce) and dynamic friction (μ 0.8, bounded by the normal impulse) are applied as velocity impulses. The ad-hoc crush damping and separation clamp were deleted |
| Bodies drifting in free fall must never be frozen | **Contact-based sleep**: a layer sleeps only while touching something, below 0.3 m/s and 0.06 rad/s, for 1 s |
| "Solving floors…" wait before Play (up to 1.1 s at 77 floors with the cheese) | **Streamed keyframes**: the worker posts 0.5 s chunks as they are solved; the tower plays from the first chunk and holds at the newest keyframe if playback ever catches up. Time to first chunk: 9–29 ms. The solve runs 27–300× faster than real time, so playback never catches up in practice |

Honest limits: a few layers on the seabed slope still creep at 0.1–0.3 m/s,
so not every pile reaches full sleep within the 30 s window, and raw solve
time is about the same as before (77 floors 0.70 s, with the cheese 1.13 s).
The speed win the viewer sees comes from streaming.

### Next optimisations, in expected order of payoff

1. Island splitting across several workers (separate piles are independent).
2. The contact and joint loops in Rust/WASM SIMD or a WebGPU compute pass
   (ADR 0004 now has the profile to justify it).
3. Static friction on the seabed slope so creeping layers can sleep too.
