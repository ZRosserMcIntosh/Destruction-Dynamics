# ADR 0010: Breakable structures as a support graph over Rapier bodies

Status: accepted (2026-10-27). Built for the Impact Lab (`apps/impact-lab`).

## Context

The floor-layer solver (ADR 0009) breaks a tower along pre-cut seams. The
Impact Lab needs things to break where they are hit: a car into a brick wall,
a bowling ball through a window. Pieces must become real rigid bodies once
loose, and the standing part must stay cheap.

## Decision

`@destruction-dynamics/destruction` gets a `fracture` module:

- **`BondGraph`** (no physics dependency): pieces (boxes, or convex shards
  made at run time) joined by joints with a capacity (strength × joint area),
  and joints to the world (foundations, frames). Impacts load the joints
  around the point hit with Gaussian weights and accumulate as damage, in the
  spirit of NVIDIA Blast's support graph and impulse damage. A box hit with
  more than `toughness × t²` in one step (t its thinnest dimension: plates
  punch through with thickness squared) shatters into **Voronoi cells around
  the impact**: radial in the mid-plane for plates, clustered towards the hit
  for blocks. Cells inherit the box's joints across the faces they touch;
  inside a punch radius that grows with the overload they come away.
- **`voronoiBox`**: cells by plane intersection (box half-spaces plus one
  bisector per other seed), with exact volumes, shared-face areas and the
  part of each box face every cell covers. No mesh clipping, no dependency.
- **`FractureWorld`** binds the graph to Rapier: one rigid body per island,
  fixed while joined to the world, dynamic once not. Contact force events
  above 20 body weights become impacts. Changed islands keep their body when
  they can (colliders added and removed) instead of being rebuilt.
- **Breaking through**: a rigid solver stops an impactor against a fixed
  body in one step, before the wall has had a chance to break. When an impact
  frees pieces near the contact, the impactor's pre-step momentum is shared
  with them (perfectly inelastic).
- **Gravity sag**: a fixed piece with nothing under it wears its remaining
  joints down by its weight. Over a hole in running bond this peels away the
  units without a supported unit beneath, leaving the stepped arch real
  masonry shows; stack-bonded walls drop whole columns.
- **`CrashCar`**: a kinematic car driven by a one-dimensional crush model
  (force rising to a plateau, then the cabin), loading the wall with the crush
  force plus the momentum flux μv² of the front coming to rest (Riera, Nucl.
  Eng. Des. 8, 1968), on every piece the bumper touches. Belted dummies with
  airbags ride on the car's deceleration.
- **Collision groups**: loose single pieces under 6 cm across are chips; they
  only collide with the ground and impactors. This removed most contact pairs
  in a fresh debris field.

The materials (`FRACTURE_MATERIALS`) have real densities and strengths
tuned so outcomes look right. They are not engineering data.

## Consequences

- Fixed step: shattering compares one step's impulse, so the world must be
  stepped at `FractureWorld.STEP` (1/120 s). The lab interpolates poses
  between steps, so slow motion stays smooth.
- Rapier is now a direct dependency of `destruction` (the adapter of ADR 0003
  has no compound bodies, contact events or collision groups). The floor
  layers and debris still use the adapter.
- Measured in Node (2.1 GHz Xeon vCPU, one core): a 6 m brick wall (756
  units) hit at 120 km/h simulates 4 s in 2–3 s, with a worst step around
  100 ms when hundreds of bricks come loose at once. In the browser the
  impact plays in slow motion, which hides most of it. Spreading the break
  over a few steps, and island sleeping, are the next things to try.
- Rapier panics (WASM `unreachable`) if a removed body is touched. Anything
  that holds a body across a step must check it still exists.
