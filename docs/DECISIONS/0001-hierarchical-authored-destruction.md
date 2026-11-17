# ADR 0001: Landmark destruction is authored motion with bounded runtime physics

Status: accepted (Milestone 3)

## Context

The flagship event is an 85-storey tower tipping into the sea. It has to be
spectacular, look the same for every player, run on modest GPUs, work with
scrubbing, save/load and late joining, and land on the water when the
director says so. An unconstrained rigid-body collapse meets none of these:
the outcome changes from run to run, the cost spikes, and nobody can
art-direct it.

## Decision

- **Macro.** Large structural sections follow motion that is a pure function
  of event time and seed (`TowerCollapse.posesAt`). The tip uses an
  inverted-pendulum profile, time-normalised to the authored stage
  durations. Relative bends add lag and whip. Seeded wreck poses sit on the
  seabed.
- **Meso.** Facade panels and chunks detach on a seeded schedule
  (`FacadePlan`) and become real rigid bodies through `DebrisManager`, which
  enforces a hard budget.
- **Micro.** Glass, dust, sparks, spray and mist are particles.
- **Collision** follows the macro motion through kinematic bodies, so the
  falling tower still pushes debris and players.

## Consequences

- Timing is exact, and seek, replay, save/load and multiplayer
  reconstruction come for free.
- Debris trajectories are not deterministic across clients and must not
  affect gameplay (see ADR 0006).
- The macro motion does not respond to gameplay forces. A player cannot prop
  the tower up. This is intended.
- New landmarks need authored data (sections, timing, wreck), not new code,
  as long as they fit the section-chain model. The bridge (Milestone 8)
  tests that claim.
