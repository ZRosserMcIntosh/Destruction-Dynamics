# ADR 0006: Server-authoritative event state; clients reconstruct the visuals

Status: accepted (Milestone 2). Transport is not implemented.

## Context

Online play is planned. Streaming thousands of shards, particles and water
cells is impossible and also unnecessary.

## Decision

- The server owns `EventAuthority`. It decides whether and when an event
  happens, its seed, and when restoration begins. Snapshots
  (`dd-authority/1`) carry canonical instants on the server's monotonic
  clock and a sequence number.
- Clients run their own director and `AuthoritySync`. They seek to the event
  time implied by the snapshot and the synchronised server time, ignore
  stale snapshots, and re-seek only when they drift past a tolerance.
- Everything gameplay depends on is a deterministic function of (event time,
  seed): stage, tower poses, collision mode, facade detach schedule, surge
  timing. Secondary physics and particles are local and cosmetic.

## Consequences

- Late joiners and reconnecting clients reach the correct state from a
  single snapshot. This is tested.
- Clock sync is a prerequisite and is not implemented yet.
- Gameplay code must never depend on debris trajectories.
