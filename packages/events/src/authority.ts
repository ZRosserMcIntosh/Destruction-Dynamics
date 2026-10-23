// Server-authoritative event state.
//
// The server decides *whether* and *when* an event happens; it never streams
// shards or particles. A snapshot carries the canonical wall-clock instants
// (on the server's monotonic clock) plus the seed. Each client reconstructs
// the authored sequence locally by seeking its director to the matching event
// time, so a late joiner lands in the correct stage with the correct seed.

import { deriveSeed, type WallClock } from '@destruction-dynamics/core';
import type { EventDirector } from './director';
import { type EventDefinition, parseEventDefinition, stageWindows } from './schema';

export type AuthorityPhase = 'intact' | 'preparing' | 'committed' | 'restoring';

export interface AuthoritySnapshot {
  schema: 'dd-authority/1';
  eventId: string;
  definitionVersion: number;
  seed: number;
  phase: AuthorityPhase;
  /** Monotonically increasing; clients ignore snapshots older than the last applied. */
  sequence: number;
  preparingSinceMs: number | null;
  committedAtMs: number | null;
  restoreAtMs: number | null;
  restorationStartedAtMs: number | null;
}

/** Runs on the server (or a host client). `clock` must be monotonic. */
export class EventAuthority {
  private readonly def: EventDefinition;
  private snap: AuthoritySnapshot;
  private occurrence = 0;

  constructor(
    definition: EventDefinition | unknown,
    private readonly clock: WallClock,
    private readonly baseSeed: number | string = 'authority',
  ) {
    this.def = parseEventDefinition(definition);
    this.snap = {
      schema: 'dd-authority/1',
      eventId: this.def.id,
      definitionVersion: this.def.version,
      seed: this.nextSeed(),
      phase: 'intact',
      sequence: 0,
      preparingSinceMs: null,
      committedAtMs: null,
      restoreAtMs: null,
      restorationStartedAtMs: null,
    };
  }

  snapshot(): AuthoritySnapshot {
    return { ...this.snap };
  }

  arm(): void {
    if (this.snap.phase !== 'intact') throw new Error(`cannot arm while ${this.snap.phase}`);
    this.update({ phase: 'preparing', preparingSinceMs: this.clock.now() });
    if (this.def.preparation.duration === 0) this.trigger();
  }

  trigger(): void {
    if (this.snap.phase === 'committed' || this.snap.phase === 'restoring')
      throw new Error(`cannot trigger while ${this.snap.phase}`);
    const now = this.clock.now();
    const aftermathStart = stageWindows(this.def).at(-1)?.start ?? 0;
    const r = this.def.restoration;
    const restoreAtMs =
      r.mode === 'real-world-duration'
        ? now + aftermathStart * 1000 + r.hours * 3_600_000
        : r.mode === 'event-duration'
          ? now + (aftermathStart + r.seconds) * 1000
          : null;
    this.update({ phase: 'committed', committedAtMs: now, restoreAtMs });
  }

  cancel(): void {
    if (this.snap.phase !== 'preparing') throw new Error(`cannot cancel while ${this.snap.phase}`);
    this.update({ phase: 'intact', preparingSinceMs: null });
  }

  /** Call periodically: commits after preparation and runs restoration. */
  tick(): void {
    const now = this.clock.now();
    const s = this.snap;
    if (s.phase === 'preparing' && s.preparingSinceMs !== null) {
      if (now - s.preparingSinceMs >= this.def.preparation.duration * 1000) {
        const committedAtMs = s.preparingSinceMs + this.def.preparation.duration * 1000;
        this.trigger();
        this.update({ committedAtMs });
      }
    } else if (s.phase === 'committed' && s.restoreAtMs !== null && now >= s.restoreAtMs) {
      this.update({ phase: 'restoring', restorationStartedAtMs: s.restoreAtMs });
    } else if (s.phase === 'restoring' && s.restorationStartedAtMs !== null) {
      if (now - s.restorationStartedAtMs >= this.def.restoration.transitionSeconds * 1000) {
        this.update({
          phase: 'intact',
          seed: this.nextSeed(),
          preparingSinceMs: null,
          committedAtMs: null,
          restoreAtMs: null,
          restorationStartedAtMs: null,
        });
      }
    }
  }

  private nextSeed(): number {
    return deriveSeed(this.baseSeed, `${this.def.id}#${this.occurrence++}`);
  }

  private update(patch: Partial<AuthoritySnapshot>): void {
    this.snap = { ...this.snap, ...patch, sequence: this.snap.sequence + 1 };
  }
}

export interface SyncOptions {
  /** Event-time error (s) tolerated before the client re-seeks. */
  toleranceSeconds?: number;
}

/**
 * Applies an authority snapshot to a client director. Returns false when the
 * snapshot was stale (older sequence) and ignored.
 */
export class AuthoritySync {
  private lastSequence = -1;

  constructor(
    private readonly director: EventDirector,
    private readonly opts: SyncOptions = {},
  ) {
    director.followsAuthority = true;
    director.setTimeScale(1);
  }

  apply(s: AuthoritySnapshot, serverNowMs: number): boolean {
    const d = this.director;
    if (s.eventId !== d.definition.id || s.definitionVersion !== d.definition.version)
      throw new Error(`snapshot for ${s.eventId}@${s.definitionVersion} does not match the definition`);
    if (s.sequence < this.lastSequence) return false;
    this.lastSequence = s.sequence;
    const tol = this.opts.toleranceSeconds ?? 0.25;

    if (s.phase === 'intact') {
      if (d.state !== 'INTACT') d.reset();
      if (d.seed !== s.seed) d.setSeed(s.seed);
      return true;
    }
    if (d.state === 'INTACT' && d.seed !== s.seed) d.setSeed(s.seed);

    if (s.phase === 'restoring' && s.restorationStartedAtMs !== null) {
      const dur = d.definition.restoration.transitionSeconds;
      const progress = dur > 0 ? (serverNowMs - s.restorationStartedAtMs) / 1000 / dur : 1;
      d.forceRestoration(progress, 'authority');
      return true;
    }

    const prep = d.definition.preparation.duration;
    const t =
      s.phase === 'preparing' && s.preparingSinceMs !== null
        ? -prep + (serverNowMs - s.preparingSinceMs) / 1000
        : s.committedAtMs !== null
          ? (serverNowMs - s.committedAtMs) / 1000
          : d.time;
    const wantCommitted = s.phase === 'committed';
    const stateMismatch =
      wantCommitted !== d.committed || (s.phase === 'preparing' && d.state !== 'PREPARING');
    if (stateMismatch || Math.abs(d.time - t) > tol) d.seekInternal(t, false, 'authority');
    return true;
  }
}
