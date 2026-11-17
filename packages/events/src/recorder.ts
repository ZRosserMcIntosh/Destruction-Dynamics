// Event recording and deterministic replay of the authored timeline. A
// recording is the sequence of control operations (including every update's
// dt) applied to a director; replaying it against the same definition and
// seed reproduces the same state changes and cues at the same event times.

import type { WallClock } from '@destruction-dynamics/core';
import { type ControlOp, EventDirector, type FiredCue, type StateChange } from './director';
import type { EventDefinition } from './schema';

export interface EventRecording {
  schema: 'dd-recording/1';
  eventId: string;
  definitionVersion: number;
  seed: number;
  ops: ControlOp[];
}

export type TimelineEntry =
  | { kind: 'state'; from: string; to: string; time: number }
  | { kind: 'cue'; id: string; time: number; seed: number };

export class EventRecorder {
  private readonly ops: ControlOp[] = [];
  private readonly stop: () => void;
  private readonly seed: number;

  constructor(private readonly director: EventDirector) {
    this.seed = director.seed;
    this.stop = director.on('control', (op) => this.ops.push(op));
  }

  finish(): EventRecording {
    this.stop();
    return {
      schema: 'dd-recording/1',
      eventId: this.director.definition.id,
      definitionVersion: this.director.definition.version,
      seed: this.seed,
      ops: [...this.ops],
    };
  }
}

/** Collects the observable timeline of a director (for comparing runs). */
export function captureTimeline(director: EventDirector): { entries: TimelineEntry[]; stop: () => void } {
  const entries: TimelineEntry[] = [];
  const a = director.on('state', (s: StateChange) =>
    entries.push({ kind: 'state', from: s.from, to: s.to, time: s.time }),
  );
  const b = director.on('cue', (c: FiredCue) =>
    entries.push({ kind: 'cue', id: c.cue.id, time: c.time, seed: c.seed }),
  );
  return {
    entries,
    stop: () => {
      a();
      b();
    },
  };
}

export function applyOp(d: EventDirector, op: ControlOp): void {
  switch (op.op) {
    case 'arm':
      return d.arm();
    case 'trigger':
      return d.trigger();
    case 'cancel':
      return d.cancel();
    case 'pause':
      return d.pause();
    case 'resume':
      return d.resume();
    case 'reset':
      return d.reset();
    case 'restoreNow':
      return d.restoreNow();
    case 'timeScale':
      return d.setTimeScale(op.value);
    case 'seek':
      return d.seek(op.time, { fireCues: op.fireCues });
    case 'update':
      return d.update(op.dt);
  }
}

/** Replays a recording on a fresh director and returns its observable timeline. */
export function replay(definition: EventDefinition, rec: EventRecording, clock?: WallClock): TimelineEntry[] {
  if (rec.eventId !== definition.id || rec.definitionVersion !== definition.version)
    throw new Error('recording does not match the definition');
  const d = new EventDirector(definition, { seed: rec.seed, ...(clock ? { clock } : {}) });
  const cap = captureTimeline(d);
  for (const op of rec.ops) {
    // restart() is recorded as its reset+trigger parts, so ops map 1:1.
    applyOp(d, op);
  }
  cap.stop();
  return cap.entries;
}
