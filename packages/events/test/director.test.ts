import { ManualClock } from '@destruction-dynamics/core';
import { describe, expect, it } from 'vitest';
import deepEnd from '../../../scenarios/the-deep-end/event.json';
import {
  AuthoritySync,
  EventAuthority,
  EventDirector,
  EventRecorder,
  InvalidTransitionError,
  type EventDefinition,
  captureTimeline,
  parseEventDefinition,
  replay,
  stageWindows,
  validateEventDefinition,
} from '../src';

const def = parseEventDefinition(deepEnd);
const clone = (): EventDefinition => JSON.parse(JSON.stringify(def)) as EventDefinition;

/** Small synthetic event for precise timing assertions. */
const tiny = (): EventDefinition => ({
  schema: 'dd-event/1',
  id: 'tiny',
  version: 1,
  title: 'Tiny',
  environment: 'test',
  eventType: 'custom',
  bounds: { minEast: 0, maxEast: 10, minNorth: 0, maxNorth: 10 },
  preparation: { duration: 2, cues: [{ id: 'p0', at: 0.5, channel: 'audio', type: 'groan' }] },
  stages: [
    {
      id: 'a',
      state: 'INITIAL_FAILURE',
      duration: 1,
      cues: [{ id: 'a0', at: 0, channel: 'audio', type: 'x' }],
    },
    {
      id: 'b',
      state: 'ACTIVE_DESTRUCTION',
      duration: 2,
      cues: [{ id: 'b1', at: 1, channel: 'audio', type: 'x' }],
    },
    { id: 'c', state: 'AFTERMATH', duration: null, cues: [{ id: 'c0', at: 0, channel: 'world', type: 'x' }] },
  ],
  restoration: { mode: 'real-world-duration', hours: 24, transitionSeconds: 3 },
});

const run = (d: EventDirector, seconds: number, dt = 1 / 60) => {
  for (let t = 0; t < seconds - 1e-9; t += dt) d.update(dt);
};

describe('schema', () => {
  it('accepts the flagship definition', () => {
    expect(validateEventDefinition(deepEnd)).toEqual({ ok: true, errors: [] });
  });

  it('reports precise errors', () => {
    const bad = clone() as unknown as Record<string, unknown>;
    bad.schema = 'dd-event/0';
    (bad.stages as { state: string }[])[1]!.state = 'INITIAL_FAILURE';
    (bad.stages as { cues: { at: number }[] }[])[0]!.cues[0]!.at = 99;
    bad.restoration = { mode: 'never' };
    const r = validateEventDefinition(bad);
    expect(r.ok).toBe(false);
    expect(r.errors.join('\n')).toMatch(/schema must be/);
    expect(r.errors.join('\n')).toMatch(/must be < stage duration/);
    expect(r.errors.join('\n')).toMatch(/restoration.mode/);
    expect(validateEventDefinition(null).ok).toBe(false);
  });

  it('rejects out-of-order lifecycle stages and duplicate cue ids', () => {
    const d = tiny();
    d.stages = [d.stages[1]!, d.stages[0]!, d.stages[2]!];
    d.stages[2]!.cues.push({ id: 'a0', at: 1, channel: 'audio', type: 'dup' });
    const r = validateEventDefinition(d);
    expect(r.errors.join('\n')).toMatch(/out of lifecycle order/);
    expect(r.errors.join('\n')).toMatch(/not unique/);
  });

  it('computes stage windows', () => {
    expect(stageWindows(tiny()).map((w) => [w.state, w.start, w.end])).toEqual([
      ['INITIAL_FAILURE', 0, 1],
      ['ACTIVE_DESTRUCTION', 1, 3],
      ['AFTERMATH', 3, Infinity],
    ]);
  });
});

describe('lifecycle', () => {
  it('walks the full lifecycle in order', () => {
    const clock = new ManualClock(0);
    const d = new EventDirector(def, { clock, seed: 1 });
    const states: string[] = [d.state];
    d.on('state', (s) => states.push(s.to));
    d.arm();
    run(d, 60, 1 / 30);
    expect(d.state).toBe('AFTERMATH');
    clock.advance(24 * 3_600_000);
    d.update(0.1);
    expect(d.state).toBe('RESTORATION');
    run(d, 5);
    expect(states).toEqual([
      'INTACT',
      'PREPARING',
      'TRIGGERED',
      'INITIAL_FAILURE',
      'ACTIVE_DESTRUCTION',
      'WATER_IMPACT',
      'SETTLING',
      'AFTERMATH',
      'RESTORATION',
      'INTACT',
    ]);
  });

  it('cancels during preparation only', () => {
    const d = new EventDirector(tiny());
    expect(() => d.cancel()).toThrow(InvalidTransitionError);
    d.arm();
    run(d, 1);
    d.cancel();
    expect(d.state).toBe('INTACT');
    d.arm();
    run(d, 2.5);
    expect(d.committed).toBe(true);
    expect(() => d.cancel()).toThrow(/cannot cancel while/);
  });

  it('rejects invalid transitions', () => {
    const d = new EventDirector(tiny());
    expect(() => d.restoreNow()).toThrow(InvalidTransitionError);
    d.trigger();
    expect(() => d.arm()).toThrow(InvalidTransitionError);
    expect(() => d.trigger()).toThrow(InvalidTransitionError);
    expect(() => d.setTimeScale(-1)).toThrow(RangeError);
  });

  it('fires cues at their authored times regardless of frame step', () => {
    const record = (dt: number) => {
      const d = new EventDirector(tiny(), { seed: 5 });
      const cap = captureTimeline(d);
      d.arm();
      run(d, 6, dt);
      return cap.entries;
    };
    const a = record(1 / 60);
    const b = record(0.37);
    const c = record(5);
    expect(b).toEqual(a);
    expect(c).toEqual(a);
    expect(a.filter((e) => e.kind === 'cue').map((e) => [e.kind === 'cue' && e.id, e.time])).toEqual([
      ['p0', -1.5],
      ['a0', 0],
      ['b1', 2],
      ['c0', 3],
    ]);
  });

  it('cues at a stage start observe the new state', () => {
    const d = new EventDirector(tiny());
    const seen: string[] = [];
    d.on('cue', (c) => seen.push(`${c.cue.id}:${d.state}`));
    d.trigger();
    run(d, 4, 0.5);
    expect(seen).toEqual(['a0:INITIAL_FAILURE', 'b1:ACTIVE_DESTRUCTION', 'c0:AFTERMATH']);
  });

  it('pauses, resumes and scales time', () => {
    const d = new EventDirector(tiny());
    d.trigger();
    d.pause();
    d.update(10);
    expect(d.time).toBe(0);
    d.resume();
    d.setTimeScale(2);
    d.update(0.25);
    expect(d.time).toBeCloseTo(0.5);
    expect(d.stage?.progress).toBeCloseTo(0.5);
  });

  it('restarts and resets', () => {
    const d = new EventDirector(tiny());
    d.trigger();
    run(d, 2.5);
    d.restart();
    expect(d.state).toBe('INITIAL_FAILURE');
    expect(d.time).toBe(0);
    d.reset();
    expect(d.state).toBe('INTACT');
  });

  it('seeks for editor scrubbing without firing one-shot cues', () => {
    const d = new EventDirector(tiny());
    const fired: string[] = [];
    d.on('cue', (c) => fired.push(c.cue.id));
    d.seek(1.5);
    expect(d.state).toBe('ACTIVE_DESTRUCTION');
    d.seek(-1);
    expect(d.state).toBe('PREPARING');
    expect(fired).toEqual([]);
    d.seek(2.5, { fireCues: true });
    expect(fired).toEqual(['a0', 'b1']); // p0 (t = -1.5) lies before the seek origin
  });

  it('runs restoration on the real-world clock, defaulting to 24 h', () => {
    const clock = new ManualClock(1_000);
    const d = new EventDirector(tiny(), { clock });
    d.trigger();
    run(d, 3.5);
    expect(d.state).toBe('AFTERMATH');
    expect(d.restorationDueAtMs).toBe(1_000 + 24 * 3_600_000);
    clock.advance(23 * 3_600_000);
    d.update(1);
    expect(d.state).toBe('AFTERMATH');
    clock.advance(3_600_000);
    d.update(0.01);
    expect(d.state).toBe('RESTORATION');
    d.update(1.5);
    expect(d.restorationProgress).toBeCloseTo(0.5, 1);
    d.update(2);
    expect(d.state).toBe('INTACT');
  });

  it('supports event-duration and manual restoration', () => {
    const t = tiny();
    t.restoration = { mode: 'event-duration', seconds: 2, transitionSeconds: 0 };
    const d = new EventDirector(t);
    d.trigger();
    run(d, 4);
    expect(d.state).toBe('AFTERMATH');
    run(d, 1.5);
    expect(d.state).toBe('INTACT');

    const m = tiny();
    m.restoration = { mode: 'manual', transitionSeconds: 1 };
    const d2 = new EventDirector(m);
    d2.trigger();
    run(d2, 100, 0.5);
    expect(d2.state).toBe('AFTERMATH');
    d2.restoreNow();
    expect(d2.state).toBe('RESTORATION');
  });
});

describe('persistence', () => {
  it('saves and loads mid-event, including restoration timers', () => {
    const clock = new ManualClock(0);
    const a = new EventDirector(def, { clock, seed: 77 });
    a.trigger();
    run(a, 12.3, 0.1);
    const save = JSON.parse(JSON.stringify(a.save()));
    const b = new EventDirector(def, { clock });
    b.load(save);
    expect(b.state).toBe(a.state);
    expect(b.time).toBeCloseTo(a.time);
    expect(b.seed).toBe(77);

    run(a, 60, 0.5);
    const late = a.save();
    clock.advance(25 * 3_600_000);
    const c = new EventDirector(def, { clock });
    c.load(late);
    c.update(0.01);
    expect(c.state).toBe('RESTORATION');
  });

  it('refuses saves from another event or version', () => {
    const a = new EventDirector(def);
    const s = a.save();
    const other = clone();
    other.version = 2;
    expect(() => new EventDirector(other).load(s)).toThrow(/version/);
  });
});

describe('replay and seeds', () => {
  it('replays a recording deterministically', () => {
    const clock = new ManualClock(0);
    const d = new EventDirector(def, { seed: 1234, clock });
    const rec = new EventRecorder(d);
    const live = captureTimeline(d);
    d.arm();
    run(d, 3, 1 / 50);
    d.pause();
    d.update(1);
    d.resume();
    d.setTimeScale(3);
    run(d, 10, 1 / 30);
    d.seek(5);
    run(d, 2, 0.2);
    const recording = rec.finish();
    expect(replay(def, recording, new ManualClock(0))).toEqual(live.entries);
  });

  it('derives the same per-cue seeds for the same event seed', () => {
    const seeds = (seed: number) => {
      const d = new EventDirector(def, { seed });
      const s: number[] = [];
      d.on('cue', (c) => s.push(c.seed));
      d.trigger();
      run(d, 50, 0.5);
      return s;
    };
    expect(seeds(9)).toEqual(seeds(9));
    expect(seeds(9)).not.toEqual(seeds(10));
    expect(new EventDirector(def, { seed: 3 }).rng('spray').next()).toBe(
      new EventDirector(def, { seed: 3 }).rng('spray').next(),
    );
  });
});

describe('authority', () => {
  it('lets a late joiner reconstruct the canonical stage and seed', () => {
    const server = new ManualClock(10_000);
    const authority = new EventAuthority(def, server, 'match-1');
    authority.arm();
    server.advance(5_000);
    authority.tick();
    expect(authority.snapshot().phase).toBe('committed');
    expect(authority.snapshot().committedAtMs).toBe(15_000);

    // Host client has been watching since the start.
    const host = new EventDirector(def);
    const hostSync = new AuthoritySync(host);
    hostSync.apply(authority.snapshot(), server.now());
    for (let i = 0; i < 200; i++) {
      server.advance(100);
      host.update(0.1);
    }
    // Late joiner arrives 20 s after commit.
    const late = new EventDirector(def);
    new AuthoritySync(late).apply(authority.snapshot(), server.now());
    expect(late.state).toBe(host.state);
    expect(late.state).toBe('WATER_IMPACT');
    expect(late.time).toBeCloseTo(host.time, 3);
    expect(late.seed).toBe(authority.snapshot().seed);
  });

  it('ignores stale snapshots and follows cancellation and restoration', () => {
    const server = new ManualClock(0);
    const authority = new EventAuthority(def, server);
    const d = new EventDirector(def);
    const sync = new AuthoritySync(d);
    authority.arm();
    const armed = authority.snapshot();
    sync.apply(armed, server.now());
    expect(d.state).toBe('PREPARING');
    server.advance(1000);
    authority.cancel();
    sync.apply(authority.snapshot(), server.now());
    expect(d.state).toBe('INTACT');
    expect(sync.apply(armed, server.now())).toBe(false);

    authority.arm();
    authority.trigger();
    server.advance(24 * 3_600_000 + 60_000);
    authority.tick();
    expect(authority.snapshot().phase).toBe('restoring');
    sync.apply(authority.snapshot(), server.now());
    expect(d.state).toBe('RESTORATION');
    server.advance(5_000);
    authority.tick();
    const fresh = authority.snapshot();
    expect(fresh.phase).toBe('intact');
    sync.apply(fresh, server.now());
    expect(d.state).toBe('INTACT');
    expect(d.seed).toBe(fresh.seed);
  });
});
