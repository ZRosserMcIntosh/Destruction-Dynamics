import { describe, expect, it } from 'vitest';
import { AudioCueRouter, NullAudioEngine } from '../src';

const fired = (type: string, params: Record<string, unknown> = {}, lateBy = 0, channel = 'audio') => ({
  cue: { id: `c-${type}`, channel, type, params },
  seed: 7,
  lateBy,
});

describe('AudioCueRouter', () => {
  it('routes audio cues with params and a default position', () => {
    const engine = new NullAudioEngine();
    const r = new AudioCueRouter(engine, { defaultPosition: () => ({ x: 1, y: 2, z: 3 }) });
    r.handle(fired('groan', { gain: 0.5, duration: 4 }));
    r.handle(fired('glass', { position: { x: 9, y: 9, z: 9 } }));
    expect(engine.played).toEqual([
      { type: 'groan', gain: 0.5, duration: 4, position: { x: 1, y: 2, z: 3 }, seed: 7 },
      { type: 'glass', gain: 1, position: { x: 9, y: 9, z: 9 }, seed: 7 },
    ]);
  });

  it('ignores other channels and drops unknown or stale cues', () => {
    const engine = new NullAudioEngine();
    const r = new AudioCueRouter(engine, { defaultPosition: () => ({ x: 0, y: 0, z: 0 }), maxLateness: 0.5 });
    expect(r.handle(fired('shot', {}, 0, 'camera'))).toBeNull();
    expect(r.handle(fired('kazoo'))).toBeNull();
    expect(r.handle(fired('rumble', {}, 3))).toBeNull();
    expect(engine.played).toHaveLength(0);
    expect(r.dropped).toBe(2);
  });
});
