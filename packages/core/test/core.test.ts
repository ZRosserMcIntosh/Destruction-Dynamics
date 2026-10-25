import { describe, expect, it } from 'vitest';
import {
  BudgetGovernor,
  CountBudget,
  DEG,
  InMemoryTileHost,
  ManualClock,
  QUALITY_PRESETS,
  QUALITY_PRESET_IDS,
  SeededRandom,
  WorldStateStore,
  deriveSeed,
  enuFromScene,
  quatFromAxisAngle,
  quatRotate,
  sceneFromEnu,
  tileAt,
  tileKey,
  tilesForBounds,
  vec3,
} from '../src';

describe('SeededRandom', () => {
  it('is reproducible for the same seed and differs across seeds', () => {
    const a = new SeededRandom(42);
    const b = new SeededRandom(42);
    const c = new SeededRandom(43);
    const sa = Array.from({ length: 50 }, () => a.next());
    const sb = Array.from({ length: 50 }, () => b.next());
    const sc = Array.from({ length: 50 }, () => c.next());
    expect(sa).toEqual(sb);
    expect(sa).not.toEqual(sc);
    expect(sa.every((v) => v >= 0 && v < 1)).toBe(true);
  });

  it('derives independent named streams', () => {
    expect(deriveSeed(7, 'spray')).toBe(deriveSeed(7, 'spray'));
    expect(deriveSeed(7, 'spray')).not.toBe(deriveSeed(7, 'dust'));
    expect(deriveSeed('the-deep-end', 'x')).toBe(deriveSeed('the-deep-end', 'x'));
  });

  it('forks without advancing the parent', () => {
    const a = new SeededRandom(1);
    const b = new SeededRandom(1);
    a.fork('child').next();
    expect(a.next()).toBe(b.next());
  });

  it('has a roughly uniform mean', () => {
    const r = new SeededRandom(9);
    let s = 0;
    for (let i = 0; i < 20000; i++) s += r.next();
    expect(s / 20000).toBeCloseTo(0.5, 1);
  });
});

describe('frames', () => {
  it('round-trips ENU and scene frames', () => {
    const p = { east: 10, north: 20, up: 3 };
    expect(sceneFromEnu(p)).toEqual({ x: 10, y: 3, z: -20 });
    expect(enuFromScene(sceneFromEnu(p))).toEqual(p);
  });

  it('rotates vectors with quaternions', () => {
    const q = quatFromAxisAngle(vec3(0, 0, 1), 90 * DEG);
    const v = quatRotate(q, vec3(1, 0, 0));
    expect(v.x).toBeCloseTo(0);
    expect(v.y).toBeCloseTo(1);
  });
});

describe('tiles', () => {
  it('indexes 500 m tiles centred on multiples of the size', () => {
    expect(tileAt(0, 0)).toEqual({ i: 0, j: 0 });
    expect(tileAt(249, -249)).toEqual({ i: 0, j: 0 });
    expect(tileAt(251, 760)).toEqual({ i: 1, j: 2 });
    expect(tileKey({ i: -3, j: 4 })).toBe('T_-3_4');
  });

  it('finds every tile an event footprint crosses', () => {
    const tiles = tilesForBounds({ minEast: -100, maxEast: 400, minNorth: -50, maxNorth: 50 });
    expect(tiles.map(tileKey).sort()).toEqual(['T_0_0', 'T_1_0']);
  });

  it('reference-counts activation leases', () => {
    const host = new InMemoryTileHost();
    const a = host.acquire([{ i: 0, j: 0 }], 'event');
    const b = host.acquire(
      [
        { i: 0, j: 0 },
        { i: 1, j: 0 },
      ],
      'player',
    );
    expect(host.activeTiles()).toEqual(['T_0_0', 'T_1_0']);
    a.release();
    a.release();
    expect(host.isActive({ i: 0, j: 0 })).toBe(true);
    b.release();
    expect(host.activeTiles()).toEqual([]);
  });

  it('saves, restores and expires world-state overrides', () => {
    const clock = new ManualClock(1000);
    const store = new WorldStateStore();
    store.set(
      'the-deep-end',
      [
        { i: 0, j: 0 },
        { i: 1, j: 0 },
      ],
      'wreck',
      clock.now() + 5000,
    );
    store.set('other', [{ i: 0, j: 0 }], 'burnt', null);
    const loaded = WorldStateStore.fromJSON(JSON.parse(JSON.stringify(store.toJSON())));
    expect(loaded.forTile({ i: 1, j: 0 })).toEqual([
      { eventId: 'the-deep-end', state: 'wreck', restoreAtMs: 6000 },
    ]);
    clock.advance(5000);
    expect(loaded.expire(clock.now())).toEqual(['the-deep-end']);
    expect(loaded.forTile({ i: 1, j: 0 })).toEqual([]);
    expect(loaded.forTile({ i: 0, j: 0 }).map((o) => o.eventId)).toEqual(['other']);
  });
});

describe('budgets', () => {
  it('degrades secondary work when frames run long and restores it with headroom', () => {
    const g = new BudgetGovernor({ targetFrameMs: 16.7, degradeAfter: 5, restoreAfter: 10 });
    for (let i = 0; i < 5; i++) g.observe(30);
    expect(g.current.level).toBe(1);
    for (let i = 0; i < 20; i++) g.observe(30);
    expect(g.current.level).toBe(3);
    expect(g.current.particleScale).toBeLessThan(0.5);
    for (let i = 0; i < 10; i++) g.observe(5);
    expect(g.current.level).toBe(2);
  });

  it('ignores isolated slow frames', () => {
    const g = new BudgetGovernor({ targetFrameMs: 16.7, degradeAfter: 5 });
    for (let i = 0; i < 50; i++) g.observe(i % 4 === 0 ? 40 : 15);
    expect(g.current.level).toBe(0);
  });

  it('caps counts', () => {
    const b = new CountBudget(10);
    expect(b.request(7)).toBe(7);
    expect(b.request(7)).toBe(3);
    b.release(5);
    expect(b.available).toBe(5);
  });

  it('gives every floor above the stump its own layer at the top preset', () => {
    expect(QUALITY_PRESETS.cinematic.structuralLayers).toBe(85 - 8);
    expect(QUALITY_PRESETS.dorito.layerSimulation).toBe(false);
  });

  it('presets scale real budgets monotonically', () => {
    const order = QUALITY_PRESET_IDS.map((id) => QUALITY_PRESETS[id]);
    for (let i = 1; i < order.length; i++) {
      const a = order[i - 1]!;
      const b = order[i]!;
      expect(b.maxParticles).toBeGreaterThan(a.maxParticles);
      expect(b.maxDebrisBodies).toBeGreaterThan(a.maxDebrisBodies);
      expect(b.fluidGridSize).toBeGreaterThan(a.fluidGridSize);
      expect(b.structuralLayers).toBeGreaterThan(a.structuralLayers);
    }
  });
});
