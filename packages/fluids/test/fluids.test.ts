import { quatIdentity, vec3 } from '@destruction-dynamics/core';
import { RapierPhysicsAdapter } from '@destruction-dynamics/physics';
import { describe, expect, it } from 'vitest';
import {
  FarFieldOcean,
  type FluidAdapter,
  HybridOcean,
  NearFieldSolver,
  StillWater,
  computeBuoyancy,
  makeWaves,
} from '../src';

const flatDepth = () => 20;

describe('far-field Gerstner ocean', () => {
  it('samples the height of the displaced surface point', () => {
    const ocean = new FarFieldOcean(makeWaves('moderate', 0.3, 4));
    const t = 12.5;
    for (const [x0, z0] of [
      [0, 0],
      [13, -40],
      [250, 90],
    ] as const) {
      const d = ocean.displace(x0, z0, t);
      expect(ocean.heightAt(x0 + d.dx, z0 + d.dz, t)).toBeCloseTo(d.dy, 2);
    }
  });

  it('is deterministic per seed and bounded by its amplitudes', () => {
    const a = makeWaves('rough', 1, 9);
    expect(makeWaves('rough', 1, 9)).toEqual(a);
    const ocean = new FarFieldOcean(a);
    const bound = a.reduce((s, w) => s + w.amplitude, 0);
    for (let i = 0; i < 200; i++)
      expect(Math.abs(ocean.heightAt(i * 7.3, i * -3.1, i * 0.37))).toBeLessThanOrEqual(bound + 1e-6);
  });
});

describe('near-field shallow-water solver', () => {
  const make = (depth: (x: number, z: number) => number = flatDepth, size = 128) =>
    new NearFieldSolver({ size, extent: 512, centerX: 0, centerZ: 0, depth, damping: 0 });

  it('conserves volume while waves are away from the sponge', () => {
    const s = make();
    s.eta[64 * 128 + 64] = 5;
    s.eta[64 * 128 + 65] = 5;
    const v0 = s.volume();
    for (let i = 0; i < 40; i++) s.step(1 / 30);
    expect(s.volume()).toBeCloseTo(v0, 0);
  });

  it('propagates a disturbance at roughly the shallow-water speed √(gD)', () => {
    const s = make();
    s.addPressureDisc(0, 0, 12, 6);
    for (let i = 0; i < 3; i++) s.step(1 / 30);
    for (let i = 0; i < 147; i++) s.step(1 / 30);
    // After ~5 s the front should be near r = 5·√(9.81·20) ≈ 70 m.
    let best = 0,
      bestR = 0;
    for (let r = 20; r < 200; r += 2) {
      const h = Math.abs(s.sampleField(s.eta, r, 0));
      if (h > best) {
        best = h;
        bestR = r;
      }
    }
    expect(bestR).toBeGreaterThan(50);
    expect(bestR).toBeLessThan(95);
  });

  it('stays stable for long runs with large steps and treats land as a wall', () => {
    const s = make((x) => (x < -100 ? -2 : 30));
    s.addPressureBox(50, 0, 120, 15, 0.3, 14);
    for (let i = 0; i < 600; i++) s.step(0.1);
    expect(Number.isFinite(s.maxAbsHeight())).toBe(true);
    expect(s.maxAbsHeight()).toBeLessThan(20);
    expect(s.sampleField(s.eta, -200, 0)).toBe(0);
    expect(s.lastSubsteps).toBeGreaterThanOrEqual(1);
  });

  it('absorbs waves at the sponge boundary', () => {
    const s = new NearFieldSolver({
      size: 96,
      extent: 384,
      centerX: 0,
      centerZ: 0,
      depth: flatDepth,
      damping: 0.02,
    });
    s.addPressureDisc(0, 0, 15, 8);
    s.step(0.1);
    for (let i = 0; i < 1200; i++) s.step(0.1);
    expect(s.maxAbsHeight()).toBeLessThan(0.2);
  });
});

const adapters: [string, () => FluidAdapter][] = [
  ['still-water', () => new StillWater(0)],
  [
    'hybrid-ocean',
    () =>
      new HybridOcean({
        waves: makeWaves('calm', 0, 1),
        nearField: { size: 96, extent: 480, centerX: 0, centerZ: 0 },
        depth: flatDepth,
      }),
  ],
];

describe.each(adapters)('FluidAdapter contract: %s', (_n, make) => {
  it('returns finite samples with an upward unit normal near sea level', () => {
    const f = make();
    f.step(0.5);
    const s = f.sample(10, -20);
    expect(Number.isFinite(s.height)).toBe(true);
    expect(Math.abs(s.height - f.seaLevel)).toBeLessThan(3);
    expect(s.normal.y).toBeGreaterThan(0.5);
    expect(Math.hypot(s.normal.x, s.normal.y, s.normal.z)).toBeCloseTo(1, 5);
    expect(f.heightAt(10, -20)).toBeCloseTo(s.height, 6);
  });

  it('accepts impacts and disturbances and resets', () => {
    const f = make();
    f.applyImpact({
      position: vec3(0, 0, 0),
      footprint: { kind: 'box', halfLength: 100, halfWidth: 15, yaw: 0 },
      intensity: 1,
      duration: 1.5,
      direction: vec3(1, 0, 0),
      visualProfile: 'landmark-collapse',
    });
    f.applyDisturbance(vec3(30, 0, 30), 3, 1);
    for (let i = 0; i < 30; i++) f.step(1 / 30);
    f.reset();
    expect(f.time).toBe(0);
    expect(f.heightAt(0, 0)).toBeCloseTo(f.seaLevel, 0);
  });

  it('advances time monotonically', () => {
    const f = make();
    f.step(0.25);
    f.step(0.25);
    expect(f.time).toBeCloseTo(0.5);
  });
});

describe('hybrid ocean impact response', () => {
  it('produces a large surge that spreads and decays', () => {
    const o = new HybridOcean({
      waves: makeWaves('calm', 0, 1),
      nearField: { size: 128, extent: 640, centerX: 0, centerZ: 0 },
      depth: () => 35,
    });
    o.applyImpact({
      position: vec3(0, 0, 0),
      footprint: { kind: 'box', halfLength: 120, halfWidth: 15, yaw: 0 },
      intensity: 1,
      duration: 1.6,
      direction: vec3(1, 0, 0),
      visualProfile: 'landmark-collapse',
    });
    let peak = 0;
    for (let i = 0; i < 90; i++) {
      o.step(1 / 30);
      peak = Math.max(peak, o.maxDisturbance());
    }
    expect(peak).toBeGreaterThan(4);
    // Broadside, 100 m off the long axis, the surge has arrived and the water moves.
    let moved = 0;
    for (let i = 0; i < 90; i++) {
      o.step(1 / 30);
      moved = Math.max(moved, Math.abs(o.heightAt(0, 100) - o.far.heightAt(0, 100, o.time)));
    }
    expect(moved).toBeGreaterThan(1);
    for (let i = 0; i < 1800; i++) o.step(1 / 15);
    expect(o.maxDisturbance()).toBeLessThan(peak * 0.2);
  });

  it('resizes its grid for quality changes', () => {
    const o = new HybridOcean({
      waves: [],
      nearField: { size: 64, extent: 320, centerX: 0, centerZ: 0 },
      depth: flatDepth,
    });
    o.resize(96);
    expect(o.near.n).toBe(96);
    expect(o.composite.length).toBe(96 * 96 * 2);
  });

  it('skips solver steps when degraded but keeps time', () => {
    const o = new HybridOcean({
      waves: [],
      nearField: { size: 64, extent: 320, centerX: 0, centerZ: 0 },
      depth: flatDepth,
    });
    o.stepDivisor = 3;
    o.applyDisturbance(vec3(0, 0, 0), 10, 5);
    o.step(0.05);
    expect(o.near.maxAbsHeight()).toBe(0);
    o.step(0.05);
    o.step(0.05);
    expect(o.time).toBeCloseTo(0.15);
  });
});

describe('buoyancy', () => {
  it('floats a half-density box about half submerged (Rapier)', async () => {
    const water = new StillWater(0);
    const p = new RapierPhysicsAdapter();
    await p.init();
    const he = vec3(1, 1, 1);
    const h = p.createBody({
      kind: 'dynamic',
      shape: { type: 'box', halfExtents: he },
      pose: { position: vec3(0, 3, 0), quaternion: quatIdentity() },
      density: 512,
    });
    for (let i = 0; i < 900; i++) {
      const b = computeBuoyancy(
        { pose: p.getPose(h), halfExtents: he, velocity: p.getLinearVelocity(h), mass: p.getMass(h) },
        water,
        { resolution: 3, drag: 1.5 },
      );
      for (const f of b.forces) p.addForce(h, f.force, f.point);
      p.step(1 / 60);
    }
    expect(p.getPose(h).position.y).toBeGreaterThan(-0.15);
    expect(p.getPose(h).position.y).toBeLessThan(0.15);
    p.dispose();
  });

  it('sinks a dense box and leaves an emerged one unforced', () => {
    const water = new StillWater(0);
    const high = computeBuoyancy(
      {
        pose: { position: vec3(0, 10, 0), quaternion: quatIdentity() },
        halfExtents: vec3(1, 1, 1),
        velocity: vec3(),
        mass: 100,
      },
      water,
    );
    expect(high.forces).toHaveLength(0);
    const deep = computeBuoyancy(
      {
        pose: { position: vec3(0, -10, 0), quaternion: quatIdentity() },
        halfExtents: vec3(1, 1, 1),
        velocity: vec3(),
        mass: 20000,
      },
      water,
    );
    const lift = deep.forces.reduce((s, f) => s + f.force.y, 0);
    expect(deep.submergedFraction).toBe(1);
    expect(lift).toBeCloseTo(1025 * 9.81 * 8, -1);
    expect(lift).toBeLessThan(20000 * 9.81);
  });
});
