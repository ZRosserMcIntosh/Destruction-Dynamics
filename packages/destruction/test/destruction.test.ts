import { DEG, add, distance, quatIdentity, quatRotate, vec3 } from '@destruction-dynamics/core';
import { RapierPhysicsAdapter, SimplePhysicsAdapter } from '@destruction-dynamics/physics';
import { DEEP_END_LAYOUT, deepEndGroundHeight, deepEndTimeline } from '@destruction-dynamics/scenarios';
import { describe, expect, it } from 'vitest';
import {
  DEEP_END_TOWER,
  DebrisManager,
  TowerCollapse,
  TowerCollision,
  buildFacadePlan,
  panelsDetachingBetween,
  towerCollisionModeAt,
  towerSections,
} from '../src';

const timeline = deepEndTimeline();
const make = (seed = 1) =>
  new TowerCollapse(DEEP_END_TOWER, timeline, { groundHeight: deepEndGroundHeight, seed });

/** World position of a section's top centre. */
const topOf = (tower: TowerCollapse, k: number, t: number | null) => {
  const p = tower.posesAt(t)[k]!;
  return add(p.position, quatRotate(p.quaternion, vec3(0, tower.sections[k]!.height, 0)));
};

describe('tower layout', () => {
  it('splits 85 floors into a stump and tipping sections', () => {
    const s = towerSections(DEEP_END_TOWER);
    expect(s).toHaveLength(6);
    expect(s[0]!.stump).toBe(true);
    expect(s.reduce((n, x) => n + x.floorCount, 0)).toBe(85);
    expect(s.at(-1)!.firstFloor + s.at(-1)!.floorCount).toBe(85);
    const height = s.reduce((h, x) => h + x.height, 0);
    expect(height).toBeCloseTo(85 * 3.4);
  });
});

describe('authored collapse', () => {
  it('is upright and stacked while intact', () => {
    const tower = make();
    const poses = tower.posesAt(null);
    for (let k = 1; k < poses.length; k++) {
      const below = topOf(tower, k - 1, null);
      expect(distance(below, poses[k]!.position)).toBeLessThan(1e-6);
    }
    expect(tower.tiltAt(timeline.failureStart - 1)).toBe(0);
  });

  it('leans slightly during the initial failure, then tips with accelerating rate', () => {
    const tower = make();
    const lean = tower.tiltAt(timeline.tipStart - 0.001);
    expect(lean / DEG).toBeGreaterThan(1.5);
    expect(lean / DEG).toBeLessThan(4);
    const d = timeline.impact - timeline.tipStart;
    const a1 = tower.tiltAt(timeline.tipStart + 0.25 * d) - tower.tiltAt(timeline.tipStart);
    const a2 = tower.tiltAt(timeline.tipStart + d) - tower.tiltAt(timeline.tipStart + 0.75 * d);
    expect(a2).toBeGreaterThan(3 * a1);
    let prev = 0;
    for (let t = timeline.tipStart; t <= timeline.impact; t += 0.1) {
      const a = tower.tiltAt(t);
      expect(a).toBeGreaterThanOrEqual(prev - 1e-9);
      prev = a;
    }
    expect(tower.tiltAt(timeline.impact) / DEG).toBeCloseTo(88, 5);
  });

  it('moves continuously across stage boundaries', () => {
    const tower = make();
    const sample = (t: number) => tower.posesAt(t).map((p) => p.position);
    for (const b of [timeline.failureStart, timeline.tipStart, timeline.impact]) {
      const before = sample(b - 1e-4);
      const after = sample(b + 1e-4);
      before.forEach((p, k) => expect(distance(p, after[k]!)).toBeLessThan(0.05));
    }
  });

  it('brings the tower top down near the water at impact and over the ocean', () => {
    const tower = make();
    const top = topOf(tower, tower.sections.length - 1, timeline.impact);
    expect(top.x).toBeGreaterThan(DEEP_END_LAYOUT.shoreX + 100);
    expect(top.y).toBeLessThan(25);
    expect(top.y).toBeGreaterThan(-5);
  });

  it('settles into a mostly submerged wreck on the seabed', () => {
    const tower = make();
    const settled = tower.posesAt(timeline.settleEnd + 10);
    const wreck = tower.wreckPoses();
    settled.forEach((p, k) => expect(distance(p.position, wreck[k]!.position)).toBeLessThan(1e-3));
    const deep = wreck.slice(1).filter((p) => p.position.x > 150);
    expect(deep.length).toBeGreaterThanOrEqual(2);
    // Over deep water the whole section (its top face, halfW above the origin... minus) is below sea level.
    for (const p of deep) expect(p.position.y + DEEP_END_TOWER.width / 2).toBeLessThan(0);
  });

  it('rewinds to intact during restoration', () => {
    const tower = make();
    const start = tower.posesAt(0, 0);
    const end = tower.posesAt(0, 1);
    const wreck = tower.posesAt(timeline.settleEnd);
    start.forEach((p, k) => expect(distance(p.position, wreck[k]!.position)).toBeLessThan(1e-3));
    end.forEach((p, k) => expect(distance(p.position, tower.posesAt(null)[k]!.position)).toBeLessThan(1e-3));
  });

  it('is deterministic per seed and varies across seeds', () => {
    expect(make(4).wreckPoses()).toEqual(make(4).wreckPoses());
    expect(make(4).wreckPoses()).not.toEqual(make(5).wreckPoses());
  });

  it('reports an impact footprint over the water along the fall direction', () => {
    const f = make().impactFootprint();
    expect(f.position.x - f.halfLength).toBeGreaterThanOrEqual(DEEP_END_LAYOUT.shoreX - 2);
    expect(f.halfLength).toBeGreaterThan(60);
    expect(f.direction.x).toBeGreaterThan(0.9);
  });

  it('rejects inconsistent timelines', () => {
    expect(
      () =>
        new TowerCollapse(
          DEEP_END_TOWER,
          { failureStart: 5, tipStart: 4, impact: 10, settleEnd: 20 },
          {
            groundHeight: () => 0,
            seed: 1,
          },
        ),
    ).toThrow();
  });
});

describe('facade plan', () => {
  const sections = towerSections(DEEP_END_TOWER);
  const plan = buildFacadePlan(DEEP_END_TOWER, sections, timeline, 6, 42);

  it('has one entry per panel and is deterministic', () => {
    expect(plan.panels).toHaveLength(85 * 4 * 6);
    expect(buildFacadePlan(DEEP_END_TOWER, sections, timeline, 6, 42).detachTime).toEqual(plan.detachTime);
  });

  it('sheds panels near the break during the initial failure and most at or after the tip', () => {
    const early = panelsDetachingBetween(plan, timeline.failureStart, timeline.tipStart);
    expect(early.length).toBeGreaterThan(30);
    expect(
      early
        .filter((i) => plan.panels[i]!.section > 0)
        .every((i) => Math.abs(plan.panels[i]!.floor - DEEP_END_TOWER.breakFloor) <= 3),
    ).toBe(true);
    const detached = plan.detachTime.filter((t) => Number.isFinite(t)).length;
    expect(detached / plan.panels.length).toBeGreaterThan(0.4);
    expect(detached / plan.panels.length).toBeLessThan(0.95);
  });

  it('answers range queries exactly like a brute-force scan', () => {
    for (const [a, b] of [
      [0, 3],
      [7, 18.5],
      [18, 20],
      [-10, 100],
    ] as const) {
      const brute = [...plan.detachTime.keys()].filter(
        (i) => plan.detachTime[i]! >= a && plan.detachTime[i]! < b,
      );
      expect(panelsDetachingBetween(plan, a, b).sort((x, y) => x - y)).toEqual(brute);
    }
  });
});

describe('debris budget', () => {
  const spawn = (y = 5) => ({
    kind: 'panel' as const,
    pose: { position: vec3(0, y, 0), quaternion: quatIdentity() },
    halfExtents: vec3(0.8, 0.6, 0.1),
    velocity: vec3(1, 0, 0),
    angularVelocity: vec3(),
    lifetime: 5,
    density: 800,
  });

  it('never exceeds its budget and refuses young pieces', async () => {
    const p = new SimplePhysicsAdapter();
    const d = new DebrisManager(p, { maxBodies: 10, minAgeForEviction: 1 });
    for (let i = 0; i < 25; i++) d.spawn(spawn());
    expect(d.active).toHaveLength(10);
    expect(d.stats.refused).toBe(15);
    expect(p.counts().dynamic).toBe(10);
  });

  it('evicts old pieces, expires by lifetime and shrinks with the limit', () => {
    const p = new SimplePhysicsAdapter();
    const d = new DebrisManager(p, { maxBodies: 4, minAgeForEviction: 0.5, killHeight: -50 });
    for (let i = 0; i < 4; i++) d.spawn(spawn());
    for (let i = 0; i < 60; i++) {
      p.step(1 / 60);
      d.update(1 / 60);
    }
    expect(d.spawn(spawn())).not.toBeNull();
    expect(d.stats.evicted).toBe(1);
    d.setLimit(2);
    expect(d.active).toHaveLength(2);
    for (let i = 0; i < 400; i++) {
      p.step(1 / 60);
      d.update(1 / 60);
    }
    expect(d.active).toHaveLength(0);
    expect(p.counts().dynamic).toBe(0);
  });
});

describe('tower collision', () => {
  it('derives the collision mode from state and time', () => {
    expect(towerCollisionModeAt('INTACT', 0, timeline)).toBe('intact');
    expect(towerCollisionModeAt('ACTIVE_DESTRUCTION', timeline.tipStart + 1, timeline)).toBe('kinematic');
    expect(towerCollisionModeAt('SETTLING', timeline.impact + 5, timeline)).toBe('wreck');
    expect(towerCollisionModeAt('RESTORATION', timeline.impact + 5, timeline)).toBe('intact');
  });

  it('switches representations and sweeps debris with the falling tower (Rapier)', async () => {
    const p = new RapierPhysicsAdapter();
    await p.init();
    const tower = make();
    const c = new TowerCollision(p, tower);
    c.update('intact', tower.posesAt(null));
    expect(p.counts().static).toBe(6);
    const t0 = timeline.tipStart + 2;
    c.update('kinematic', tower.posesAt(t0));
    expect(p.counts()).toMatchObject({ static: 1, kinematic: 5 });
    // A loose block resting on the roof of section 3 is carried along as it tips.
    const roof = topOf(tower, 3, t0);
    const block = p.createBody({
      kind: 'dynamic',
      shape: { type: 'box', halfExtents: vec3(1, 1, 1) },
      pose: { position: vec3(roof.x, roof.y + 1.2, roof.z), quaternion: quatIdentity() },
    });
    for (let t = t0; t < t0 + 3; t += 1 / 60) {
      c.update('kinematic', tower.posesAt(t));
      p.step(1 / 60);
    }
    expect(p.getPose(block).position.x).toBeGreaterThan(roof.x + 5);
    c.update('wreck', tower.posesAt(timeline.impact + 3));
    expect(p.counts()).toMatchObject({ static: 6, kinematic: 0 });
    expect(c.transitions).toBe(3);
    p.dispose();
  });
});
