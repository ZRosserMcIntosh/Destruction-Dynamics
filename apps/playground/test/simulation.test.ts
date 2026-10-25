import { describe, expect, it } from 'vitest';
import { DeepEndSimulation } from '../src/simulation';

const runUntil = (sim: DeepEndSimulation, pred: () => boolean, dt = 1 / 30, maxSteps = 5000) => {
  for (let i = 0; i < maxSteps && !pred(); i++) sim.update(dt, 10);
  return pred();
};

describe('The Deep End, headless', () => {
  it('runs the full sequence within budgets and restores', async () => {
    const sim = new DeepEndSimulation('balanced', 1234);
    await sim.init();
    expect(sim.physics.name).toBe('rapier');
    sim.update(1 / 30, 10);
    expect(sim.collision.mode).toBe('intact');

    sim.director.setTimeScale(2);
    const states = new Set<string>();
    sim.director.on('state', (s) => states.add(s.to));
    sim.director.arm();
    let maxDebris = 0;
    let maxParticles = 0;
    let kinematicSeen = false;
    const ok = runUntil(sim, () => {
      maxDebris = Math.max(maxDebris, sim.debris.active.length);
      maxParticles = Math.max(maxParticles, sim.particles.count);
      kinematicSeen ||= sim.collision.mode === 'kinematic';
      return sim.director.state === 'AFTERMATH';
    });
    expect(ok).toBe(true);
    expect([...states]).toEqual(
      expect.arrayContaining([
        'PREPARING',
        'TRIGGERED',
        'INITIAL_FAILURE',
        'ACTIVE_DESTRUCTION',
        'WATER_IMPACT',
        'SETTLING',
        'AFTERMATH',
      ]),
    );
    expect(kinematicSeen).toBe(true);
    expect(sim.collision.mode).toBe('wreck');
    expect(sim.navigationVersion).toBe(2); // intact → kinematic → wreck
    expect(maxDebris).toBeGreaterThan(20);
    expect(maxDebris).toBeLessThanOrEqual(sim.quality.maxDebrisBodies);
    expect(maxParticles).toBeLessThanOrEqual(sim.quality.maxParticles);
    expect(maxParticles).toBeGreaterThan(500);
    expect(sim.peakWave).toBeGreaterThan(3);
    expect(sim.stats().tiltDeg).toBeGreaterThan(80);
    expect(sim.stats().worldOverrides).toBeGreaterThan(0);
    expect(sim.tiles.activeTiles().length).toBeGreaterThan(0);
    // Balanced splits the tower into 12 independently simulated layers.
    expect(sim.stats().layers).toMatchObject({ count: 12, attached: true, status: 'ready' });

    // Floating props near the impact were pushed around by the water.
    expect(sim.floaters.every((f) => Number.isFinite(f.pose.position.y))).toBe(true);
    expect(sim.floaters.every((f) => Math.abs(f.pose.position.y) < 15)).toBe(true);

    sim.director.restoreNow();
    runUntil(sim, () => sim.director.state === 'INTACT');
    expect(sim.director.state).toBe('INTACT');
    expect(sim.debris.active).toHaveLength(0);
    expect(sim.collision.mode).toBe('intact');
    expect(sim.tiles.activeTiles()).toEqual([]);
    expect(sim.stats().worldOverrides).toBe(0);
  }, 120_000);

  it('applies quality presets to real budgets', async () => {
    const sim = new DeepEndSimulation('high', 1);
    await sim.init();
    sim.setQuality('dorito');
    expect(sim.ocean.near.n).toBe(96);
    expect(sim.particles.capacity).toBe(1500);
    expect(sim.debris.maxBodies).toBe(40);
    expect(sim.floaters).toHaveLength(6);
    expect(sim.plan.columns).toBe(3);
    expect(sim.tower.sections).toHaveLength(6);
    expect(sim.stats().layers.status).toBe('authored');
  });

  it('saves and loads mid-event, re-creating the surge', async () => {
    const a = new DeepEndSimulation('dorito', 99);
    await a.init();
    a.director.trigger();
    a.director.setTimeScale(4);
    runUntil(a, () => a.director.state === 'SETTLING');
    const save = JSON.parse(JSON.stringify(a.save()));
    const b = new DeepEndSimulation('dorito', 5);
    await b.init();
    b.load(save);
    b.update(1 / 30, 10);
    expect(b.director.state).toBe('SETTLING');
    expect(b.director.seed).toBe(99);
    expect(b.ocean.surges.length).toBeGreaterThan(0);
    expect(b.collision.mode).toBe('wreck');
  });

  it('keeps the event correct when the budget governor degrades secondary work', async () => {
    const sim = new DeepEndSimulation('balanced', 3);
    await sim.init();
    sim.director.trigger();
    sim.director.setTimeScale(3);
    for (let i = 0; i < 200; i++) sim.update(1 / 30, 80);
    expect(sim.degradation.level).toBe(3);
    expect(sim.particles.limit).toBeLessThan(sim.quality.maxParticles);
    expect(sim.ocean.stepDivisor).toBe(3);
    runUntil(sim, () => sim.director.state === 'AFTERMATH', 1 / 30);
    expect(sim.director.state).toBe('AFTERMATH');
    expect(sim.stats().tiltDeg).toBeGreaterThan(80);
  }, 120_000);
});
