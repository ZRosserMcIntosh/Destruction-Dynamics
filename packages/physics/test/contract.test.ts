import { type Pose, identityPose, quatIdentity, vec3 } from '@destruction-dynamics/core';
import { afterEach, describe, expect, it } from 'vitest';
import { type PhysicsAdapter, RapierPhysicsAdapter, SimplePhysicsAdapter } from '../src';

const at = (x: number, y: number, z: number): Pose => ({
  position: vec3(x, y, z),
  quaternion: quatIdentity(),
});

const backends: [string, () => PhysicsAdapter][] = [
  ['rapier', () => new RapierPhysicsAdapter()],
  ['simple', () => new SimplePhysicsAdapter()],
];

describe.each(backends)('PhysicsAdapter contract: %s', (_name, make) => {
  let p: PhysicsAdapter;
  afterEach(() => p?.dispose());

  const ground = (adapter: PhysicsAdapter) =>
    adapter.createBody({
      kind: 'static',
      shape: { type: 'box', halfExtents: vec3(50, 1, 50) },
      pose: at(0, -1, 0),
    });

  const steps = (n: number, dt = 1 / 60) => {
    for (let i = 0; i < n; i++) p.step(dt);
  };

  it('drops a dynamic box onto static ground where it comes to rest and sleeps', async () => {
    p = make();
    await p.init();
    ground(p);
    const box = p.createBody({
      kind: 'dynamic',
      shape: { type: 'box', halfExtents: vec3(0.5, 0.5, 0.5) },
      pose: at(0, 5, 0),
    });
    steps(30);
    expect(p.getPose(box).position.y).toBeLessThan(5);
    expect(p.getLinearVelocity(box).y).toBeLessThan(0);
    steps(400);
    expect(p.getPose(box).position.y).toBeGreaterThan(0.3);
    expect(p.getPose(box).position.y).toBeLessThan(0.75);
    expect(p.isSleeping(box)).toBe(true);
    expect(p.counts()).toEqual({ dynamic: 1, kinematic: 0, static: 1, sleeping: 1 });
  });

  it('moves kinematic bodies to their targets', async () => {
    p = make();
    await p.init();
    const k = p.createBody({
      kind: 'kinematic',
      shape: { type: 'box', halfExtents: vec3(1, 1, 1) },
      pose: at(0, 0, 0),
    });
    p.setKinematicTarget(k, at(3, 2, 1));
    p.step(1 / 60);
    const pos = p.getPose(k).position;
    expect(pos.x).toBeCloseTo(3, 4);
    expect(pos.y).toBeCloseTo(2, 4);
  });

  it('responds to impulses and per-step forces in proportion to mass', async () => {
    p = make();
    await p.init();
    p.setGravity(vec3(0, 0, 0));
    const b = p.createBody({
      kind: 'dynamic',
      shape: { type: 'box', halfExtents: vec3(0.5, 0.5, 0.5) },
      pose: identityPose(),
      density: 1000,
      linearDamping: 0,
    });
    expect(p.getMass(b)).toBeCloseTo(1000, 0);
    p.applyImpulse(b, vec3(2000, 0, 0));
    expect(p.getLinearVelocity(b).x).toBeCloseTo(2, 3);
    p.addForce(b, vec3(0, 60000, 0));
    p.step(1 / 60);
    expect(p.getLinearVelocity(b).y).toBeCloseTo(1, 1);
    p.step(1 / 60);
    // Forces last one step only.
    expect(p.getLinearVelocity(b).y).toBeCloseTo(1, 1);
  });

  it('removes and clears bodies', async () => {
    p = make();
    await p.init();
    const a = ground(p);
    const b = p.createBody({ kind: 'dynamic', shape: { type: 'sphere', radius: 0.5 }, pose: at(0, 3, 0) });
    p.removeBody(a);
    expect(p.hasBody(a)).toBe(false);
    expect(p.hasBody(b)).toBe(true);
    p.clear();
    expect(p.counts()).toEqual({ dynamic: 0, kinematic: 0, static: 0, sleeping: 0 });
  });
});
