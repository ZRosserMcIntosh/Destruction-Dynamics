// Rapier (https://rapier.rs) backend, via the -compat build that embeds its
// WASM so it loads in browsers, workers and Node without bundler plugins.

import type { Pose, Vec3 } from '@destruction-dynamics/core';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  type BodyCounts,
  type BodyDesc,
  type BodyHandle,
  DEFAULT_GRAVITY,
  type PhysicsAdapter,
} from './types';

let initPromise: Promise<void> | null = null;

/** Loads the Rapier WASM module once per process. */
export function initRapier(): Promise<void> {
  // rapier3d-compat logs a deprecation note for the zero-argument init; pass an empty object.
  initPromise ??= (RAPIER.init as (opts?: object) => Promise<void>)({});
  return initPromise;
}

interface Entry {
  body: RAPIER.RigidBody;
  kind: BodyDesc['kind'];
}

export class RapierPhysicsAdapter implements PhysicsAdapter {
  readonly name = 'rapier';
  private world: RAPIER.World | null = null;
  private readonly bodies = new Map<number, Entry>();
  private nextHandle = 1;
  private gravity: Vec3 = DEFAULT_GRAVITY;
  private forced = new Set<RAPIER.RigidBody>();

  async init(): Promise<void> {
    await initRapier();
    this.world = new RAPIER.World(this.gravity);
  }

  private get w(): RAPIER.World {
    if (!this.world) throw new Error('RapierPhysicsAdapter used before init()');
    return this.world;
  }

  private entry(h: BodyHandle): Entry {
    const e = this.bodies.get(h);
    if (!e) throw new Error(`unknown body ${h}`);
    return e;
  }

  setGravity(g: Vec3): void {
    this.gravity = g;
    if (this.world) this.world.gravity = g;
  }

  createBody(d: BodyDesc): BodyHandle {
    const rb =
      d.kind === 'dynamic'
        ? RAPIER.RigidBodyDesc.dynamic()
        : d.kind === 'kinematic'
          ? RAPIER.RigidBodyDesc.kinematicPositionBased()
          : RAPIER.RigidBodyDesc.fixed();
    rb.setTranslation(d.pose.position.x, d.pose.position.y, d.pose.position.z).setRotation(d.pose.quaternion);
    if (d.kind === 'dynamic') {
      rb.setLinearDamping(d.linearDamping ?? 0.05).setAngularDamping(d.angularDamping ?? 0.1);
      if (d.linearVelocity) rb.setLinvel(d.linearVelocity.x, d.linearVelocity.y, d.linearVelocity.z);
      if (d.angularVelocity) rb.setAngvel(d.angularVelocity);
      if (d.ccd) rb.setCcdEnabled(true);
    }
    const body = this.w.createRigidBody(rb);
    const cd =
      d.shape.type === 'box'
        ? RAPIER.ColliderDesc.cuboid(d.shape.halfExtents.x, d.shape.halfExtents.y, d.shape.halfExtents.z)
        : d.shape.type === 'sphere'
          ? RAPIER.ColliderDesc.ball(d.shape.radius)
          : RAPIER.ColliderDesc.convexHull(Float32Array.from(d.shape.points.flatMap((p) => [p.x, p.y, p.z])));
    if (!cd) throw new Error('could not build a convex hull from the given points');
    cd.setDensity(d.density ?? 1000)
      .setFriction(d.friction ?? 0.6)
      .setRestitution(d.restitution ?? 0.1);
    if (d.group !== undefined || d.mask !== undefined) {
      const g = (d.group ?? 0xffff) & 0xffff;
      const m = (d.mask ?? 0xffff) & 0xffff;
      cd.setCollisionGroups((g << 16) | m);
    }
    this.w.createCollider(cd, body);
    const h = this.nextHandle++ as BodyHandle;
    this.bodies.set(h, { body, kind: d.kind });
    return h;
  }

  removeBody(h: BodyHandle): void {
    const e = this.bodies.get(h);
    if (!e) return;
    this.forced.delete(e.body);
    this.w.removeRigidBody(e.body);
    this.bodies.delete(h);
  }

  hasBody(h: BodyHandle): boolean {
    return this.bodies.has(h);
  }

  setKinematicTarget(h: BodyHandle, pose: Pose): void {
    const { body } = this.entry(h);
    body.setNextKinematicTranslation(pose.position);
    body.setNextKinematicRotation(pose.quaternion);
  }

  setPose(h: BodyHandle, pose: Pose): void {
    const { body } = this.entry(h);
    body.setTranslation(pose.position, true);
    body.setRotation(pose.quaternion, true);
  }

  getPose(h: BodyHandle): Pose {
    const { body } = this.entry(h);
    const t = body.translation();
    const r = body.rotation();
    return { position: { x: t.x, y: t.y, z: t.z }, quaternion: { x: r.x, y: r.y, z: r.z, w: r.w } };
  }

  getLinearVelocity(h: BodyHandle): Vec3 {
    const v = this.entry(h).body.linvel();
    return { x: v.x, y: v.y, z: v.z };
  }

  setLinearVelocity(h: BodyHandle, v: Vec3): void {
    this.entry(h).body.setLinvel(v, true);
  }

  setAngularVelocity(h: BodyHandle, v: Vec3): void {
    this.entry(h).body.setAngvel(v, true);
  }

  getMass(h: BodyHandle): number {
    return this.entry(h).body.mass();
  }

  applyImpulse(h: BodyHandle, impulse: Vec3): void {
    this.entry(h).body.applyImpulse(impulse, true);
  }

  addForce(h: BodyHandle, force: Vec3, worldPoint?: Vec3): void {
    const { body } = this.entry(h);
    if (worldPoint) body.addForceAtPoint(force, worldPoint, true);
    else body.addForce(force, true);
    this.forced.add(body);
  }

  isSleeping(h: BodyHandle): boolean {
    return this.entry(h).body.isSleeping();
  }

  step(dt: number): void {
    this.w.timestep = dt;
    this.w.step();
    for (const b of this.forced) b.resetForces(false);
    this.forced.clear();
  }

  counts(): BodyCounts {
    const c: BodyCounts = { dynamic: 0, kinematic: 0, static: 0, sleeping: 0 };
    for (const e of this.bodies.values()) {
      c[e.kind]++;
      if (e.kind === 'dynamic' && e.body.isSleeping()) c.sleeping++;
    }
    return c;
  }

  clear(): void {
    for (const h of [...this.bodies.keys()]) this.removeBody(h as BodyHandle);
  }

  dispose(): void {
    this.bodies.clear();
    this.forced.clear();
    this.world?.free();
    this.world = null;
  }
}
