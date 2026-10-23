// A tiny dependency-free backend: dynamic bodies are treated as spheres
// (radius = smallest half-extent) colliding with static and kinematic oriented
// boxes and spheres. Orientation integrates angular velocity but there is no
// rotational collision response. It exists as a fallback when WASM is not
// available and as a second implementation that keeps the adapter contract honest.

import {
  type Pose,
  type Quat,
  type Vec3,
  add,
  dot,
  length,
  quatMultiply,
  quatRotate,
  scale,
  sub,
} from '@destruction-dynamics/core';
import {
  type BodyCounts,
  type BodyDesc,
  type BodyHandle,
  DEFAULT_GRAVITY,
  type PhysicsAdapter,
  shapeVolume,
} from './types';

interface SimpleBody {
  desc: BodyDesc;
  pose: Pose;
  target: Pose | null;
  vel: Vec3;
  angVel: Vec3;
  force: Vec3;
  mass: number;
  radius: number;
  restTime: number;
  sleeping: boolean;
}

const conj = (q: Quat): Quat => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });

export class SimplePhysicsAdapter implements PhysicsAdapter {
  readonly name = 'simple';
  private gravity: Vec3 = DEFAULT_GRAVITY;
  private readonly bodies = new Map<number, SimpleBody>();
  private nextHandle = 1;

  async init(): Promise<void> {}

  setGravity(g: Vec3): void {
    this.gravity = g;
  }

  private get(h: BodyHandle): SimpleBody {
    const b = this.bodies.get(h);
    if (!b) throw new Error(`unknown body ${h}`);
    return b;
  }

  createBody(desc: BodyDesc): BodyHandle {
    const s = desc.shape;
    const radius =
      s.type === 'sphere' ? s.radius : Math.min(s.halfExtents.x, s.halfExtents.y, s.halfExtents.z);
    const h = this.nextHandle++ as BodyHandle;
    this.bodies.set(h, {
      desc,
      pose: { position: { ...desc.pose.position }, quaternion: { ...desc.pose.quaternion } },
      target: null,
      vel: { ...(desc.linearVelocity ?? { x: 0, y: 0, z: 0 }) },
      angVel: { ...(desc.angularVelocity ?? { x: 0, y: 0, z: 0 }) },
      force: { x: 0, y: 0, z: 0 },
      mass: (desc.density ?? 1000) * shapeVolume(s),
      radius,
      restTime: 0,
      sleeping: false,
    });
    return h;
  }

  removeBody(h: BodyHandle): void {
    this.bodies.delete(h);
  }
  hasBody(h: BodyHandle): boolean {
    return this.bodies.has(h);
  }
  setKinematicTarget(h: BodyHandle, pose: Pose): void {
    this.get(h).target = pose;
  }
  setPose(h: BodyHandle, pose: Pose): void {
    const b = this.get(h);
    b.pose = { position: { ...pose.position }, quaternion: { ...pose.quaternion } };
    b.sleeping = false;
  }
  getPose(h: BodyHandle): Pose {
    const p = this.get(h).pose;
    return { position: { ...p.position }, quaternion: { ...p.quaternion } };
  }
  getLinearVelocity(h: BodyHandle): Vec3 {
    return { ...this.get(h).vel };
  }
  setLinearVelocity(h: BodyHandle, v: Vec3): void {
    const b = this.get(h);
    b.vel = { ...v };
    b.sleeping = false;
  }
  setAngularVelocity(h: BodyHandle, v: Vec3): void {
    this.get(h).angVel = { ...v };
  }
  getMass(h: BodyHandle): number {
    return this.get(h).mass;
  }
  applyImpulse(h: BodyHandle, impulse: Vec3): void {
    const b = this.get(h);
    if (b.desc.kind !== 'dynamic') return;
    b.vel = add(b.vel, scale(impulse, 1 / b.mass));
    b.sleeping = false;
  }
  addForce(h: BodyHandle, force: Vec3): void {
    const b = this.get(h);
    if (b.desc.kind !== 'dynamic') return;
    b.force = add(b.force, force);
    if (length(force) > 1e-3 * b.mass) b.sleeping = false;
  }
  isSleeping(h: BodyHandle): boolean {
    return this.get(h).sleeping;
  }

  step(dt: number): void {
    const colliders = [...this.bodies.values()].filter((b) => b.desc.kind !== 'dynamic');
    for (const b of this.bodies.values()) {
      if (b.desc.kind === 'kinematic' && b.target) {
        b.vel = scale(sub(b.target.position, b.pose.position), 1 / dt);
        b.pose = b.target;
        b.target = null;
      }
    }
    for (const b of this.bodies.values()) {
      if (b.desc.kind !== 'dynamic' || b.sleeping) {
        b.force = { x: 0, y: 0, z: 0 };
        continue;
      }
      const acc = add(this.gravity, scale(b.force, 1 / b.mass));
      b.vel = scale(add(b.vel, scale(acc, dt)), Math.max(0, 1 - (b.desc.linearDamping ?? 0.05) * dt));
      b.pose.position = add(b.pose.position, scale(b.vel, dt));
      const w = b.angVel;
      const dq = quatMultiply(
        { x: w.x * dt * 0.5, y: w.y * dt * 0.5, z: w.z * dt * 0.5, w: 0 },
        b.pose.quaternion,
      );
      const q = {
        x: b.pose.quaternion.x + dq.x,
        y: b.pose.quaternion.y + dq.y,
        z: b.pose.quaternion.z + dq.z,
        w: b.pose.quaternion.w + dq.w,
      };
      const ql = Math.hypot(q.x, q.y, q.z, q.w);
      b.pose.quaternion = { x: q.x / ql, y: q.y / ql, z: q.z / ql, w: q.w / ql };
      b.angVel = scale(b.angVel, Math.max(0, 1 - (b.desc.angularDamping ?? 0.1) * dt));
      let contact = false;
      for (const c of colliders) contact = this.collide(b, c) || contact;
      b.force = { x: 0, y: 0, z: 0 };
      if (contact && length(b.vel) < 0.15) {
        b.restTime += dt;
        if (b.restTime > 0.5) {
          b.sleeping = true;
          b.vel = { x: 0, y: 0, z: 0 };
          b.angVel = { x: 0, y: 0, z: 0 };
        }
      } else b.restTime = 0;
    }
  }

  /** Sphere vs oriented box or sphere; returns true on contact. */
  private collide(b: SimpleBody, c: SimpleBody): boolean {
    let normal: Vec3;
    let depth: number;
    if (c.desc.shape.type === 'sphere') {
      const d = sub(b.pose.position, c.pose.position);
      const dist = length(d);
      depth = b.radius + c.desc.shape.radius - dist;
      if (depth <= 0) return false;
      normal = dist > 1e-9 ? scale(d, 1 / dist) : { x: 0, y: 1, z: 0 };
    } else {
      const he = c.desc.shape.halfExtents;
      const inv = conj(c.pose.quaternion);
      const local = quatRotate(inv, sub(b.pose.position, c.pose.position));
      const cl = {
        x: Math.max(-he.x, Math.min(he.x, local.x)),
        y: Math.max(-he.y, Math.min(he.y, local.y)),
        z: Math.max(-he.z, Math.min(he.z, local.z)),
      };
      const d = sub(local, cl);
      const dist = length(d);
      let nLocal: Vec3;
      if (dist > 1e-9) {
        depth = b.radius - dist;
        nLocal = scale(d, 1 / dist);
      } else {
        // Centre inside the box: push out along the axis of least penetration.
        const px = he.x - Math.abs(local.x);
        const py = he.y - Math.abs(local.y);
        const pz = he.z - Math.abs(local.z);
        if (py <= px && py <= pz) nLocal = { x: 0, y: Math.sign(local.y) || 1, z: 0 };
        else if (px <= pz) nLocal = { x: Math.sign(local.x) || 1, y: 0, z: 0 };
        else nLocal = { x: 0, y: 0, z: Math.sign(local.z) || 1 };
        depth = b.radius + Math.min(px, py, pz);
      }
      if (depth <= 0) return false;
      normal = quatRotate(c.pose.quaternion, nLocal);
    }
    b.pose.position = add(b.pose.position, scale(normal, depth));
    const rel = sub(b.vel, c.vel);
    const vn = dot(rel, normal);
    if (vn < 0) {
      const e = b.desc.restitution ?? 0.1;
      const mu = b.desc.friction ?? 0.6;
      const vnVec = scale(normal, vn);
      const vt = sub(rel, vnVec);
      const vtLen = length(vt);
      const friction = vtLen > 0 ? Math.max(0, 1 - (mu * Math.abs(vn) * (1 + e)) / vtLen) : 0;
      b.vel = add(c.vel, add(scale(vt, friction), scale(vnVec, -e)));
    }
    return true;
  }

  counts(): BodyCounts {
    const c: BodyCounts = { dynamic: 0, kinematic: 0, static: 0, sleeping: 0 };
    for (const b of this.bodies.values()) {
      c[b.desc.kind]++;
      if (b.desc.kind === 'dynamic' && b.sleeping) c.sleeping++;
    }
    return c;
  }

  clear(): void {
    this.bodies.clear();
  }
  dispose(): void {
    this.bodies.clear();
  }
}
