// Budgeted, pooled debris bodies. The manager never lets the simulation exceed
// its body budget: a spawn either gets a body (evicting the oldest settled
// piece if needed) or is refused, and the caller degrades to particles.

import type { Pose, Vec3 } from '@destruction-dynamics/core';
import type { BodyHandle, PhysicsAdapter } from '@destruction-dynamics/physics';
import type { DebrisSpawn } from './types';

export interface DebrisBody {
  id: number;
  handle: BodyHandle;
  kind: DebrisSpawn['kind'];
  halfExtents: Vec3;
  age: number;
  lifetime: number;
  pose: Pose;
}

export interface DebrisStats {
  active: number;
  spawned: number;
  refused: number;
  evicted: number;
  expired: number;
}

export interface DebrisOptions {
  maxBodies: number;
  /** Bodies younger than this are never evicted, s. */
  minAgeForEviction?: number;
  /** Bodies below this height are retired (sunk out of view). */
  killHeight?: number;
  /** Collision group for debris (see BodyDesc.group). */
  group?: number;
  mask?: number;
}

export class DebrisManager {
  private readonly bodies: DebrisBody[] = [];
  private nextId = 1;
  private limit: number;
  readonly stats: DebrisStats = { active: 0, spawned: 0, refused: 0, evicted: 0, expired: 0 };

  constructor(
    private readonly physics: PhysicsAdapter,
    private readonly opts: DebrisOptions,
  ) {
    this.limit = opts.maxBodies;
  }

  get active(): readonly DebrisBody[] {
    return this.bodies;
  }

  get maxBodies(): number {
    return this.limit;
  }

  /** Changes the budget; excess bodies are retired oldest-first. */
  setLimit(n: number): void {
    this.limit = Math.max(0, Math.floor(n));
    while (this.bodies.length > this.limit) this.retire(0, 'evicted');
  }

  spawn(s: DebrisSpawn): DebrisBody | null {
    if (this.bodies.length >= this.limit && !this.evictOne()) {
      this.stats.refused++;
      return null;
    }
    const handle = this.physics.createBody({
      kind: 'dynamic',
      shape: { type: 'box', halfExtents: s.halfExtents },
      pose: s.pose,
      density: s.density,
      linearVelocity: s.velocity,
      angularVelocity: s.angularVelocity,
      friction: 0.7,
      restitution: 0.15,
      linearDamping: 0.05,
      angularDamping: 0.2,
      ...(this.opts.group !== undefined ? { group: this.opts.group } : {}),
      ...(this.opts.mask !== undefined ? { mask: this.opts.mask } : {}),
    });
    const body: DebrisBody = {
      id: this.nextId++,
      handle,
      kind: s.kind,
      halfExtents: s.halfExtents,
      age: 0,
      lifetime: s.lifetime,
      pose: s.pose,
    };
    this.bodies.push(body);
    this.stats.spawned++;
    this.stats.active = this.bodies.length;
    return body;
  }

  /** Prefer the oldest sleeping body, then the oldest body past the minimum age. */
  private evictOne(): boolean {
    const minAge = this.opts.minAgeForEviction ?? 1.5;
    let idx = this.bodies.findIndex((b) => this.physics.isSleeping(b.handle));
    if (idx < 0) idx = this.bodies.findIndex((b) => b.age >= minAge);
    if (idx < 0) return false;
    this.retire(idx, 'evicted');
    return true;
  }

  private retire(idx: number, why: 'evicted' | 'expired'): void {
    const [b] = this.bodies.splice(idx, 1);
    if (!b) return;
    this.physics.removeBody(b.handle);
    this.stats[why]++;
    this.stats.active = this.bodies.length;
  }

  /** Ages bodies, refreshes cached poses and retires expired ones. Call after physics.step. */
  update(dt: number): void {
    const kill = this.opts.killHeight ?? -Infinity;
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      const b = this.bodies[i]!;
      b.age += dt;
      b.pose = this.physics.getPose(b.handle);
      if (b.age > b.lifetime || b.pose.position.y < kill) this.retire(i, 'expired');
    }
  }

  clear(): void {
    for (const b of this.bodies) this.physics.removeBody(b.handle);
    this.bodies.length = 0;
    this.stats.active = 0;
  }
}
