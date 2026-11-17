// The physics adapter contract. Destruction, fluids and gameplay code talk to
// physics only through this interface, so the backing engine (Rapier today)
// can be swapped or run in a worker without touching them.

import type { Pose, Vec3 } from '@destruction-dynamics/core';

export type BodyKind = 'dynamic' | 'kinematic' | 'static';

export type ShapeDesc = { type: 'box'; halfExtents: Vec3 } | { type: 'sphere'; radius: number };

export interface BodyDesc {
  kind: BodyKind;
  shape: ShapeDesc;
  pose: Pose;
  /** kg/m³; dynamic bodies only. */
  density?: number;
  friction?: number;
  restitution?: number;
  linearDamping?: number;
  angularDamping?: number;
  linearVelocity?: Vec3;
  angularVelocity?: Vec3;
  /** Continuous collision detection for fast debris. */
  ccd?: boolean;
  /** Collision group bitmask: bodies collide when (a.group & b.mask) && (b.group & a.mask). */
  group?: number;
  mask?: number;
}

/** Opaque handle. */
export type BodyHandle = number & { readonly __body: unique symbol };

export interface BodyCounts {
  dynamic: number;
  kinematic: number;
  static: number;
  sleeping: number;
}

export interface PhysicsAdapter {
  readonly name: string;
  init(): Promise<void>;
  setGravity(g: Vec3): void;
  createBody(desc: BodyDesc): BodyHandle;
  removeBody(h: BodyHandle): void;
  hasBody(h: BodyHandle): boolean;
  /** Kinematic bodies: target pose reached at the end of the next step. */
  setKinematicTarget(h: BodyHandle, pose: Pose): void;
  /** Teleport (any kind). */
  setPose(h: BodyHandle, pose: Pose): void;
  getPose(h: BodyHandle): Pose;
  getLinearVelocity(h: BodyHandle): Vec3;
  setLinearVelocity(h: BodyHandle, v: Vec3): void;
  setAngularVelocity(h: BodyHandle, v: Vec3): void;
  getMass(h: BodyHandle): number;
  applyImpulse(h: BodyHandle, impulse: Vec3): void;
  /** Force applied during the next step only, at a world point (defaults to the centre of mass). */
  addForce(h: BodyHandle, force: Vec3, worldPoint?: Vec3): void;
  isSleeping(h: BodyHandle): boolean;
  step(dt: number): void;
  counts(): BodyCounts;
  /** Removes every body. */
  clear(): void;
  dispose(): void;
}

export const DEFAULT_GRAVITY: Vec3 = { x: 0, y: -9.81, z: 0 };

export const shapeVolume = (s: ShapeDesc): number =>
  s.type === 'box'
    ? 8 * s.halfExtents.x * s.halfExtents.y * s.halfExtents.z
    : (4 / 3) * Math.PI * s.radius ** 3;
