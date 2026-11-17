// The physics adapter contract. Destruction, fluids and gameplay code talk to
// physics only through this interface, so the backing engine (Rapier today)
// can be swapped or run in a worker without touching them.

import type { Pose, Vec3 } from '@destruction-dynamics/core';

export type BodyKind = 'dynamic' | 'kinematic' | 'static';

export type ShapeDesc =
  | { type: 'box'; halfExtents: Vec3 }
  | { type: 'sphere'; radius: number }
  /** Convex hull of points in the body frame (static obstacles, props). */
  | { type: 'convex'; points: Vec3[] };

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

/** Axis-aligned half extents and centre of a convex point set. */
export function pointsBounds(points: Vec3[]): { center: Vec3; halfExtents: Vec3 } {
  const lo = { x: Infinity, y: Infinity, z: Infinity };
  const hi = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const p of points) {
    lo.x = Math.min(lo.x, p.x);
    lo.y = Math.min(lo.y, p.y);
    lo.z = Math.min(lo.z, p.z);
    hi.x = Math.max(hi.x, p.x);
    hi.y = Math.max(hi.y, p.y);
    hi.z = Math.max(hi.z, p.z);
  }
  return {
    center: { x: (lo.x + hi.x) / 2, y: (lo.y + hi.y) / 2, z: (lo.z + hi.z) / 2 },
    halfExtents: { x: (hi.x - lo.x) / 2, y: (hi.y - lo.y) / 2, z: (hi.z - lo.z) / 2 },
  };
}

export const shapeVolume = (s: ShapeDesc): number => {
  if (s.type === 'box') return 8 * s.halfExtents.x * s.halfExtents.y * s.halfExtents.z;
  if (s.type === 'sphere') return (4 / 3) * Math.PI * s.radius ** 3;
  // Bounding-box estimate; convex shapes are used for static bodies, where mass does not matter.
  const h = pointsBounds(s.points).halfExtents;
  return 8 * h.x * h.y * h.z;
};
