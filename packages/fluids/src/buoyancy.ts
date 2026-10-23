// Buoyancy and drag for box-shaped bodies, independent of any physics engine.
// The body is sampled at a lattice of points; each submerged point contributes
// its share of displaced water at its own location, so partially submerged
// bodies pitch and roll on waves.

import { type Pose, type Vec3, add, quatRotate, vec3 } from '@destruction-dynamics/core';
import type { FluidAdapter } from './types';

export const WATER_DENSITY = 1025;

export interface BuoyantBody {
  pose: Pose;
  halfExtents: Vec3;
  velocity: Vec3;
  mass: number;
}

export interface PointForce {
  force: Vec3;
  point: Vec3;
}

export interface BuoyancyOptions {
  /** Lattice points per axis (1 → centre only, 2 → 8 points). */
  resolution?: number;
  /** Linear drag coefficient per submerged fraction, 1/s (scaled by mass). */
  drag?: number;
  gravity?: number;
}

export interface BuoyancyResult {
  forces: PointForce[];
  submergedFraction: number;
}

export function computeBuoyancy(
  body: BuoyantBody,
  water: FluidAdapter,
  o: BuoyancyOptions = {},
): BuoyancyResult {
  const res = Math.max(1, Math.floor(o.resolution ?? 2));
  const g = o.gravity ?? 9.81;
  const drag = o.drag ?? 0.8;
  const he = body.halfExtents;
  const volume = 8 * he.x * he.y * he.z;
  const pointVolume = volume / (res * res * res);
  // Each point owns a slab of height 2·he.y / res.
  const slab = (2 * he.y) / res;
  const forces: PointForce[] = [];
  let submerged = 0;
  for (let a = 0; a < res; a++)
    for (let b = 0; b < res; b++)
      for (let c = 0; c < res; c++) {
        const local = vec3(
          he.x * (((a + 0.5) / res) * 2 - 1),
          he.y * (((b + 0.5) / res) * 2 - 1),
          he.z * (((c + 0.5) / res) * 2 - 1),
        );
        const p = add(body.pose.position, quatRotate(body.pose.quaternion, local));
        const height = water.heightAt(p.x, p.z);
        const frac = Math.max(0, Math.min(1, (height - p.y) / slab + 0.5));
        if (frac <= 0) continue;
        submerged += frac;
        const lift = WATER_DENSITY * g * pointVolume * frac;
        const k = (drag * body.mass * frac) / (res * res * res);
        const current = water.velocityAt(p.x, p.z);
        forces.push({
          point: p,
          force: {
            x: k * (current.x - body.velocity.x),
            y: lift - k * body.velocity.y,
            z: k * (current.z - body.velocity.z),
          },
        });
      }
  return { forces, submergedFraction: submerged / (res * res * res) };
}
