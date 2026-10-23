// Particle effect emitters keyed by cue type. Each takes a seeded stream so
// the same event seed produces the same spectacle.

import type { SeededRandom, Vec3 } from '@destruction-dynamics/core';
import type { ParticleSim } from '@destruction-dynamics/rendering';

export interface Footprint {
  position: Vec3;
  halfLength: number;
  halfWidth: number;
  yaw: number;
}

export function emitDust(p: ParticleSim, r: SeededRandom, center: Vec3, spread: number, count: number): void {
  p.emit('dust', count, r, (s, rr) => {
    const a = rr.range(0, Math.PI * 2);
    const d = rr.range(0, spread);
    s.x = center.x + Math.cos(a) * d;
    s.y = center.y + rr.range(-3, 6);
    s.z = center.z + Math.sin(a) * d;
    s.vx = Math.cos(a) * rr.range(1, 7);
    s.vy = rr.range(-1, 3);
    s.vz = Math.sin(a) * rr.range(1, 7);
    s.size = rr.range(3, 8);
    s.life = rr.range(5, 12);
  });
}

export function emitSparks(p: ParticleSim, r: SeededRandom, center: Vec3, count: number): void {
  p.emit('sparks', count, r, (s, rr) => {
    s.x = center.x + rr.range(-16, 16);
    s.y = center.y + rr.range(-1, 2);
    s.z = center.z + rr.range(-15, 15);
    s.vx = rr.range(-8, 8);
    s.vy = rr.range(0, 9);
    s.vz = rr.range(-8, 8);
    s.size = rr.range(0.25, 0.5);
    s.life = rr.range(0.6, 1.8);
  });
}

export function emitGlass(
  p: ParticleSim,
  r: SeededRandom,
  at: Vec3,
  vel: Vec3,
  normal: Vec3,
  count: number,
): void {
  p.emit('glass', count, r, (s, rr) => {
    s.x = at.x + rr.range(-1, 1);
    s.y = at.y + rr.range(-1, 1);
    s.z = at.z + rr.range(-1, 1);
    const k = rr.range(1, 6);
    s.vx = vel.x + normal.x * k + rr.range(-2, 2);
    s.vy = vel.y + normal.y * k + rr.range(-1, 3);
    s.vz = vel.z + normal.z * k + rr.range(-2, 2);
    s.size = rr.range(0.15, 0.45);
    s.life = rr.range(2, 6);
  });
}

/** Point on a footprint's perimeter (or interior when `inside`). */
function footprintPoint(
  f: Footprint,
  rr: SeededRandom,
  inside: boolean,
): { x: number; z: number; nx: number; nz: number } {
  const c = Math.cos(f.yaw);
  const sn = Math.sin(f.yaw);
  let lx: number, lz: number, nx: number, nz: number;
  if (inside) {
    lx = rr.range(-f.halfLength, f.halfLength);
    lz = rr.range(-f.halfWidth, f.halfWidth);
    nx = 0;
    nz = Math.sign(lz) || 1;
  } else if (rr.chance(0.85)) {
    lx = rr.range(-f.halfLength, f.halfLength);
    lz = rr.chance(0.5) ? f.halfWidth : -f.halfWidth;
    nx = 0;
    nz = Math.sign(lz);
  } else {
    lz = rr.range(-f.halfWidth, f.halfWidth);
    lx = rr.chance(0.7) ? f.halfLength : -f.halfLength;
    nx = Math.sign(lx);
    nz = 0;
  }
  return {
    x: f.position.x + c * lx - sn * lz,
    z: f.position.z + sn * lx + c * lz,
    nx: c * nx - sn * nz,
    nz: sn * nx + c * nz,
  };
}

/** The main sheet of spray thrown up along the impact footprint. */
export function emitSpraySheet(
  p: ParticleSim,
  r: SeededRandom,
  f: Footprint,
  count: number,
  power = 1,
): void {
  p.emit('spray', count, r, (s, rr) => {
    const q = footprintPoint(f, rr, false);
    const out = rr.range(6, 22) * power;
    s.x = q.x;
    s.y = rr.range(0, 3);
    s.z = q.z;
    s.vx = q.nx * out + rr.range(-3, 3);
    s.vy = rr.range(14, 48) * power * (0.5 + 0.5 * rr.next());
    s.vz = q.nz * out + rr.range(-3, 3);
    s.size = rr.range(1.2, 4);
    s.life = rr.range(2.5, 6);
  });
}

export function emitMist(p: ParticleSim, r: SeededRandom, f: Footprint, count: number): void {
  p.emit('mist', count, r, (s, rr) => {
    const q = footprintPoint(f, rr, true);
    s.x = q.x + rr.range(-20, 20);
    s.y = rr.range(4, 45);
    s.z = q.z + rr.range(-25, 25);
    s.vx = rr.range(-3, 3) + 2;
    s.vy = rr.range(0, 2);
    s.vz = rr.range(-3, 3);
    s.size = rr.range(10, 22);
    s.life = rr.range(8, 18);
  });
}

export function emitFoam(p: ParticleSim, r: SeededRandom, f: Footprint, count: number): void {
  p.emit('foam', count, r, (s, rr) => {
    const q = footprintPoint(f, rr, true);
    s.x = q.x + rr.range(-25, 25);
    s.z = q.z + rr.range(-25, 25);
    s.vx = rr.range(-4, 4);
    s.vz = rr.range(-4, 4);
    s.size = rr.range(3, 7);
    s.life = rr.range(6, 14);
  });
}

export function emitBubbles(
  p: ParticleSim,
  r: SeededRandom,
  around: Vec3[],
  seabed: (x: number, z: number) => number,
  count: number,
): void {
  if (around.length === 0) return;
  p.emit('bubble', count, r, (s, rr) => {
    const c = rr.pick(around);
    s.x = c.x + rr.range(-30, 30);
    s.z = c.z + rr.range(-14, 14);
    s.y = Math.max(seabed(s.x, s.z) + 1, c.y + rr.range(-10, 10));
    s.vy = rr.range(0.5, 2.5);
    s.vx = rr.range(-0.3, 0.3);
    s.vz = rr.range(-0.3, 0.3);
    s.size = rr.range(0.08, 0.4);
    s.life = rr.range(6, 20);
  });
}

export function emitSplash(p: ParticleSim, r: SeededRandom, at: Vec3, speed: number, count: number): void {
  p.emit('spray', count, r, (s, rr) => {
    const a = rr.range(0, Math.PI * 2);
    s.x = at.x;
    s.y = at.y + 0.2;
    s.z = at.z;
    s.vx = Math.cos(a) * rr.range(1, 4);
    s.vy = rr.range(2, 4 + speed * 0.6);
    s.vz = Math.sin(a) * rr.range(1, 4);
    s.size = rr.range(0.4, 1.2);
    s.life = rr.range(1, 2.5);
  });
}
