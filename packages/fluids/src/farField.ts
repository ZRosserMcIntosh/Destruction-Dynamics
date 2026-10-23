// Far-field ocean: a sum of Gerstner waves. The same parameters drive the
// vertex shader and CPU sampling (buoyancy, cameras), so floating objects
// ride the waves the player sees.

import { SeededRandom } from '@destruction-dynamics/core';

export const GRAVITY = 9.81;

export interface GerstnerWave {
  /** Unit direction of travel in the x/z plane. */
  dirX: number;
  dirZ: number;
  amplitude: number;
  wavelength: number;
  /** 0..1 crest sharpness (Q, before normalisation across waves). */
  steepness: number;
  phase: number;
}

export type SeaState = 'calm' | 'moderate' | 'rough';

const SEA_STATES: Record<SeaState, { amp: number; lmin: number; lmax: number; steep: number }> = {
  calm: { amp: 0.25, lmin: 6, lmax: 40, steep: 0.35 },
  moderate: { amp: 0.6, lmin: 8, lmax: 70, steep: 0.5 },
  rough: { amp: 1.4, lmin: 12, lmax: 110, steep: 0.65 },
};

export const MAX_GERSTNER_WAVES = 8;

/** A deterministic wave set for a sea state and wind direction (radians, 0 = +x). */
export function makeWaves(state: SeaState, windAngle: number, seed = 1, count = 6): GerstnerWave[] {
  const p = SEA_STATES[state];
  const r = new SeededRandom(seed);
  const n = Math.min(count, MAX_GERSTNER_WAVES);
  const waves: GerstnerWave[] = [];
  for (let i = 0; i < n; i++) {
    const f = i / Math.max(1, n - 1);
    const wavelength = p.lmax * Math.pow(p.lmin / p.lmax, f) * r.range(0.85, 1.15);
    const a = windAngle + r.range(-0.6, 0.6);
    waves.push({
      dirX: Math.cos(a),
      dirZ: Math.sin(a),
      amplitude: p.amp * (wavelength / p.lmax) ** 0.8 * r.range(0.7, 1.1),
      wavelength,
      steepness: p.steep,
      phase: r.range(0, Math.PI * 2),
    });
  }
  return waves;
}

/** Per-wave constants: k, ω and normalised Q (so crests never loop). */
export function waveConstants(waves: readonly GerstnerWave[]) {
  return waves.map((w) => {
    const k = (2 * Math.PI) / w.wavelength;
    const omega = Math.sqrt(GRAVITY * k);
    const q = w.steepness / (k * w.amplitude * waves.length || 1);
    return { k, omega, q: Math.min(q, 1) };
  });
}

export class FarFieldOcean {
  readonly constants: ReturnType<typeof waveConstants>;

  constructor(readonly waves: readonly GerstnerWave[]) {
    this.constants = waveConstants(waves);
  }

  /** Displacement of the undisturbed surface point (x0, z0) at time t. */
  displace(x0: number, z0: number, t: number): { dx: number; dy: number; dz: number } {
    let dx = 0,
      dy = 0,
      dz = 0;
    for (let i = 0; i < this.waves.length; i++) {
      const w = this.waves[i]!;
      const c = this.constants[i]!;
      const theta = c.k * (w.dirX * x0 + w.dirZ * z0) - c.omega * t + w.phase;
      const cos = Math.cos(theta);
      const qa = c.q * w.amplitude * cos;
      dx += qa * w.dirX;
      dz += qa * w.dirZ;
      dy += w.amplitude * Math.sin(theta);
    }
    return { dx, dy, dz };
  }

  /** Surface height above the fixed point (x, z): inverts the horizontal displacement. */
  heightAt(x: number, z: number, t: number): number {
    let x0 = x,
      z0 = z;
    for (let it = 0; it < 2; it++) {
      const d = this.displace(x0, z0, t);
      x0 = x - d.dx;
      z0 = z - d.dz;
    }
    return this.displace(x0, z0, t).dy;
  }

  /** Orbital velocity of the surface at (x, z), horizontal components. */
  velocityAt(x: number, z: number, t: number): { x: number; z: number } {
    let vx = 0,
      vz = 0;
    for (let i = 0; i < this.waves.length; i++) {
      const w = this.waves[i]!;
      const c = this.constants[i]!;
      const theta = c.k * (w.dirX * x + w.dirZ * z) - c.omega * t + w.phase;
      const s = c.q * w.amplitude * c.omega * Math.sin(theta);
      vx += s * w.dirX;
      vz += s * w.dirZ;
    }
    return { x: vx, z: vz };
  }
}
