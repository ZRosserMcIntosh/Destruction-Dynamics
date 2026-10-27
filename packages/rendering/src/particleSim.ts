// CPU particle simulation for spray, mist, dust, glass, sparks, bubbles and
// suspended sediment. Structure-of-arrays, fixed capacity, swap-remove on
// death, no allocation per frame. Rendering reads the arrays directly.

import type { SeededRandom } from '@destruction-dynamics/core';

export const PARTICLE_KINDS = [
  'spray',
  'mist',
  'dust',
  'glass',
  'sparks',
  'bubble',
  'foam',
  'juice',
] as const;
export type ParticleKind = (typeof PARTICLE_KINDS)[number];

export interface KindParams {
  /** Multiplier on gravity (negative rises). */
  gravity: number;
  /** Linear drag, 1/s. */
  drag: number;
  /** Size growth, m/s. */
  growth: number;
  color: [number, number, number];
  alpha: number;
  additive: boolean;
  /** Dies on entering water (spray, glass) or on leaving it (bubbles). */
  waterBehaviour: 'die-below' | 'die-above' | 'float' | 'ignore';
}

export const KIND_PARAMS: Record<ParticleKind, KindParams> = {
  spray: {
    gravity: 1,
    drag: 0.25,
    growth: 0.6,
    color: [0.92, 0.96, 1],
    alpha: 0.75,
    additive: false,
    waterBehaviour: 'die-below',
  },
  mist: {
    gravity: -0.015,
    drag: 0.6,
    growth: 3.5,
    color: [0.88, 0.92, 0.95],
    alpha: 0.12,
    additive: false,
    waterBehaviour: 'ignore',
  },
  dust: {
    gravity: 0.02,
    drag: 0.45,
    growth: 2.2,
    color: [0.62, 0.58, 0.52],
    alpha: 0.35,
    additive: false,
    waterBehaviour: 'ignore',
  },
  glass: {
    gravity: 1,
    drag: 0.05,
    growth: 0,
    color: [0.85, 0.95, 1],
    alpha: 0.95,
    additive: true,
    waterBehaviour: 'die-below',
  },
  sparks: {
    gravity: 1,
    drag: 0.4,
    growth: -0.05,
    color: [1, 0.6, 0.2],
    alpha: 1,
    additive: true,
    waterBehaviour: 'die-below',
  },
  bubble: {
    gravity: -0.12,
    drag: 1.2,
    growth: 0.04,
    color: [0.8, 0.95, 1],
    alpha: 0.6,
    additive: true,
    waterBehaviour: 'die-above',
  },
  foam: {
    gravity: 0,
    drag: 0.8,
    growth: 0.3,
    color: [0.95, 0.97, 1],
    alpha: 0.4,
    additive: false,
    waterBehaviour: 'float',
  },
  juice: {
    gravity: 1,
    drag: 0.15,
    growth: 0.02,
    color: [0.82, 0.12, 0.18],
    alpha: 0.85,
    additive: false,
    waterBehaviour: 'ignore',
  },
};

export interface SpawnParams {
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
  size: number;
  life: number;
}

export interface ParticleEnvironment {
  gravity: number;
  waterHeight: (x: number, z: number) => number;
  groundHeight: (x: number, z: number) => number;
}

export class ParticleSim {
  readonly capacity: number;
  count = 0;
  /** Effective cap (quality × governor); emission beyond it is dropped. */
  limit: number;
  readonly px: Float32Array;
  readonly py: Float32Array;
  readonly pz: Float32Array;
  readonly vx: Float32Array;
  readonly vy: Float32Array;
  readonly vz: Float32Array;
  readonly size: Float32Array;
  readonly age: Float32Array;
  readonly life: Float32Array;
  readonly kind: Uint8Array;
  dropped = 0;
  private readonly scratch: SpawnParams = { x: 0, y: 0, z: 0, vx: 0, vy: 0, vz: 0, size: 1, life: 1 };

  constructor(capacity: number) {
    this.capacity = capacity;
    this.limit = capacity;
    this.px = new Float32Array(capacity);
    this.py = new Float32Array(capacity);
    this.pz = new Float32Array(capacity);
    this.vx = new Float32Array(capacity);
    this.vy = new Float32Array(capacity);
    this.vz = new Float32Array(capacity);
    this.size = new Float32Array(capacity);
    this.age = new Float32Array(capacity);
    this.life = new Float32Array(capacity);
    this.kind = new Uint8Array(capacity);
  }

  /**
   * Emits up to `n` particles of `kind`; `init` fills each spawn from a
   * seeded stream. Returns how many were emitted.
   */
  emit(
    kind: ParticleKind,
    n: number,
    rng: SeededRandom,
    init: (s: SpawnParams, r: SeededRandom, i: number) => void,
  ): number {
    const room = Math.max(0, Math.min(this.limit, this.capacity) - this.count);
    const m = Math.min(n, room);
    this.dropped += n - m;
    const k = PARTICLE_KINDS.indexOf(kind);
    const s = this.scratch;
    for (let i = 0; i < m; i++) {
      s.x = s.y = s.z = s.vx = s.vy = s.vz = 0;
      s.size = 1;
      s.life = 1;
      init(s, rng, i);
      const j = this.count++;
      this.px[j] = s.x;
      this.py[j] = s.y;
      this.pz[j] = s.z;
      this.vx[j] = s.vx;
      this.vy[j] = s.vy;
      this.vz[j] = s.vz;
      this.size[j] = s.size;
      this.age[j] = 0;
      this.life[j] = Math.max(0.01, s.life);
      this.kind[j] = k;
    }
    return m;
  }

  update(dt: number, env: ParticleEnvironment): void {
    if (dt <= 0) return;
    const params = PARTICLE_KINDS.map((k) => KIND_PARAMS[k]);
    let i = 0;
    while (i < this.count) {
      const p = params[this.kind[i]!]!;
      this.age[i]! += dt;
      let dead = this.age[i]! >= this.life[i]!;
      if (!dead) {
        const damp = Math.max(0, 1 - p.drag * dt);
        this.vx[i] = this.vx[i]! * damp;
        this.vy[i] = (this.vy[i]! - env.gravity * p.gravity * dt) * damp;
        this.vz[i] = this.vz[i]! * damp;
        const x = (this.px[i]! += this.vx[i]! * dt);
        const y = (this.py[i]! += this.vy[i]! * dt);
        const z = (this.pz[i]! += this.vz[i]! * dt);
        this.size[i] = Math.max(0.02, this.size[i]! + p.growth * dt);
        if (p.waterBehaviour !== 'ignore') {
          const w = env.waterHeight(x, z);
          if (p.waterBehaviour === 'die-below' && y < w) dead = true;
          else if (p.waterBehaviour === 'die-above' && y > w - 0.2) dead = true;
          else if (p.waterBehaviour === 'float') this.py[i] = w + 0.1;
        }
        if (!dead && y < env.groundHeight(x, z) && p.waterBehaviour !== 'die-above') {
          if (p.gravity > 0) dead = true;
          else this.py[i] = env.groundHeight(x, z) + 0.05;
        }
      }
      if (dead) this.remove(i);
      else i++;
    }
  }

  private remove(i: number): void {
    const last = --this.count;
    if (i === last) return;
    this.px[i] = this.px[last]!;
    this.py[i] = this.py[last]!;
    this.pz[i] = this.pz[last]!;
    this.vx[i] = this.vx[last]!;
    this.vy[i] = this.vy[last]!;
    this.vz[i] = this.vz[last]!;
    this.size[i] = this.size[last]!;
    this.age[i] = this.age[last]!;
    this.life[i] = this.life[last]!;
    this.kind[i] = this.kind[last]!;
  }

  clear(): void {
    this.count = 0;
  }

  countByKind(): Record<ParticleKind, number> {
    const out = Object.fromEntries(PARTICLE_KINDS.map((k) => [k, 0])) as Record<ParticleKind, number>;
    for (let i = 0; i < this.count; i++) out[PARTICLE_KINDS[this.kind[i]!]!]++;
    return out;
  }
}
