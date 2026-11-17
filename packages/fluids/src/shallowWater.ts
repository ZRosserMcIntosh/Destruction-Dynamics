// Near-field interactive water: linearised shallow-water equations on a
// staggered (Arakawa C) grid around the player or an active event.
//
//   ∂η/∂t = -∇·(D u)          (continuity; η surface elevation, D still-water depth)
//   ∂u/∂t = -g ∇(η + P) - γu   (momentum; P a pressure head from bodies pushing on the water)
//
// The wave speed varies with depth (√(gD)), so impact waves slow and steepen
// toward the beach, and land cells act as walls. A sponge layer absorbs waves
// at the domain edge. Cost is O(N²) per substep with no allocation.

import type { DepthFunction } from './types';

export const G = 9.81;

export interface NearFieldOptions {
  /** Cells per side. */
  size: number;
  /** World extent per side, metres. */
  extent: number;
  /** Domain centre in the scene frame. */
  centerX: number;
  centerZ: number;
  depth: DepthFunction;
  /** Depths are clamped to this to bound the wave speed (and the CFL step). */
  maxDepth?: number;
  /** Velocity damping, 1/s. */
  damping?: number;
  /** Sponge width as a fraction of the domain. */
  spongeFraction?: number;
  /** Foam decay time constant, s. */
  foamLifetime?: number;
}

export class NearFieldSolver {
  readonly n: number;
  readonly extent: number;
  readonly dx: number;
  readonly originX: number;
  readonly originZ: number;
  readonly centerX: number;
  readonly centerZ: number;
  /** Surface elevation at cell centres. */
  readonly eta: Float32Array;
  /** Foam coverage 0..1 at cell centres. */
  readonly foam: Float32Array;
  /** x-face velocities, (n+1) × n. */
  readonly u: Float32Array;
  /** z-face velocities, n × (n+1). */
  readonly w: Float32Array;
  /** Pressure head accumulated for the next step. */
  readonly pressure: Float32Array;
  /** Still-water depth per cell (0 = land). */
  readonly depth: Float32Array;
  private readonly faceDepthU: Float32Array;
  private readonly faceDepthW: Float32Array;
  private readonly sponge: Float32Array;
  private readonly foamScratch: Float32Array;
  private readonly damping: number;
  private readonly foamLifetime: number;
  readonly maxStableDt: number;
  /** Substeps taken by the last step() call (instrumentation). */
  lastSubsteps = 0;

  constructor(o: NearFieldOptions) {
    const n = (this.n = Math.max(8, Math.floor(o.size)));
    this.extent = o.extent;
    this.dx = o.extent / n;
    this.centerX = o.centerX;
    this.centerZ = o.centerZ;
    this.originX = o.centerX - o.extent / 2;
    this.originZ = o.centerZ - o.extent / 2;
    this.eta = new Float32Array(n * n);
    this.foam = new Float32Array(n * n);
    this.foamScratch = new Float32Array(n * n);
    this.pressure = new Float32Array(n * n);
    this.depth = new Float32Array(n * n);
    this.sponge = new Float32Array(n * n);
    this.u = new Float32Array((n + 1) * n);
    this.w = new Float32Array(n * (n + 1));
    this.faceDepthU = new Float32Array((n + 1) * n);
    this.faceDepthW = new Float32Array(n * (n + 1));
    this.damping = o.damping ?? 0.04;
    this.foamLifetime = o.foamLifetime ?? 6;
    const maxDepth = o.maxDepth ?? 60;

    let dMax = 1;
    const spongeCells = Math.max(2, Math.floor(n * (o.spongeFraction ?? 0.08)));
    for (let j = 0; j < n; j++)
      for (let i = 0; i < n; i++) {
        const x = this.originX + (i + 0.5) * this.dx;
        const z = this.originZ + (j + 0.5) * this.dx;
        const d = o.depth(x, z);
        const dc = d < 0.5 ? 0 : Math.min(d, maxDepth);
        this.depth[j * n + i] = dc;
        dMax = Math.max(dMax, dc);
        const edge = Math.min(i, j, n - 1 - i, n - 1 - j);
        const s = edge < spongeCells ? 1 - edge / spongeCells : 0;
        this.sponge[j * n + i] = 2.5 * s * s;
      }
    // Face depth: the shallower neighbour; zero if either side is land or outside.
    for (let j = 0; j < n; j++)
      for (let i = 0; i <= n; i++) {
        const a = i > 0 ? this.depth[j * n + i - 1]! : 0;
        const b = i < n ? this.depth[j * n + i]! : 0;
        this.faceDepthU[j * (n + 1) + i] = a > 0 && b > 0 ? Math.min(a, b) : 0;
      }
    for (let j = 0; j <= n; j++)
      for (let i = 0; i < n; i++) {
        const a = j > 0 ? this.depth[(j - 1) * n + i]! : 0;
        const b = j < n ? this.depth[j * n + i]! : 0;
        this.faceDepthW[j * n + i] = a > 0 && b > 0 ? Math.min(a, b) : 0;
      }
    this.maxStableDt = (0.45 * this.dx) / Math.sqrt(G * dMax);
  }

  /** Fractional cell coordinates of a scene point. */
  cellCoords(x: number, z: number): { fx: number; fz: number } {
    return { fx: (x - this.originX) / this.dx - 0.5, fz: (z - this.originZ) / this.dx - 0.5 };
  }

  contains(x: number, z: number): boolean {
    return (
      x >= this.originX &&
      z >= this.originZ &&
      x <= this.originX + this.extent &&
      z <= this.originZ + this.extent
    );
  }

  /** Bilinear sample of a cell-centred field; 0 outside the domain. */
  sampleField(field: Float32Array, x: number, z: number): number {
    const n = this.n;
    const { fx, fz } = this.cellCoords(x, z);
    if (fx < 0 || fz < 0 || fx > n - 1 || fz > n - 1) return 0;
    const i = Math.min(n - 2, Math.floor(fx));
    const j = Math.min(n - 2, Math.floor(fz));
    const tx = fx - i;
    const tz = fz - j;
    const k = j * n + i;
    const a = field[k]! * (1 - tx) + field[k + 1]! * tx;
    const b = field[k + n]! * (1 - tx) + field[k + n + 1]! * tx;
    return a * (1 - tz) + b * tz;
  }

  /** Cell-centred current at (x, z), m/s. */
  velocityAt(x: number, z: number): { x: number; z: number } {
    const n = this.n;
    const { fx, fz } = this.cellCoords(x, z);
    const i = Math.round(fx);
    const j = Math.round(fz);
    if (i < 0 || j < 0 || i >= n || j >= n) return { x: 0, z: 0 };
    return {
      x: 0.5 * (this.u[j * (n + 1) + i]! + this.u[j * (n + 1) + i + 1]!),
      z: 0.5 * (this.w[j * n + i]! + this.w[(j + 1) * n + i]!),
    };
  }

  /**
   * Adds a pressure head (metres of water) over a disc with a soft edge for the
   * next step. Positive pushes the surface down and drives water outward.
   */
  addPressureDisc(x: number, z: number, radius: number, head: number): void {
    const n = this.n;
    const r = radius + 2 * this.dx;
    const { fx, fz } = this.cellCoords(x, z);
    const ri = Math.ceil(r / this.dx);
    const i0 = Math.max(0, Math.floor(fx) - ri);
    const i1 = Math.min(n - 1, Math.ceil(fx) + ri);
    const j0 = Math.max(0, Math.floor(fz) - ri);
    const j1 = Math.min(n - 1, Math.ceil(fz) + ri);
    for (let j = j0; j <= j1; j++)
      for (let i = i0; i <= i1; i++) {
        const cx = this.originX + (i + 0.5) * this.dx - x;
        const cz = this.originZ + (j + 0.5) * this.dx - z;
        const d = Math.sqrt(cx * cx + cz * cz);
        if (d >= r) continue;
        const s = d <= radius ? 1 : 0.5 + 0.5 * Math.cos((Math.PI * (d - radius)) / (r - radius));
        this.pressure[j * n + i]! += head * s;
      }
  }

  /** Adds a pressure head over a rotated box with soft edges. */
  addPressureBox(
    x: number,
    z: number,
    halfLength: number,
    halfWidth: number,
    yaw: number,
    head: number,
  ): void {
    const n = this.n;
    const soft = Math.max(2 * this.dx, 0.15 * halfWidth);
    const reach = Math.hypot(halfLength, halfWidth) + soft;
    const { fx, fz } = this.cellCoords(x, z);
    const ri = Math.ceil(reach / this.dx);
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    for (let j = Math.max(0, Math.floor(fz) - ri); j <= Math.min(n - 1, Math.ceil(fz) + ri); j++)
      for (let i = Math.max(0, Math.floor(fx) - ri); i <= Math.min(n - 1, Math.ceil(fx) + ri); i++) {
        const px = this.originX + (i + 0.5) * this.dx - x;
        const pz = this.originZ + (j + 0.5) * this.dx - z;
        const lx = Math.abs(c * px + s * pz) - halfLength;
        const lz = Math.abs(-s * px + c * pz) - halfWidth;
        const out = Math.max(lx, lz);
        if (out >= soft) continue;
        const f = out <= 0 ? 1 : 0.5 + 0.5 * Math.cos((Math.PI * out) / soft);
        this.pressure[j * n + i]! += head * f;
      }
  }

  /** Adds horizontal momentum (m/s per second) over a disc for the next step. */
  addCurrent(x: number, z: number, radius: number, ax: number, az: number, dt: number): void {
    const n = this.n;
    const { fx, fz } = this.cellCoords(x, z);
    const ri = Math.ceil(radius / this.dx);
    for (let j = Math.max(0, Math.floor(fz) - ri); j <= Math.min(n - 1, Math.ceil(fz) + ri); j++)
      for (let i = Math.max(1, Math.floor(fx) - ri); i <= Math.min(n - 1, Math.ceil(fx) + ri); i++) {
        const px = this.originX + (i + 0.5) * this.dx - x;
        const pz = this.originZ + (j + 0.5) * this.dx - z;
        const f = 1 - Math.hypot(px, pz) / radius;
        if (f <= 0) continue;
        if (this.faceDepthU[j * (n + 1) + i]! > 0) this.u[j * (n + 1) + i]! += ax * f * dt;
        if (this.faceDepthW[j * n + i]! > 0) this.w[j * n + i]! += az * f * dt;
      }
  }

  /** Advances by dt using as many stable substeps as needed (capped). Clears pressure. */
  step(dt: number, maxSubsteps = 12): void {
    if (dt <= 0) return;
    const sub = Math.min(maxSubsteps, Math.max(1, Math.ceil(dt / this.maxStableDt)));
    const h = Math.min(dt / sub, this.maxStableDt);
    for (let s = 0; s < sub; s++) this.substep(h);
    this.updateFoam(dt);
    this.pressure.fill(0);
    this.lastSubsteps = sub;
  }

  private substep(h: number): void {
    const n = this.n;
    const n1 = n + 1;
    const { eta, u, w, pressure: p, faceDepthU: du, faceDepthW: dw, sponge } = this;
    const k = (G * h) / this.dx;
    const damp = Math.max(0, 1 - this.damping * h);
    // Momentum.
    for (let j = 0; j < n; j++) {
      const row = j * n;
      for (let i = 1; i < n; i++) {
        const f = j * n1 + i;
        if (du[f] === 0) {
          u[f] = 0;
          continue;
        }
        const grad = eta[row + i]! + p[row + i]! - eta[row + i - 1]! - p[row + i - 1]!;
        u[f] = (u[f]! - k * grad) * damp * (1 - h * sponge[row + i]!);
      }
    }
    for (let j = 1; j < n; j++) {
      for (let i = 0; i < n; i++) {
        const f = j * n + i;
        if (dw[f] === 0) {
          w[f] = 0;
          continue;
        }
        const grad = eta[f]! + p[f]! - eta[f - n]! - p[f - n]!;
        w[f] = (w[f]! - k * grad) * damp * (1 - h * sponge[f]!);
      }
    }
    // Continuity.
    const c = h / this.dx;
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        const cell = j * n + i;
        if (this.depth[cell] === 0) {
          eta[cell] = 0;
          continue;
        }
        const fu = j * n1 + i;
        const div =
          du[fu + 1]! * u[fu + 1]! - du[fu]! * u[fu]! + dw[cell + n]! * w[cell + n]! - dw[cell]! * w[cell]!;
        eta[cell] = (eta[cell]! - c * div) * (1 - h * sponge[cell]!);
      }
    }
  }

  private updateFoam(dt: number): void {
    const n = this.n;
    const n1 = n + 1;
    const { eta, foam, u, w, depth } = this;
    const src = this.foamScratch;
    const decay = Math.exp(-dt / this.foamLifetime);
    const invDx = 1 / this.dx;
    // Semi-Lagrangian advection along the current, then sources from steep or
    // strongly converging surface.
    for (let j = 0; j < n; j++)
      for (let i = 0; i < n; i++) {
        const cell = j * n + i;
        if (depth[cell] === 0) {
          src[cell] = 0;
          continue;
        }
        const vx = 0.5 * (u[j * n1 + i]! + u[j * n1 + i + 1]!);
        const vz = 0.5 * (w[cell]! + w[cell + n]!);
        const bx = Math.min(n - 1, Math.max(0, i - vx * dt * invDx));
        const bz = Math.min(n - 1, Math.max(0, j - vz * dt * invDx));
        const i0 = Math.min(n - 2, Math.floor(bx));
        const j0 = Math.min(n - 2, Math.floor(bz));
        const tx = bx - i0;
        const tz = bz - j0;
        const k = j0 * n + i0;
        const advected =
          (foam[k]! * (1 - tx) + foam[k + 1]! * tx) * (1 - tz) +
          (foam[k + n]! * (1 - tx) + foam[k + n + 1]! * tx) * tz;
        const gx = i > 0 && i < n - 1 ? (eta[cell + 1]! - eta[cell - 1]!) * 0.5 * invDx : 0;
        const gz = j > 0 && j < n - 1 ? (eta[cell + n]! - eta[cell - n]!) * 0.5 * invDx : 0;
        const slope = Math.sqrt(gx * gx + gz * gz);
        const crest = Math.max(0, eta[cell]! - 0.8);
        const speed = Math.sqrt(vx * vx + vz * vz);
        const source = Math.max(0, slope - 0.12) * 4 + crest * 0.25 + Math.max(0, speed - 2.5) * 0.08;
        src[cell] = Math.min(1, advected * decay + source * dt);
      }
    foam.set(src);
  }

  /** Total displaced volume Σ η dx² (mass conservation check), m³. */
  volume(): number {
    let s = 0;
    for (let k = 0; k < this.eta.length; k++) s += this.eta[k]!;
    return s * this.dx * this.dx;
  }

  maxAbsHeight(): number {
    let m = 0;
    for (let k = 0; k < this.eta.length; k++) m = Math.max(m, Math.abs(this.eta[k]!));
    return m;
  }

  reset(): void {
    this.eta.fill(0);
    this.foam.fill(0);
    this.u.fill(0);
    this.w.fill(0);
    this.pressure.fill(0);
  }
}
