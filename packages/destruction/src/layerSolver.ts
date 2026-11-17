// Floor-layer structural solver.
//
// After the authored fall, each structural layer (a floor, or a few floors at
// lower fidelity) becomes its own rigid body. Neighbouring layers are glued by
// breakable joints at the corners of their shared slab, so the tower comes
// apart floor by floor where the slam overloads it. Layers float while air is
// trapped, flood at seeded rates, sink, and pile up on the seabed.
//
// Extended position-based dynamics (XPBD) for rigid bodies (Müller et al.,
// "Detailed Rigid Body Simulation with Extended Position Based Dynamics",
// 2020): substepped, one constraint pass per substep. The step uses only
// + − × ÷ and sqrt, so for the same inputs it gives the same answer every run.
// It runs ahead of time and stores keyframes, so the result stays a pure
// function of event time like the rest of the collapse.

import {
  type Pose,
  type Quat,
  SeededRandom,
  type Vec3,
  add,
  cross,
  quatRotate,
  vec3,
} from '@destruction-dynamics/core';
import { type ConvexShape, type StaticObstacle, obstacleBounds, shapeEdgePoints } from './shapes';

export interface LayerBodyInit {
  /** Pose of the layer's origin (bottom centre of its box). */
  pose: Pose;
  /** Velocity of the origin and angular velocity at release. */
  velocity: Vec3;
  angularVelocity: Vec3;
  halfExtents: Vec3;
}

export interface LayerSolverOptions {
  groundHeight: (x: number, z: number) => number;
  seaLevel: number;
  seed: number;
  /** Bulk density of a layer (structure plus trapped air), kg/m³. */
  density?: number;
  /** Seconds simulated after release. */
  duration?: number;
  /** Frame rate of the stored keyframes. */
  keyframeHz?: number;
  /** Physics steps per second and XPBD substeps per step. */
  stepHz?: number;
  substeps?: number;
  /** Joint stretch (m) beyond which a slab connection breaks: [min, max], seeded per joint. */
  jointStrength?: [number, number];
  /** Corner connection capacity, in multiples of the lighter layer's weight: [min, max], seeded. */
  jointCapacity?: [number, number];
  /** Immovable, indestructible convex obstacles the layers collide with. */
  obstacles?: StaticObstacle[];
}

export interface LayerSimulationResult {
  /** Keyframe rate, Hz. */
  readonly hz: number;
  readonly frames: number;
  readonly layers: number;
  /** frames × layers × 7 (origin x, y, z, quaternion x, y, z, w). */
  readonly data: Float32Array;
  /** Seconds after release at which each joint broke (Infinity = held). */
  readonly jointBreakTimes: Float32Array;
  /** Seconds after release when every layer had come to rest (or the duration). */
  readonly settledAfter: number;
  /** Wall-clock milliseconds the precompute took. */
  readonly computeMs: number;
  /** Substeps run in total (adaptive). */
  readonly substeps?: number;
}

const G = 9.81;
const WATER = 1025;
const SIGNS = [-1, 1] as const;

const MAX_SPEED = 70;
/** Fastest speed (m/s) at which a contact may push overlapping bodies apart. */
const MAX_DEPENETRATION = 6;
/** Velocity pass: bounce kept from real impacts (concrete barely bounces). */
const RESTITUTION = 0.1;
/** Dynamic friction coefficient (structure on structure, ground and obstacles). */
const FRICTION = 0.8;
const CONTACT_STRIDE = 13;
/** A layer sleeps after SLEEP_AFTER seconds below these speeds (m/s, rad/s). */
const SLEEP_SPEED = 0.3;
const SLEEP_SPIN = 0.06;
const SLEEP_AFTER = 1;
/** Adaptive substeps: at least this many, and a substep may move a layer at most this fraction of its thickness. */
const MIN_SUBSTEPS = 2;
const SUBSTEP_TRAVEL = 0.15;

function normalizeQuat(q: Quat): Quat {
  const l = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return { x: q.x / l, y: q.y / l, z: q.z / l, w: q.w / l };
}

/**
 * Structure-of-arrays solver. Hot loops use scalars only (no allocation);
 * bodies that come to rest sleep until something disturbs them.
 */
export class LayerSolver {
  private readonly n: number;
  private readonly o: Required<LayerSolverOptions>;
  // Per body.
  private readonly hx: Float64Array;
  private readonly hy: Float64Array;
  private readonly hz: Float64Array;
  private readonly im: Float64Array;
  private readonly iix: Float64Array;
  private readonly iiy: Float64Array;
  private readonly iiz: Float64Array;
  private readonly X: Float64Array; // x y z
  private readonly Q: Float64Array; // x y z w
  private readonly V: Float64Array;
  private readonly W: Float64Array;
  private readonly PX: Float64Array;
  private readonly PQ: Float64Array;
  private readonly flood: Float64Array;
  private readonly floodRate: Float64Array;
  private readonly restTime: Float64Array;
  private readonly asleep: Uint8Array;
  /**
   * Contacts found this substep, for the velocity pass. Per contact: a, b
   * (−1 = static), local point on a (3), local point on b (3), normal along
   * which a was pushed (3), positional multiplier λ, normal velocity before
   * the position solve.
   */
  private contacts = new Float64Array(CONTACT_STRIDE * 1024);
  private contactCount = 0;
  /** Set when a body touched the ground, an obstacle or another layer this substep. */
  private touching = new Uint8Array(0);
  /** One contact normal per body this substep, used to cap how fast contacts separate. */
  // Per joint (4 per interface between layer k and k + 1).
  private readonly jl: Float64Array; // la xyz, lb xyz
  private readonly jStrength: Float64Array;
  /** Force (N) at which each corner connection fails. */
  private readonly jForce: Float64Array;
  private readonly jBroken: Uint8Array;
  private readonly jBreakTime: Float32Array;
  private time = 0;
  /** Substeps actually run (adaptive). */
  substepsTaken = 0;
  /** Ground sampled once into a grid (bilinear lookups replace the scenario's function). */
  private readonly groundGrid: {
    x0: number;
    z0: number;
    step: number;
    nx: number;
    nz: number;
    h: Float32Array;
  };
  // Scratch outputs.
  private ox = 0;
  private oy = 0;
  private oz = 0;

  constructor(layers: readonly LayerBodyInit[], opts: LayerSolverOptions) {
    this.o = {
      density: 380,
      duration: 32,
      keyframeHz: 30,
      stepHz: 60,
      substeps: 4,
      jointStrength: [0.06, 0.9],
      obstacles: [],
      jointCapacity: [1.5, 8],
      ...opts,
    };
    const n = (this.n = layers.length);
    // Obstacles are bodies n.. with zero inverse mass and inertia: corrections never move them.
    const total = n + this.o.obstacles.length;
    const f = () => new Float64Array(total);
    this.hx = f();
    this.hy = f();
    this.hz = f();
    this.im = f();
    this.iix = f();
    this.iiy = f();
    this.iiz = f();
    this.X = new Float64Array(total * 3);
    this.Q = new Float64Array(total * 4);
    this.V = new Float64Array(total * 3);
    this.W = new Float64Array(total * 3);
    this.PX = new Float64Array(total * 3);
    this.PQ = new Float64Array(total * 4);
    this.flood = f();
    this.floodRate = f();
    this.restTime = f();
    this.asleep = new Uint8Array(total);
    this.touching = new Uint8Array(total);
    const rng = new SeededRandom(opts.seed);
    layers.forEach((l, i) => {
      const he = l.halfExtents;
      const mass = this.o.density * 8 * he.x * he.y * he.z;
      this.hx[i] = he.x;
      this.hy[i] = he.y;
      this.hz[i] = he.z;
      this.im[i] = 1 / mass;
      this.iix[i] = 3 / (mass * (he.y * he.y + he.z * he.z));
      this.iiy[i] = 3 / (mass * (he.x * he.x + he.z * he.z));
      this.iiz[i] = 3 / (mass * (he.x * he.x + he.y * he.y));
      const off = quatRotate(l.pose.quaternion, vec3(0, he.y, 0));
      const c = add(l.pose.position, off);
      const v = add(l.velocity, cross(l.angularVelocity, off));
      this.X.set([c.x, c.y, c.z], i * 3);
      const q = l.pose.quaternion;
      this.Q.set([q.x, q.y, q.z, q.w], i * 4);
      this.V.set([v.x, v.y, v.z], i * 3);
      this.W.set([l.angularVelocity.x, l.angularVelocity.y, l.angularVelocity.z], i * 3);
      this.floodRate[i] = 1 / rng.range(2.5, 11);
    });
    this.o.obstacles.forEach((ob, k) => {
      const i = n + k;
      this.X.set([ob.position.x, ob.position.y, ob.position.z], i * 3);
      this.Q.set([ob.quaternion.x, ob.quaternion.y, ob.quaternion.z, ob.quaternion.w], i * 4);
      this.obstacleSphere.push(obstacleBounds(ob));
    });
    if (this.o.obstacles.length > 0) {
      this.layerLattice = Array.from({ length: n }, (_, i) => this.latticeSamples(i));
      const thinnest = Math.min(...Array.from(this.hy.subarray(0, n)), ...Array.from(this.hx.subarray(0, n)));
      this.obstacleEdges = this.o.obstacles.map((ob) =>
        Float64Array.from(
          shapeEdgePoints(ob.shape, Math.max(1.5, 2 * thinnest)).flatMap((p) => [p.x, p.y, p.z]),
        ),
      );
      this.obstacleEdgesWorld = this.obstacleEdges.map((pts, k) => {
        const out = new Float64Array(pts.length);
        for (let p = 0; p < pts.length; p += 3) {
          this.rot(n + k, pts[p]!, pts[p + 1]!, pts[p + 2]!);
          out[p] = this.X[(n + k) * 3]! + this.ox;
          out[p + 1] = this.X[(n + k) * 3 + 1]! + this.oy;
          out[p + 2] = this.X[(n + k) * 3 + 2]! + this.oz;
        }
        return out;
      });
    }
    this.groundGrid = this.sampleGround(layers);
    const joints = Math.max(0, n - 1) * 4;
    this.jl = new Float64Array(joints * 6);
    this.jStrength = new Float64Array(joints);
    this.jForce = new Float64Array(joints);
    this.jBroken = new Uint8Array(joints);
    this.jBreakTime = new Float32Array(joints).fill(Infinity);
    const [smin, smax] = this.o.jointStrength;
    let j = 0;
    for (let k = 0; k + 1 < n; k++)
      for (const sx of SIGNS)
        for (const sz of SIGNS) {
          const cx = sx * Math.min(this.hx[k]!, this.hx[k + 1]!) * 0.92;
          const cz = sz * Math.min(this.hz[k]!, this.hz[k + 1]!) * 0.92;
          this.jl.set([cx, this.hy[k]!, cz, cx, -this.hy[k + 1]!, cz], j * 6);
          this.jStrength[j] = smin + (smax - smin) * rng.next() * rng.next();
          // Capacity as a multiple of the lighter neighbour's weight, so a joint
          // means the same thing whether a layer is one floor or twenty.
          const lighter = Math.min(1 / this.im[k]!, 1 / this.im[k + 1]!);
          const [fmin, fmax] = this.o.jointCapacity;
          this.jForce[j] = lighter * G * (fmin + (fmax - fmin) * rng.next());
          j++;
        }
  }

  /** Samples the ground over everywhere the layers can reach (1 m grid). */
  private sampleGround(layers: readonly LayerBodyInit[]) {
    const reach =
      150 +
      Math.max(0, ...layers.map((l) => 2 * Math.max(l.halfExtents.x, l.halfExtents.y, l.halfExtents.z)));
    const xs = layers.map((l) => l.pose.position.x);
    const zs = layers.map((l) => l.pose.position.z);
    const step = 1;
    const x0 = Math.floor(Math.min(...xs, 0) - reach),
      z0 = Math.floor(Math.min(...zs, 0) - reach);
    const nx = Math.ceil(Math.max(...xs, 0) + reach - x0) + 2,
      nz = Math.ceil(Math.max(...zs, 0) + reach - z0) + 2;
    const h = new Float32Array(nx * nz);
    for (let j = 0; j < nz; j++)
      for (let i = 0; i < nx; i++) h[j * nx + i] = this.o.groundHeight(x0 + i * step, z0 + j * step);
    return { x0, z0, step, nx, nz, h };
  }

  /** Ground height from the cached grid (falls back to the function outside it). */
  private ground(x: number, z: number): number {
    const g = this.groundGrid;
    const fx = (x - g.x0) / g.step,
      fz = (z - g.z0) / g.step;
    if (fx < 0 || fz < 0 || fx >= g.nx - 1 || fz >= g.nz - 1) return this.o.groundHeight(x, z);
    const i = fx | 0,
      j = fz | 0;
    const tx = fx - i,
      tz = fz - j;
    const k = j * g.nx + i;
    const a = g.h[k]! + (g.h[k + 1]! - g.h[k]!) * tx;
    const b = g.h[k + g.nx]! + (g.h[k + g.nx + 1]! - g.h[k + g.nx]!) * tx;
    return a + (b - a) * tz;
  }

  // -- scalar helpers (results in ox, oy, oz) --------------------------------

  /** Rotate (x, y, z) by body i's quaternion (sign −1: inverse); `Q` may be the previous-substep array. */
  private rot(i: number, x: number, y: number, z: number, inverse = false, Q: Float64Array = this.Q): void {
    const qx = inverse ? -Q[i * 4]! : Q[i * 4]!;
    const qy = inverse ? -Q[i * 4 + 1]! : Q[i * 4 + 1]!;
    const qz = inverse ? -Q[i * 4 + 2]! : Q[i * 4 + 2]!;
    const qw = Q[i * 4 + 3]!;
    const tx = 2 * (qy * z - qz * y);
    const ty = 2 * (qz * x - qx * z);
    const tz = 2 * (qx * y - qy * x);
    this.ox = x + qw * tx + (qy * tz - qz * ty);
    this.oy = y + qw * ty + (qz * tx - qx * tz);
    this.oz = z + qw * tz + (qx * ty - qy * tx);
  }

  /** World position of body i's local point at the start of the substep (into ox, oy, oz). */
  private prevWorld(i: number, lx: number, ly: number, lz: number): void {
    this.rot(i, lx, ly, lz, false, this.PQ);
    this.ox += this.PX[i * 3]!;
    this.oy += this.PX[i * 3 + 1]!;
    this.oz += this.PX[i * 3 + 2]!;
  }

  /**
   * Split a penetration into the part this substep's motion created (resolved
   * in full, so fast bodies stop at the surface) and overlap that already
   * existed (released slowly, so it never turns into a launch).
   */
  private allowed(depth: number, prevDepth: number, h: number): number {
    const old = Math.max(0, Math.min(depth, prevDepth));
    return depth - old + Math.min(old, MAX_DEPENETRATION * h);
  }

  /** World inverse inertia of body i applied to (x, y, z). */
  private invI(i: number, x: number, y: number, z: number): void {
    this.rot(i, x, y, z, true);
    this.rot(i, this.ox * this.iix[i]!, this.oy * this.iiy[i]!, this.oz * this.iiz[i]!);
  }

  /** q_i += s·½[r, 0]·q_i, normalised. */
  private spin(i: number, rx: number, ry: number, rz: number, s: number): void {
    const Q = this.Q;
    const k = i * 4;
    const qx = Q[k]!,
      qy = Q[k + 1]!,
      qz = Q[k + 2]!,
      qw = Q[k + 3]!;
    const h = 0.5 * s;
    const nx = qx + h * (rx * qw + ry * qz - rz * qy);
    const ny = qy + h * (ry * qw + rz * qx - rx * qz);
    const nz = qz + h * (rz * qw + rx * qy - ry * qx);
    const nw = qw + h * (-rx * qx - ry * qy - rz * qz);
    const l = 1 / Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
    Q[k] = nx * l;
    Q[k + 1] = ny * l;
    Q[k + 2] = nz * l;
    Q[k + 3] = nw * l;
  }

  /** Generalised inverse mass of body i for offset r along n. */
  private gim(i: number, rx: number, ry: number, rz: number, nx: number, ny: number, nz: number): number {
    const cx = ry * nz - rz * ny,
      cy = rz * nx - rx * nz,
      cz = rx * ny - ry * nx;
    this.invI(i, cx, cy, cz);
    return this.im[i]! + cx * this.ox + cy * this.oy + cz * this.oz;
  }

  /** Applies impulse-like correction p at offset r on body i (sign ±1). */
  private push(
    i: number,
    rx: number,
    ry: number,
    rz: number,
    px: number,
    py: number,
    pz: number,
    s: number,
  ): void {
    const m = this.im[i]! * s;
    this.X[i * 3]! += px * m;
    this.X[i * 3 + 1]! += py * m;
    this.X[i * 3 + 2]! += pz * m;
    this.invI(i, ry * pz - rz * py, rz * px - rx * pz, rx * py - ry * px);
    this.spin(i, this.ox, this.oy, this.oz, s);
  }

  /** XPBD positional correction of size c along unit n between A (+) and optional B (−). */
  private correct(
    a: number,
    rax: number,
    ray: number,
    raz: number,
    b: number,
    rbx: number,
    rby: number,
    rbz: number,
    nx: number,
    ny: number,
    nz: number,
    c: number,
    alpha: number,
    h: number,
  ): number {
    const wa = this.gim(a, rax, ray, raz, nx, ny, nz);
    const wb = b >= 0 ? this.gim(b, rbx, rby, rbz, nx, ny, nz) : 0;
    const dl = c / (wa + wb + alpha / (h * h));
    this.push(a, rax, ray, raz, nx * dl, ny * dl, nz * dl, 1);
    if (b >= 0) this.push(b, rbx, rby, rbz, nx * dl, ny * dl, nz * dl, -1);
    return dl;
  }

  private wake(i: number): void {
    this.asleep[i] = 0;
    this.restTime[i] = 0;
  }

  // -- the step ---------------------------------------------------------------

  private substep(h: number): void {
    const { n, X, Q, V, W, PX, PQ } = this;
    const sea = this.o.seaLevel;
    for (let i = 0; i < n; i++) {
      PX[i * 3] = X[i * 3]!;
      PX[i * 3 + 1] = X[i * 3 + 1]!;
      PX[i * 3 + 2] = X[i * 3 + 2]!;
      PQ.set(Q.subarray(i * 4, i * 4 + 4), i * 4);
      if (this.asleep[i]) continue;
      // Buoyancy from the corners below the sea surface.
      let lo = Infinity,
        hi = -Infinity,
        sx = 0,
        sy = 0,
        sz = 0,
        under = 0;
      for (const cx of SIGNS)
        for (const cy of SIGNS)
          for (const cz of SIGNS) {
            this.rot(i, cx * this.hx[i]!, cy * this.hy[i]!, cz * this.hz[i]!);
            const py = X[i * 3 + 1]! + this.oy;
            if (py < lo) lo = py;
            if (py > hi) hi = py;
            if (py < sea) {
              sx += this.ox;
              sy += this.oy;
              sz += this.oz;
              under++;
            }
          }
      const frac = hi > lo ? Math.min(1, Math.max(0, (sea - lo) / (hi - lo))) : 0;
      let ay = -G;
      if (frac > 0) {
        const vol = 8 * this.hx[i]! * this.hy[i]! * this.hz[i]!;
        const lift = WATER * G * vol * frac * (1 - this.flood[i]!);
        ay += lift * this.im[i]!;
        // Torque about the centre from lift at the centre of buoyancy (r × (0, lift, 0)).
        const rx = (0.5 * sx) / under,
          rz = (0.5 * sz) / under;
        this.invI(i, -rz * lift, 0, rx * lift);
        W[i * 3]! += this.ox * h;
        W[i * 3 + 1]! += this.oy * h;
        W[i * 3 + 2]! += this.oz * h;
        if (frac > 0.3) this.flood[i] = Math.min(1, this.flood[i]! + this.floodRate[i]! * h * frac);
        const dv = Math.max(0, 1 - 1.4 * frac * h);
        const dw = Math.max(0, 1 - 1.8 * frac * h);
        for (let k = 0; k < 3; k++) {
          V[i * 3 + k]! *= dv;
          W[i * 3 + k]! *= dw;
        }
        void sy;
      }
      V[i * 3 + 1]! += ay * h;
      X[i * 3]! += V[i * 3]! * h;
      X[i * 3 + 1]! += V[i * 3 + 1]! * h;
      X[i * 3 + 2]! += V[i * 3 + 2]! * h;
      this.spin(i, W[i * 3]!, W[i * 3 + 1]!, W[i * 3 + 2]!, h);
    }

    // Slab joints.
    for (let j = 0; j < this.jStrength.length; j++) {
      if (this.jBroken[j]) continue;
      const a = j >> 2;
      const b = a + 1;
      if (this.asleep[a] && this.asleep[b]) continue;
      const L = this.jl;
      this.rot(a, L[j * 6]!, L[j * 6 + 1]!, L[j * 6 + 2]!);
      const rax = this.ox,
        ray = this.oy,
        raz = this.oz;
      this.rot(b, L[j * 6 + 3]!, L[j * 6 + 4]!, L[j * 6 + 5]!);
      const rbx = this.ox,
        rby = this.oy,
        rbz = this.oz;
      const dx = X[b * 3]! + rbx - X[a * 3]! - rax;
      const dy = X[b * 3 + 1]! + rby - X[a * 3 + 1]! - ray;
      const dz = X[b * 3 + 2]! + rbz - X[a * 3 + 2]! - raz;
      const c = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (c > this.jStrength[j]!) {
        this.jBroken[j] = 1;
        this.jBreakTime[j] = this.time;
        continue;
      }
      if (c <= 1e-9) continue;
      // The multiplier over h² is the force the joint carried this substep:
      // slab connections fail by load, not by how far a stiff joint stretched.
      const force =
        this.correct(a, rax, ray, raz, b, rbx, rby, rbz, dx / c, dy / c, dz / c, c, 1e-8, h) / (h * h);
      if (force > this.jForce[j]!) {
        this.jBroken[j] = 1;
        this.jBreakTime[j] = this.time;
      }
    }

    // Ground contact with static friction.
    for (let i = 0; i < n; i++) {
      if (this.asleep[i]) continue;
      for (const cx of SIGNS)
        for (const cy of SIGNS)
          for (const cz of SIGNS) {
            const lx = cx * this.hx[i]!,
              ly = cy * this.hy[i]!,
              lz = cz * this.hz[i]!;
            this.rot(i, lx, ly, lz);
            let rx = this.ox,
              ry = this.oy,
              rz = this.oz;
            const px = X[i * 3]! + rx,
              pz = X[i * 3 + 2]! + rz;
            const g = this.ground(px, pz);
            const depth = g - (X[i * 3 + 1]! + ry);
            if (depth <= 0) continue;
            this.prevWorld(i, lx, ly, lz);
            const prevDepth = this.ground(this.ox, this.oz) - this.oy;
            const lam = this.correct(
              i,
              rx,
              ry,
              rz,
              -1,
              0,
              0,
              0,
              0,
              1,
              0,
              this.allowed(depth, prevDepth, h),
              0,
              h,
            );
            this.record(i, lx, ly, lz, -1, 0, 0, 0, 0, 1, 0, lam);
            // Undo tangential slip of this corner during the substep (static friction).
            this.rot(i, lx, ly, lz);
            rx = this.ox;
            ry = this.oy;
            rz = this.oz;
            const qx = PQ[i * 4]!,
              qy = PQ[i * 4 + 1]!,
              qz = PQ[i * 4 + 2]!,
              qw = PQ[i * 4 + 3]!;
            const tx = 2 * (qy * lz - qz * ly),
              tz = 2 * (qx * ly - qy * lx),
              ty = 2 * (qz * lx - qx * lz);
            const prx = lx + qw * tx + (qy * tz - qz * ty);
            const prz = lz + qw * tz + (qx * ty - qy * tx);
            const slx = X[i * 3]! + rx - (PX[i * 3]! + prx);
            const slz = X[i * 3 + 2]! + rz - (PX[i * 3 + 2]! + prz);
            const sl = Math.sqrt(slx * slx + slz * slz);
            if (sl > 1e-9 && sl < 0.6 * (depth + 0.02))
              this.correct(i, rx, ry, rz, -1, 0, 0, 0, -slx / sl, 0, -slz / sl, sl, 0, h);
          }
    }

    // Contacts between nearby layers whose slab joints have failed.
    for (let i = 0; i < n; i++)
      for (let k = i + 1; k <= Math.min(n - 1, i + 2); k++) {
        if (this.asleep[i] && this.asleep[k]) continue;
        if (k === i + 1 && this.interfaceIntact(i)) continue;
        this.boxContact(i, k, h);
        this.boxContact(k, i, h);
      }
    // Layers against obstacles, both ways: layer surface points inside the
    // block, and the block's edges cutting through a layer.
    for (let o = n; o < this.hx.length; o++) {
      const local = this.obstacleEdges[o - n]!;
      const world = this.obstacleEdgesWorld[o - n]!;
      const sphere = this.obstacleSphere[o - n]!;
      const shape = this.o.obstacles[o - n]!.shape;
      for (let i = 0; i < n; i++) {
        const lr = Math.sqrt(this.hx[i]! ** 2 + this.hy[i]! ** 2 + this.hz[i]! ** 2);
        const bx = X[i * 3]! - sphere.center.x,
          by = X[i * 3 + 1]! - sphere.center.y,
          bz = X[i * 3 + 2]! - sphere.center.z;
        if (bx * bx + by * by + bz * bz > (lr + sphere.radius) ** 2) continue;
        // Exact early-out: the layer's sphere is entirely outside one face plane.
        this.rot(
          o,
          X[i * 3]! - X[o * 3]!,
          X[i * 3 + 1]! - X[o * 3 + 1]!,
          X[i * 3 + 2]! - X[o * 3 + 2]!,
          true,
        );
        let outside = false;
        for (const pl of shape.planes)
          if (pl.n.x * this.ox + pl.n.y * this.oy + pl.n.z * this.oz - pl.d > lr) {
            outside = true;
            break;
          }
        if (outside) continue;
        const lat = this.layerLattice[i]!;
        for (let p = 0; p < lat.length; p += 3)
          this.convexContact(i, o, shape, lat[p]!, lat[p + 1]!, lat[p + 2]!, h);
        // The obstacle never moves: its edge points are fixed in the world, so
        // reject them against the layer's bounding sphere before any rotation.
        const cx = X[i * 3]!,
          cy = X[i * 3 + 1]!,
          cz = X[i * 3 + 2]!;
        const r2 = this.hx[i]! ** 2 + this.hy[i]! ** 2 + this.hz[i]! ** 2;
        for (let p = 0; p < world.length; p += 3) {
          const dx = world[p]! - cx,
            dy = world[p + 1]! - cy,
            dz = world[p + 2]! - cz;
          if (dx * dx + dy * dy + dz * dz < r2)
            this.pointContact(o, i, local[p]!, local[p + 1]!, local[p + 2]!, h);
        }
      }
    }

    // Velocities from positions.
    for (let i = 0; i < n; i++) {
      if (this.asleep[i]) continue;
      V[i * 3] = (X[i * 3]! - PX[i * 3]!) / h;
      V[i * 3 + 1] = (X[i * 3 + 1]! - PX[i * 3 + 1]!) / h;
      V[i * 3 + 2] = (X[i * 3 + 2]! - PX[i * 3 + 2]!) / h;
      // ω from Δq = q · conj(q_prev).
      const ax = Q[i * 4]!,
        ay = Q[i * 4 + 1]!,
        az = Q[i * 4 + 2]!,
        aw = Q[i * 4 + 3]!;
      const bx = -PQ[i * 4]!,
        by = -PQ[i * 4 + 1]!,
        bz = -PQ[i * 4 + 2]!,
        bw = PQ[i * 4 + 3]!;
      const dw = aw * bw - ax * bx - ay * by - az * bz;
      const s = dw >= 0 ? 2 / h : -2 / h;
      W[i * 3] = (aw * bx + ax * bw + ay * bz - az * by) * s;
      W[i * 3 + 1] = (aw * by - ax * bz + ay * bw + az * bx) * s;
      W[i * 3 + 2] = (aw * bz + ax * by - ay * bx + az * bw) * s;
    }

    // Velocity pass: restitution and dynamic friction at every contact.
    this.solveVelocities(h);

    // Safety clamp, then sleep bodies that have stayed still.
    for (let i = 0; i < n; i++) {
      if (this.asleep[i]) continue;
      const vx = V[i * 3]!,
        vy = V[i * 3 + 1]!,
        vz = V[i * 3 + 2]!;
      const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
      if (sp > MAX_SPEED) {
        const k = MAX_SPEED / sp;
        V[i * 3] = vx * k;
        V[i * 3 + 1] = vy * k;
        V[i * 3 + 2] = vz * k;
      }
      const w2 = W[i * 3]! ** 2 + W[i * 3 + 1]! ** 2 + W[i * 3 + 2]! ** 2;
      // Only a body that is resting on something may sleep: one still sinking
      // through open water keeps going, however slowly.
      const still = this.touching[i] === 1 && sp * sp < SLEEP_SPEED ** 2 && w2 < SLEEP_SPIN ** 2;
      this.touching[i] = 0;
      this.restTime[i] = still ? this.restTime[i]! + h : 0;
      if (this.restTime[i]! > SLEEP_AFTER) {
        this.asleep[i] = 1;
        V.fill(0, i * 3, i * 3 + 3);
        W.fill(0, i * 3, i * 3 + 3);
      }
    }
  }

  /** Relative velocity of a's local point minus b's (b < 0: static), along (nx, ny, nz). */
  private relNormalVelocity(
    a: number,
    lax: number,
    lay: number,
    laz: number,
    b: number,
    lbx: number,
    lby: number,
    lbz: number,
    nx: number,
    ny: number,
    nz: number,
  ): number {
    const V = this.V,
      W = this.W;
    this.rot(a, lax, lay, laz);
    let vx = V[a * 3]! + W[a * 3 + 1]! * this.oz - W[a * 3 + 2]! * this.oy;
    let vy = V[a * 3 + 1]! + W[a * 3 + 2]! * this.ox - W[a * 3]! * this.oz;
    let vz = V[a * 3 + 2]! + W[a * 3]! * this.oy - W[a * 3 + 1]! * this.ox;
    if (b >= 0) {
      this.rot(b, lbx, lby, lbz);
      vx -= V[b * 3]! + W[b * 3 + 1]! * this.oz - W[b * 3 + 2]! * this.oy;
      vy -= V[b * 3 + 1]! + W[b * 3 + 2]! * this.ox - W[b * 3]! * this.oz;
      vz -= V[b * 3 + 2]! + W[b * 3]! * this.oy - W[b * 3 + 1]! * this.ox;
    }
    this.vx = vx;
    this.vy = vy;
    this.vz = vz;
    return vx * nx + vy * ny + vz * nz;
  }

  private vx = 0;
  private vy = 0;
  private vz = 0;

  /** Records a contact for the velocity pass (velocities are still the predicted ones here). */
  private record(
    a: number,
    lax: number,
    lay: number,
    laz: number,
    b: number,
    lbx: number,
    lby: number,
    lbz: number,
    nx: number,
    ny: number,
    nz: number,
    lambda: number,
  ): void {
    if (b >= this.n) b = -1;
    this.touching[a] = 1;
    if (b >= 0) this.touching[b] = 1;
    if (this.contactCount * CONTACT_STRIDE >= this.contacts.length) {
      const grown = new Float64Array(this.contacts.length * 2);
      grown.set(this.contacts);
      this.contacts = grown;
    }
    const vn = this.relNormalVelocity(a, lax, lay, laz, b, lbx, lby, lbz, nx, ny, nz);
    const c = this.contacts,
      k = this.contactCount++ * CONTACT_STRIDE;
    c[k] = a;
    c[k + 1] = b;
    c[k + 2] = lax;
    c[k + 3] = lay;
    c[k + 4] = laz;
    c[k + 5] = lbx;
    c[k + 6] = lby;
    c[k + 7] = lbz;
    c[k + 8] = nx;
    c[k + 9] = ny;
    c[k + 10] = nz;
    c[k + 11] = lambda;
    c[k + 12] = vn;
  }

  /**
   * Velocity pass (Müller et al. 2020): at every contact, set the normal
   * velocity to the restitution target (a little bounce for real impacts,
   * none for resting contact or for the push-out of an overlap) and apply
   * dynamic friction proportional to the normal force. This is what lets a
   * pile come to rest, and it stops position corrections turning into launches.
   */
  private solveVelocities(h: number): void {
    const c = this.contacts,
      V = this.V,
      W = this.W;
    for (let k = 0; k < this.contactCount * CONTACT_STRIDE; k += CONTACT_STRIDE) {
      const a = c[k]!,
        b = c[k + 1]!;
      if (this.asleep[a] && (b < 0 || this.asleep[b])) continue;
      const nx = c[k + 8]!,
        ny = c[k + 9]!,
        nz = c[k + 10]!;
      const vn = this.relNormalVelocity(
        a,
        c[k + 2]!,
        c[k + 3]!,
        c[k + 4]!,
        b,
        c[k + 5]!,
        c[k + 6]!,
        c[k + 7]!,
        nx,
        ny,
        nz,
      );
      const tx = this.vx - vn * nx,
        ty = this.vy - vn * ny,
        tz = this.vz - vn * nz;
      const vt = Math.sqrt(tx * tx + ty * ty + tz * tz);
      // Contact offsets (world) on both bodies.
      this.rot(a, c[k + 2]!, c[k + 3]!, c[k + 4]!);
      const rax = this.ox,
        ray = this.oy,
        raz = this.oz;
      let rbx = 0,
        rby = 0,
        rbz = 0;
      if (b >= 0) {
        this.rot(b, c[k + 5]!, c[k + 6]!, c[k + 7]!);
        rbx = this.ox;
        rby = this.oy;
        rbz = this.oz;
      }
      // Restitution: bounce only off real impacts, and never more than they had.
      const vnPrev = c[k + 12]!;
      const e = vnPrev < -2 * G * h ? RESTITUTION : 0;
      const target = vnPrev < 0 ? -e * vnPrev : 0;
      let dx = nx * (target - vn),
        dy = ny * (target - vn),
        dz = nz * (target - vn);
      // Dynamic friction: at most μ times the normal speed change this contact's push produced.
      if (vt > 1e-9) {
        const wn =
          this.gim(a, rax, ray, raz, nx, ny, nz) + (b >= 0 ? this.gim(b, rbx, rby, rbz, nx, ny, nz) : 0);
        const cut = Math.min((FRICTION * c[k + 11]! * wn) / h, vt) / vt;
        dx -= tx * cut;
        dy -= ty * cut;
        dz -= tz * cut;
      }
      const dl = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (dl < 1e-9) continue;
      const ux = dx / dl,
        uy = dy / dl,
        uz = dz / dl;
      // Impulse along Δv through both bodies' generalised inverse masses.
      const wsum =
        this.gim(a, rax, ray, raz, ux, uy, uz) + (b >= 0 ? this.gim(b, rbx, rby, rbz, ux, uy, uz) : 0);
      const p = dl / wsum;
      const px = ux * p,
        py = uy * p,
        pz = uz * p;
      V[a * 3]! += px * this.im[a]!;
      V[a * 3 + 1]! += py * this.im[a]!;
      V[a * 3 + 2]! += pz * this.im[a]!;
      this.invI(a, ray * pz - raz * py, raz * px - rax * pz, rax * py - ray * px);
      W[a * 3]! += this.ox;
      W[a * 3 + 1]! += this.oy;
      W[a * 3 + 2]! += this.oz;
      if (b >= 0) {
        V[b * 3]! -= px * this.im[b]!;
        V[b * 3 + 1]! -= py * this.im[b]!;
        V[b * 3 + 2]! -= pz * this.im[b]!;
        this.invI(b, rby * pz - rbz * py, rbz * px - rbx * pz, rbx * py - rby * px);
        W[b * 3]! -= this.ox;
        W[b * 3 + 1]! -= this.oy;
        W[b * 3 + 2]! -= this.oz;
      }
    }
    this.contactCount = 0;
  }

  private interfaceIntact(i: number): boolean {
    let held = 0;
    for (let k = i * 4; k < i * 4 + 4; k++) if (!this.jBroken[k]) held++;
    return held >= 2;
  }

  /** Corners of box a inside box b are pushed out along b's shallowest axis. */
  private boxContact(a: number, b: number, h: number, samples?: Float64Array): void {
    const X = this.X;
    const dx = X[a * 3]! - X[b * 3]!,
      dy = X[a * 3 + 1]! - X[b * 3 + 1]!,
      dz = X[a * 3 + 2]! - X[b * 3 + 2]!;
    const ra = Math.sqrt(this.hx[a]! ** 2 + this.hy[a]! ** 2 + this.hz[a]! ** 2);
    const rb = Math.sqrt(this.hx[b]! ** 2 + this.hy[b]! ** 2 + this.hz[b]! ** 2);
    if (dx * dx + dy * dy + dz * dz > (ra + rb) ** 2) return;
    const pts = samples ?? this.cornerSamples(a);
    for (let s = 0; s < pts.length; s += 3) this.pointContact(a, b, pts[s]!, pts[s + 1]!, pts[s + 2]!, h);
  }

  private readonly cornerCache = new Map<number, Float64Array>();
  private layerLattice: Float64Array[] = [];
  private obstacleEdges: Float64Array[] = [];
  private obstacleEdgesWorld: Float64Array[] = [];
  private readonly obstacleSphere: { center: Vec3; radius: number }[] = [];

  private cornerSamples(i: number): Float64Array {
    let c = this.cornerCache.get(i);
    if (!c) {
      const out: number[] = [];
      for (const cx of SIGNS)
        for (const cy of SIGNS)
          for (const cz of SIGNS) out.push(cx * this.hx[i]!, cy * this.hy[i]!, cz * this.hz[i]!);
      c = Float64Array.from(out);
      this.cornerCache.set(i, c);
    }
    return c;
  }

  /** The 26 points of a 3×3×3 lattice on box i's surface (corners, edge midpoints, face centres). */
  private latticeSamples(i: number): Float64Array {
    const out: number[] = [];
    for (const x of [-1, 0, 1])
      for (const y of [-1, 0, 1])
        for (const z of [-1, 0, 1])
          if (x || y || z) out.push(x * this.hx[i]!, y * this.hy[i]!, z * this.hz[i]!);
    return Float64Array.from(out);
  }

  /** Points along all 12 edges of box i, at most `spacing` apart. */
  private edgeSamples(i: number, spacing: number): Float64Array {
    const he = [this.hx[i]!, this.hy[i]!, this.hz[i]!];
    const out: number[] = [];
    for (let axis = 0; axis < 3; axis++) {
      const steps = Math.max(1, Math.ceil((2 * he[axis]!) / spacing));
      const [u, v] = [(axis + 1) % 3, (axis + 2) % 3];
      for (const su of SIGNS)
        for (const sv of SIGNS)
          for (let k = 0; k <= steps; k++) {
            const p = [0, 0, 0];
            p[axis] = -he[axis]! + (2 * he[axis]! * k) / steps;
            p[u] = su * he[u]!;
            p[v] = sv * he[v]!;
            out.push(p[0]!, p[1]!, p[2]!);
          }
    }
    return Float64Array.from(out);
  }

  /** Local point of layer i, if inside convex obstacle o, is pushed out through the nearest plane. */
  private convexContact(
    i: number,
    o: number,
    shape: ConvexShape,
    lx: number,
    ly: number,
    lz: number,
    h: number,
  ): void {
    const X = this.X;
    this.rot(i, lx, ly, lz);
    const rx = this.ox,
      ry = this.oy,
      rz = this.oz;
    const wx = X[i * 3]! + rx,
      wy = X[i * 3 + 1]! + ry,
      wz = X[i * 3 + 2]! + rz;
    this.rot(o, wx - X[o * 3]!, wy - X[o * 3 + 1]!, wz - X[o * 3 + 2]!, true);
    let best = -Infinity,
      plane = 0;
    for (let k = 0; k < shape.planes.length; k++) {
      const pl = shape.planes[k]!;
      const d = pl.n.x * this.ox + pl.n.y * this.oy + pl.n.z * this.oz - pl.d;
      if (d > best) {
        best = d;
        plane = k;
      }
      if (best >= 0) return;
    }
    const pl = shape.planes[plane]!;
    const pn = pl.n;
    // Depth of the same material point behind the same plane one substep ago.
    this.prevWorld(i, lx, ly, lz);
    this.rot(o, this.ox - X[o * 3]!, this.oy - X[o * 3 + 1]!, this.oz - X[o * 3 + 2]!, true);
    const prevDepth = -(pn.x * this.ox + pn.y * this.oy + pn.z * this.oz - pl.d);
    const depth = this.allowed(-best, prevDepth, h);
    this.rot(o, pn.x, pn.y, pn.z);
    const nx = this.ox,
      ny = this.oy,
      nz = this.oz;
    if (depth > 0.02) this.wake(i);
    const lam = this.correct(i, rx, ry, rz, -1, 0, 0, 0, nx, ny, nz, depth, 0, h);
    this.record(i, lx, ly, lz, -1, 0, 0, 0, nx, ny, nz, lam);
    this.obstacleFriction(i, o, lx, ly, lz, 0, 0, 0, nx, ny, nz, depth, h);
  }

  /** Local point (lx, ly, lz) of box a, if inside box b, is pushed out along b's shallowest axis. */
  private pointContact(a: number, b: number, lx0: number, ly0: number, lz0: number, h: number): void {
    const X = this.X;
    this.rot(a, lx0, ly0, lz0);
    const rax = this.ox,
      ray = this.oy,
      raz = this.oz;
    const wx = X[a * 3]! + rax,
      wy = X[a * 3 + 1]! + ray,
      wz = X[a * 3 + 2]! + raz;
    this.rot(b, wx - X[b * 3]!, wy - X[b * 3 + 1]!, wz - X[b * 3 + 2]!, true);
    const lx = this.ox,
      ly = this.oy,
      lz = this.oz;
    const px = this.hx[b]! - Math.abs(lx);
    const py = this.hy[b]! - Math.abs(ly);
    const pz = this.hz[b]! - Math.abs(lz);
    if (px <= 0 || py <= 0 || pz <= 0) return;
    let nx = 0,
      ny = 0,
      nz = 0,
      depth: number;
    if (px <= py && px <= pz) {
      nx = Math.sign(lx) || 1;
      depth = px;
    } else if (py <= pz) {
      ny = Math.sign(ly) || 1;
      depth = py;
    } else {
      nz = Math.sign(lz) || 1;
      depth = pz;
    }
    // Same axis, one substep ago (both bodies at their previous poses).
    this.prevWorld(a, lx0, ly0, lz0);
    this.rot(
      b,
      this.ox - this.PX[b * 3]!,
      this.oy - this.PX[b * 3 + 1]!,
      this.oz - this.PX[b * 3 + 2]!,
      true,
      this.PQ,
    );
    const prevDepth = nx
      ? this.hx[b]! - nx * this.ox
      : ny
        ? this.hy[b]! - ny * this.oy
        : this.hz[b]! - nz * this.oz;
    depth = this.allowed(depth, prevDepth, h);
    this.rot(b, nx, ny, nz);
    const wnx = this.ox,
      wny = this.oy,
      wnz = this.oz;
    if (depth > 0.02) {
      if (a < this.n) this.wake(a);
      if (b < this.n) this.wake(b);
    }
    const lam = this.correct(
      a,
      rax,
      ray,
      raz,
      b,
      wx - X[b * 3]!,
      wy - X[b * 3 + 1]!,
      wz - X[b * 3 + 2]!,
      wnx,
      wny,
      wnz,
      depth,
      0,
      h,
    );
    // a was pushed along +n. Record with the moving layer first (obstacles are static).
    if (a >= this.n) this.record(b, lx, ly, lz, -1, 0, 0, 0, -wnx, -wny, -wnz, lam);
    else this.record(a, lx0, ly0, lz0, b, lx, ly, lz, wnx, wny, wnz, lam);
    if (a >= this.n || b >= this.n)
      this.obstacleFriction(a, b, lx0, ly0, lz0, lx, ly, lz, wnx, wny, wnz, depth, h);
  }

  /**
   * Friction against an obstacle: the layer's touching point may not slide
   * along the surface while the push is strong enough (static), and slides
   * with reduced speed otherwise (kinetic).
   */
  private obstacleFriction(
    a: number,
    b: number,
    alx: number,
    aly: number,
    alz: number,
    blx: number,
    bly: number,
    blz: number,
    nx: number,
    ny: number,
    nz: number,
    depth: number,
    h: number,
  ): void {
    const mu = 0.9;
    const X = this.X,
      PX = this.PX;
    // The layer body and its material point (local) at the contact.
    const layer = a < this.n ? a : b;
    const lx = layer === a ? alx : blx,
      ly = layer === a ? aly : bly,
      lz = layer === a ? alz : blz;
    this.rot(layer, lx, ly, lz);
    const rx = this.ox,
      ry = this.oy,
      rz = this.oz;
    this.rot(layer, lx, ly, lz, false, this.PQ);
    let sx = X[layer * 3]! + rx - (PX[layer * 3]! + this.ox);
    let sy = X[layer * 3 + 1]! + ry - (PX[layer * 3 + 1]! + this.oy);
    let sz = X[layer * 3 + 2]! + rz - (PX[layer * 3 + 2]! + this.oz);
    const sn = sx * nx + sy * ny + sz * nz;
    sx -= sn * nx;
    sy -= sn * ny;
    sz -= sn * nz;
    const slip = Math.sqrt(sx * sx + sy * sy + sz * sz);
    if (slip < 1e-9) return;
    const limit = mu * (depth + 0.01);
    const undo = slip < limit ? slip : limit;
    this.correct(layer, rx, ry, rz, -1, 0, 0, 0, -sx / slip, -sy / slip, -sz / slip, undo, 0, h);
  }

  /**
   * Runs the whole simulation and returns keyframes of layer origin poses.
   * `onFrames(from, to, data)` is called as keyframes become ready (every
   * `chunkFrames`, and once at the end), so a caller can start playing the
   * collapse long before the solve finishes.
   */
  run(stream?: {
    onFrames: (from: number, to: number, data: Float32Array) => void;
    chunkFrames?: number;
  }): LayerSimulationResult {
    const t0 = performance.now();
    const chunk = Math.max(1, stream?.chunkFrames ?? 15);
    let sent = 0;
    const flush = (upTo: number) => {
      if (stream && upTo > sent) {
        stream.onFrames(sent, upTo, data);
        sent = upTo;
      }
    };
    const { duration, keyframeHz, stepHz, substeps } = this.o;
    const n = this.n;
    const frames = Math.ceil(duration * keyframeHz) + 1;
    const data = new Float32Array(frames * n * 7);
    const dt = 1 / stepHz;
    let thinnest = Infinity;
    for (let i = 0; i < n; i++)
      thinnest = Math.min(thinnest, 2 * Math.min(this.hx[i]!, this.hy[i]!, this.hz[i]!));
    let frame = 0;
    let nextKey = 1 / keyframeHz;
    let settledAfter = duration;
    const write = (f: number) => {
      for (let i = 0; i < n; i++) {
        this.rot(i, 0, -this.hy[i]!, 0);
        const k = (f * n + i) * 7;
        data[k] = this.X[i * 3]! + this.ox;
        data[k + 1] = this.X[i * 3 + 1]! + this.oy;
        data[k + 2] = this.X[i * 3 + 2]! + this.oz;
        data[k + 3] = this.Q[i * 4]!;
        data[k + 4] = this.Q[i * 4 + 1]!;
        data[k + 5] = this.Q[i * 4 + 2]!;
        data[k + 6] = this.Q[i * 4 + 3]!;
      }
    };
    write(frame++);
    while (frame < frames) {
      // Adaptive substeps: enough that the fastest awake layer moves at most a
      // fraction of the thinnest slab per substep, never more than configured.
      let vmax = 0;
      for (let i = 0; i < n; i++)
        if (!this.asleep[i]) {
          const v2 = this.V[i * 3]! ** 2 + this.V[i * 3 + 1]! ** 2 + this.V[i * 3 + 2]! ** 2;
          if (v2 > vmax) vmax = v2;
        }
      const sub = Math.min(
        substeps,
        Math.max(MIN_SUBSTEPS, Math.ceil((Math.sqrt(vmax) * dt) / (SUBSTEP_TRAVEL * thinnest))),
      );
      const hs = dt / sub;
      for (let s = 0; s < sub; s++) {
        this.substep(hs);
        this.time += hs;
      }
      this.substepsTaken += sub;
      while (this.time >= nextKey - 1e-9 && frame < frames) {
        write(frame++);
        nextKey += 1 / keyframeHz;
      }
      if (frame - sent >= chunk) flush(frame);
      if (this.asleep.subarray(0, n).every((a) => a === 1)) {
        settledAfter = this.time;
        const last = (frame - 1) * n * 7;
        for (; frame < frames; frame++) data.copyWithin(frame * n * 7, last, last + n * 7);
        break;
      }
    }
    flush(frames);
    return {
      hz: keyframeHz,
      frames,
      layers: n,
      data,
      jointBreakTimes: this.jBreakTime,
      settledAfter,
      computeMs: performance.now() - t0,
      substeps: this.substepsTaken,
    };
  }
}

/** Pose of layer `i` at `seconds` after release, interpolated between keyframes. */
export function layerPoseAt(r: LayerSimulationResult, i: number, seconds: number): Pose {
  const f = Math.max(0, Math.min(r.frames - 1, seconds * r.hz));
  const f0 = Math.floor(f);
  const f1 = Math.min(r.frames - 1, f0 + 1);
  const u = f - f0;
  const a = (f0 * r.layers + i) * 7;
  const b = (f1 * r.layers + i) * 7;
  const d = r.data;
  let qx = d[b + 3]!,
    qy = d[b + 4]!,
    qz = d[b + 5]!,
    qw = d[b + 6]!;
  if (d[a + 3]! * qx + d[a + 4]! * qy + d[a + 5]! * qz + d[a + 6]! * qw < 0) {
    qx = -qx;
    qy = -qy;
    qz = -qz;
    qw = -qw;
  }
  // Normalised lerp is accurate at 30 Hz keyframes.
  const q = normalizeQuat({
    x: d[a + 3]! + (qx - d[a + 3]!) * u,
    y: d[a + 4]! + (qy - d[a + 4]!) * u,
    z: d[a + 5]! + (qz - d[a + 5]!) * u,
    w: d[a + 6]! + (qw - d[a + 6]!) * u,
  });
  return {
    position: vec3(
      d[a]! + (d[b]! - d[a]!) * u,
      d[a + 1]! + (d[b + 1]! - d[a + 1]!) * u,
      d[a + 2]! + (d[b + 2]! - d[a + 2]!) * u,
    ),
    quaternion: q,
  };
}
