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
}

const G = 9.81;
const WATER = 1025;
const SIGNS = [-1, 1] as const;
const MAX_SPEED = 70;

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
  // Per joint (4 per interface between layer k and k + 1).
  private readonly jl: Float64Array; // la xyz, lb xyz
  private readonly jStrength: Float64Array;
  private readonly jBroken: Uint8Array;
  private readonly jBreakTime: Float32Array;
  private time = 0;
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
      ...opts,
    };
    const n = (this.n = layers.length);
    const f = () => new Float64Array(n);
    this.hx = f();
    this.hy = f();
    this.hz = f();
    this.im = f();
    this.iix = f();
    this.iiy = f();
    this.iiz = f();
    this.X = new Float64Array(n * 3);
    this.Q = new Float64Array(n * 4);
    this.V = new Float64Array(n * 3);
    this.W = new Float64Array(n * 3);
    this.PX = new Float64Array(n * 3);
    this.PQ = new Float64Array(n * 4);
    this.flood = f();
    this.floodRate = f();
    this.restTime = f();
    this.asleep = new Uint8Array(n);
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
    const joints = Math.max(0, n - 1) * 4;
    this.jl = new Float64Array(joints * 6);
    this.jStrength = new Float64Array(joints);
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
          j++;
        }
  }

  // -- scalar helpers (results in ox, oy, oz) --------------------------------

  /** Rotate (x, y, z) by body i's quaternion (sign −1: inverse). */
  private rot(i: number, x: number, y: number, z: number, inverse = false): void {
    const Q = this.Q;
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
  ): void {
    const wa = this.gim(a, rax, ray, raz, nx, ny, nz);
    const wb = b >= 0 ? this.gim(b, rbx, rby, rbz, nx, ny, nz) : 0;
    const dl = c / (wa + wb + alpha / (h * h));
    this.push(a, rax, ray, raz, nx * dl, ny * dl, nz * dl, 1);
    if (b >= 0) this.push(b, rbx, rby, rbz, nx * dl, ny * dl, nz * dl, -1);
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
      if (c > 1e-9) this.correct(a, rax, ray, raz, b, rbx, rby, rbz, dx / c, dy / c, dz / c, c, 1e-8, h);
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
            const g = this.o.groundHeight(px, pz);
            const depth = g - (X[i * 3 + 1]! + ry);
            if (depth <= 0) continue;
            this.correct(i, rx, ry, rz, -1, 0, 0, 0, 0, 1, 0, depth, 0, h);
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

    // Velocities from positions; sleep bodies that have stayed still.
    for (let i = 0; i < n; i++) {
      if (this.asleep[i]) continue;
      const vx = (X[i * 3]! - PX[i * 3]!) / h,
        vy = (X[i * 3 + 1]! - PX[i * 3 + 1]!) / h,
        vz = (X[i * 3 + 2]! - PX[i * 3 + 2]!) / h;
      // Safety clamp: no structural piece outruns the fall itself.
      const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
      const k = sp > MAX_SPEED ? MAX_SPEED / sp : 1;
      V[i * 3] = vx * k;
      V[i * 3 + 1] = vy * k;
      V[i * 3 + 2] = vz * k;
      // ω from Δq = q · conj(q_prev).
      const ax = Q[i * 4]!,
        ay = Q[i * 4 + 1]!,
        az = Q[i * 4 + 2]!,
        aw = Q[i * 4 + 3]!;
      const bx = -PQ[i * 4]!,
        by = -PQ[i * 4 + 1]!,
        bz = -PQ[i * 4 + 2]!,
        bw = PQ[i * 4 + 3]!;
      const dx = aw * bx + ax * bw + ay * bz - az * by;
      const dy = aw * by - ax * bz + ay * bw + az * bx;
      const dz = aw * bz + ax * by - ay * bx + az * bw;
      const dw = aw * bw - ax * bx - ay * by - az * bz;
      const s = dw >= 0 ? 2 / h : -2 / h;
      W[i * 3] = dx * s;
      W[i * 3 + 1] = dy * s;
      W[i * 3 + 2] = dz * s;
      const still = vx * vx + vy * vy + vz * vz < 0.01 && (dx * dx + dy * dy + dz * dz) * s * s < 0.002;
      this.restTime[i] = still ? this.restTime[i]! + h : 0;
      if (this.restTime[i]! > 0.6) {
        this.asleep[i] = 1;
        V.fill(0, i * 3, i * 3 + 3);
        W.fill(0, i * 3, i * 3 + 3);
      }
    }
  }

  private interfaceIntact(i: number): boolean {
    let held = 0;
    for (let k = i * 4; k < i * 4 + 4; k++) if (!this.jBroken[k]) held++;
    return held >= 2;
  }

  /** Corners of box a inside box b are pushed out along b's shallowest axis. */
  private boxContact(a: number, b: number, h: number): void {
    const X = this.X;
    const dx = X[a * 3]! - X[b * 3]!,
      dy = X[a * 3 + 1]! - X[b * 3 + 1]!,
      dz = X[a * 3 + 2]! - X[b * 3 + 2]!;
    const ra = Math.sqrt(this.hx[a]! ** 2 + this.hy[a]! ** 2 + this.hz[a]! ** 2);
    const rb = Math.sqrt(this.hx[b]! ** 2 + this.hy[b]! ** 2 + this.hz[b]! ** 2);
    if (dx * dx + dy * dy + dz * dz > (ra + rb) ** 2) return;
    for (const cx of SIGNS)
      for (const cy of SIGNS)
        for (const cz of SIGNS) {
          this.rot(a, cx * this.hx[a]!, cy * this.hy[a]!, cz * this.hz[a]!);
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
          if (px <= 0 || py <= 0 || pz <= 0) continue;
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
          this.rot(b, nx, ny, nz);
          const wnx = this.ox,
            wny = this.oy,
            wnz = this.oz;
          if (depth > 0.02) {
            this.wake(a);
            this.wake(b);
          }
          this.correct(
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
        }
  }

  /** Runs the whole simulation and returns keyframes of layer origin poses. */
  run(): LayerSimulationResult {
    const t0 = performance.now();
    const { duration, keyframeHz, stepHz, substeps } = this.o;
    const n = this.n;
    const frames = Math.ceil(duration * keyframeHz) + 1;
    const data = new Float32Array(frames * n * 7);
    const dt = 1 / stepHz;
    const h = dt / substeps;
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
      for (let s = 0; s < substeps; s++) {
        this.substep(h);
        this.time += h;
      }
      while (this.time >= nextKey - 1e-9 && frame < frames) {
        write(frame++);
        nextKey += 1 / keyframeHz;
      }
      if (this.asleep.every((a) => a === 1)) {
        settledAfter = this.time;
        const last = (frame - 1) * n * 7;
        for (; frame < frames; frame++) data.copyWithin(frame * n * 7, last, last + n * 7);
        break;
      }
    }
    return {
      hz: keyframeHz,
      frames,
      layers: n,
      data,
      jointBreakTimes: this.jBreakTime,
      settledAfter,
      computeMs: performance.now() - t0,
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
