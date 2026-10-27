// Tower layout and the authored macro-scale collapse.
//
// The tower is split into a stump (which stays standing, damaged) and a few
// tipping sections. Their motion is authored, not simulated: an inverted
// pendulum solution for the overall tip, remapped so impact lands exactly on
// the event's WATER_IMPACT stage; small inter-section bends for lag and whip;
// and a blend into an authored wreck pose on the seabed. Everything is a pure
// function of event time, so scrubbing, replay and late joining are exact.

import {
  DEG,
  type Pose,
  type Quat,
  SeededRandom,
  type Vec3,
  add,
  clamp,
  composePose,
  easeInOutCubic,
  easeOutCubic,
  quatFromAxisAngle,
  quatIdentity,
  quatMultiply,
  quatRotate,
  quatSlerp,
  lerpVec,
  scale,
  sub,
  normalize,
  smoothstep,
  vec3,
} from '@destruction-dynamics/core';
import {
  type LayerBodyInit,
  type LayerSimulationResult,
  LayerSolver,
  type LayerSolverOptions,
  layerPoseAt,
} from './layerSolver';
import { type StaticObstacle, obstacleEdgePoints, pointInObstacle } from './shapes';

export interface TowerSpec {
  floors: number;
  floorHeight: number;
  /** Footprint along x (the fall direction) and z. */
  width: number;
  depth: number;
  /** Floors in the stump that stays standing. */
  breakFloor: number;
  /** Tipping sections above the stump. */
  sections: number;
  /** Base centre in the scene frame. */
  base: Vec3;
}

export const DEEP_END_TOWER: TowerSpec = {
  floors: 85,
  floorHeight: 3.4,
  width: 32,
  depth: 30,
  breakFloor: 8,
  sections: 5,
  base: vec3(0, 1.5, 0),
};

export interface TowerSection {
  index: number;
  /** First floor (inclusive) and floor count. */
  firstFloor: number;
  floorCount: number;
  height: number;
  /** Whether this is the stump. */
  stump: boolean;
}

export function towerSections(spec: TowerSpec): TowerSection[] {
  const out: TowerSection[] = [
    {
      index: 0,
      firstFloor: 0,
      floorCount: spec.breakFloor,
      height: spec.breakFloor * spec.floorHeight,
      stump: true,
    },
  ];
  const rest = spec.floors - spec.breakFloor;
  let floor = spec.breakFloor;
  for (let s = 0; s < spec.sections; s++) {
    const count = Math.round((rest * (s + 1)) / spec.sections) - (floor - spec.breakFloor);
    out.push({
      index: s + 1,
      firstFloor: floor,
      floorCount: count,
      height: count * spec.floorHeight,
      stump: false,
    });
    floor += count;
  }
  return out;
}

export interface CollapseTimeline {
  /** Event time INITIAL_FAILURE begins. */
  failureStart: number;
  /** Event time the tower starts to tip (ACTIVE_DESTRUCTION). */
  tipStart: number;
  /** Event time the tower meets the water (WATER_IMPACT). */
  impact: number;
  /** Event time the wreck is fully settled (end of SETTLING). */
  settleEnd: number;
}

export interface CollapseOptions {
  /** Ground / seabed height in the scene frame. */
  groundHeight: (x: number, z: number) => number;
  seed: number;
  /** Lean reached by the end of the initial failure, degrees. */
  initialLeanDeg?: number;
  /** Overall tip angle at impact, degrees. */
  impactAngleDeg?: number;
  /**
   * From impact onward, simulate every tipping layer as its own rigid body
   * (LayerSolver) instead of blending into an authored wreck.
   */
  simulateLayers?: boolean | 'deferred';
  seaLevel?: number;
  /** Overrides for the layer solver (testing and tuning). */
  layerSolver?: Partial<LayerSolverOptions>;
  /**
   * Indestructible boxes in the tower's path. With simulated layers, the
   * layers are released the moment the authored fall first touches one.
   */
  obstacles?: StaticObstacle[];
}

export interface LayerStats {
  layers: number;
  joints: number;
  jointsBroken: number;
  settledAfter: number;
  computeMs: number;
}

const zAxis = vec3(0, 0, 1);
const yAxis = vec3(0, 1, 0);
/** Rotation that tips +y toward +x by `a` radians. */
const tip = (a: number): Quat => quatFromAxisAngle(zAxis, -a);

const translate = (v: Vec3): Pose => ({ position: v, quaternion: quatIdentity() });
const rotate = (q: Quat): Pose => ({ position: vec3(), quaternion: q });

export class TowerCollapse {
  readonly sections: readonly TowerSection[];
  private readonly halfW: number;
  private readonly hingeHeight: number;
  private readonly lean0: number;
  private readonly impactAngle: number;
  /** Event time the layers leave the authored path (impact, or first obstacle contact). */
  readonly releaseTime: number;
  /** θ(s) samples of the pendulum fall, s in [0, 1]. */
  private readonly fallTable: Float64Array;
  private wreck: Pose[];
  private readonly fallEnd: Pose[];
  /** Seeded shape of the bend along the tower (amplitude and waviness). */
  private readonly bendAmp: number;
  private readonly bendPhase: number;
  private layers: LayerSimulationResult | null = null;
  private authoredWreck: Pose[];
  /** Bumped when simulated layers are attached (views and collision rebuild). */
  layerVersion = 0;

  constructor(
    readonly spec: TowerSpec,
    readonly timeline: CollapseTimeline,
    private readonly opts: CollapseOptions,
  ) {
    if (!(timeline.failureStart <= timeline.tipStart && timeline.tipStart < timeline.impact))
      throw new Error('collapse timeline must satisfy failureStart <= tipStart < impact');
    this.sections = towerSections(spec);
    this.halfW = spec.width / 2;
    this.hingeHeight = spec.base.y + spec.breakFloor * spec.floorHeight;
    this.lean0 = (opts.initialLeanDeg ?? 2.5) * DEG;
    this.impactAngle = (opts.impactAngleDeg ?? 88) * DEG;
    this.fallTable = this.solveFall();
    const rng = new SeededRandom(opts.seed);
    this.bendAmp = rng.range(0.75, 1.25);
    this.bendPhase = rng.range(0, Math.PI * 2);
    this.fallEnd = this.fallPoses(1);
    this.releaseTime = this.findRelease();
    this.authoredWreck = this.buildWreck(rng);
    this.wreck = this.authoredWreck;
    if (opts.simulateLayers === true && this.sections.length > 1) this.attachLayers(this.simulateLayers());
  }

  /**
   * What the layer solver needs, as plain data (it can be posted to a worker).
   * The ground function is not included; the worker supplies its own.
   */
  layerSolverInput(): { layers: LayerBodyInit[]; options: Omit<LayerSolverOptions, 'groundHeight'> } {
    const dt = 1 / 120;
    const at = this.authoredFall(this.releaseTime);
    const before = this.authoredFall(this.releaseTime - dt);
    const layers: LayerBodyInit[] = this.sections.slice(1).map((sec, i) => {
      const a = before[i + 1]!;
      const b = at[i + 1]!;
      const dq = quatMultiply(b.quaternion, {
        x: -a.quaternion.x,
        y: -a.quaternion.y,
        z: -a.quaternion.z,
        w: a.quaternion.w,
      });
      const sgn = dq.w >= 0 ? 2 / dt : -2 / dt;
      return {
        pose: b,
        velocity: scale(sub(b.position, a.position), 1 / dt),
        angularVelocity: vec3(dq.x * sgn, dq.y * sgn, dq.z * sgn),
        halfExtents: vec3(this.halfW, sec.height / 2, this.spec.depth / 2),
      };
    });
    // Thin one-floor slabs need finer substeps to stay stable.
    const thinnest = Math.min(...this.sections.slice(1).map((sec) => sec.height));
    return {
      layers,
      options: {
        seaLevel: this.opts.seaLevel ?? 0,
        seed: this.opts.seed ^ 0x1a7e5,
        // The wreck must be at rest by the end of SETTLING.
        duration: Math.max(8, this.timeline.settleEnd - this.releaseTime),
        obstacles: this.opts.obstacles ?? [],
        substeps: thinnest < 8 ? 6 : 4,
        ...this.opts.layerSolver,
      },
    };
  }

  /** Authored fall poses at event time t (failure through impact). */
  private authoredFall(t: number): Pose[] {
    const { failureStart, tipStart, impact } = this.timeline;
    if (t < failureStart) return this.sections.map((_, k) => this.intactPose(k));
    const p = t < tipStart ? 0 : Math.min(1, (t - tipStart) / (impact - tipStart));
    return this.fallPoses(p, Math.min(t, impact));
  }

  /** Whether any tipping section overlaps any obstacle at event time t. */
  private touchesObstacle(t: number, obstacles: readonly StaticObstacle[]): boolean {
    const poses = this.authoredFall(t);
    for (let k = 1; k < poses.length; k++) {
      const pose = poses[k]!;
      const half = this.sectionHalfExtents(k);
      const centre = this.sectionCenter(k, pose);
      const inv = {
        x: -pose.quaternion.x,
        y: -pose.quaternion.y,
        z: -pose.quaternion.z,
        w: pose.quaternion.w,
      };
      for (const o of obstacles) {
        // Obstacle edge points inside the section box…
        for (const p of obstacleEdgePoints(o, 1.5)) {
          const l = quatRotate(inv, sub(p, centre));
          if (Math.abs(l.x) < half.x && Math.abs(l.y) < half.y && Math.abs(l.z) < half.z) return true;
        }
        // …or section surface points (2 m lattice) inside the obstacle.
        const steps = (h: number) => Math.max(1, Math.ceil((2 * h) / 2));
        const [nx, ny, nz] = [steps(half.x), steps(half.y), steps(half.z)];
        for (let i = 0; i <= nx; i++)
          for (let j = 0; j <= ny; j++)
            for (let m = 0; m <= nz; m++) {
              if (i > 0 && i < nx && j > 0 && j < ny && m > 0 && m < nz) continue;
              const local = vec3(
                -half.x + (2 * half.x * i) / nx,
                -half.y + (2 * half.y * j) / ny,
                -half.z + (2 * half.z * m) / nz,
              );
              if (pointInObstacle(add(centre, quatRotate(pose.quaternion, local)), o)) return true;
            }
      }
    }
    return false;
  }

  /** First event time the authored fall touches an obstacle (or impact if never), to 1 ms. */
  private findRelease(): number {
    const obstacles = this.opts.obstacles ?? [];
    const { tipStart, impact } = this.timeline;
    if (obstacles.length === 0) return impact;
    let lo = tipStart;
    for (let t = tipStart; t <= impact; t += 0.05) {
      if (this.touchesObstacle(t, obstacles)) {
        let hi = t;
        while (hi - lo > 1e-3) {
          const mid = (lo + hi) / 2;
          if (this.touchesObstacle(mid, obstacles)) hi = mid;
          else lo = mid;
        }
        return lo;
      }
      lo = t;
    }
    return impact;
  }

  /**
   * Switches from the authored wreck to simulated layers. `framesReady` lets a
   * result be attached while it is still streaming in from a worker: playback
   * holds at the last ready keyframe until more arrive (extendLayers).
   */
  attachLayers(result: LayerSimulationResult, framesReady = result.frames): void {
    if (result.layers !== this.sections.length - 1) throw new Error('layer result does not match this tower');
    this.layers = result;
    this.framesReady = Math.max(1, Math.min(framesReady, result.frames));
    this.updateWreck();
    this.layerVersion++;
  }

  /** More keyframes of an attached, streaming result are ready. */
  extendLayers(framesReady: number): void {
    if (!this.layers) return;
    this.framesReady = Math.max(this.framesReady, Math.min(framesReady, this.layers.frames));
    this.updateWreck();
  }

  private framesReady = 0;

  private updateWreck(): void {
    const r = this.layers!;
    const end = (this.framesReady - 1) / r.hz;
    this.wreck = [this.intactPose(0), ...this.sections.slice(1).map((_, i) => layerPoseAt(r, i, end))];
  }

  get layersAttached(): boolean {
    return this.layers !== null;
  }

  /** Whether every keyframe of the attached result has arrived. */
  get layersComplete(): boolean {
    return this.layers !== null && this.framesReady >= this.layers.frames;
  }

  /** Runs the layer solver synchronously. */
  private simulateLayers(): LayerSimulationResult {
    const { layers, options } = this.layerSolverInput();
    return new LayerSolver(layers, { ...options, groundHeight: this.opts.groundHeight }).run();
  }

  /** Per-layer simulation statistics, or null when layers are authored. */
  get layerStats(): LayerStats | null {
    const r = this.layers;
    if (!r) return null;
    let broken = 0;
    for (const t of r.jointBreakTimes) if (Number.isFinite(t)) broken++;
    return {
      layers: r.layers,
      joints: r.jointBreakTimes.length,
      jointsBroken: broken,
      settledAfter: r.settledAfter,
      computeMs: r.computeMs,
    };
  }

  /** Event time after which the structure no longer moves. */
  get settledAt(): number {
    return this.layers ? this.releaseTime + this.layers.settledAfter : this.timeline.settleEnd;
  }

  /**
   * Inverted pendulum (uniform rod about its base): θ'' = (3g / 2L) sin θ.
   * Integrated with RK4 from the initial lean to the impact angle, then
   * normalised in time so the authored tip duration is exact.
   */
  private solveFall(): Float64Array {
    const L = (this.spec.floors - this.spec.breakFloor) * this.spec.floorHeight;
    const k = (3 * 9.81) / (2 * L);
    const f = (th: number) => k * Math.sin(th);
    let th = this.lean0;
    let om = 0.02;
    const h = 0.005;
    const samples: { t: number; th: number }[] = [{ t: 0, th }];
    let t = 0;
    while (th < this.impactAngle && t < 120) {
      const k1t = om,
        k1o = f(th);
      const k2t = om + (h / 2) * k1o,
        k2o = f(th + (h / 2) * k1t);
      const k3t = om + (h / 2) * k2o,
        k3o = f(th + (h / 2) * k2t);
      const k4t = om + h * k3o,
        k4o = f(th + h * k3t);
      th += (h / 6) * (k1t + 2 * k2t + 2 * k3t + k4t);
      om += (h / 6) * (k1o + 2 * k2o + 2 * k3o + k4o);
      t += h;
      samples.push({ t, th: Math.min(th, this.impactAngle) });
    }
    const total = t;
    const table = new Float64Array(257);
    let j = 0;
    for (let i = 0; i <= 256; i++) {
      const target = (i / 256) * total;
      while (j < samples.length - 2 && samples[j + 1]!.t < target) j++;
      const a = samples[j]!;
      const b = samples[Math.min(j + 1, samples.length - 1)]!;
      const u = b.t > a.t ? clamp((target - a.t) / (b.t - a.t), 0, 1) : 0;
      table[i] = a.th + (b.th - a.th) * u;
    }
    return table;
  }

  private fallAngle(p: number): number {
    const x = clamp(p, 0, 1) * 256;
    const i = Math.min(255, Math.floor(x));
    return this.fallTable[i]! + (this.fallTable[i + 1]! - this.fallTable[i]!) * (x - i);
  }

  /** Overall tilt of the tipping block at event time t, radians. */
  tiltAt(t: number | null): number {
    if (t === null) return 0;
    const { failureStart, tipStart, impact } = this.timeline;
    if (t < failureStart) return 0;
    if (t < tipStart) {
      const p = (t - failureStart) / Math.max(1e-6, tipStart - failureStart);
      // Sway envelope vanishes at both ends so the tip starts exactly at the lean.
      const sway = 0.5 * DEG * Math.sin(Math.PI * p) * Math.sin(2 * Math.PI * 0.35 * (t - failureStart));
      return this.lean0 * p * p + sway;
    }
    if (t < impact) return this.fallAngle((t - tipStart) / (impact - tipStart));
    return this.impactAngle;
  }

  /** Pose chain for the falling block at fall progress p (0..1). */
  private fallPoses(p: number, t?: number): Pose[] {
    const phi = t === undefined ? this.fallAngle(p) : this.tiltAt(t);
    const s = smoothstep(0.35 * this.impactAngle, this.impactAngle, phi);
    const ground = this.opts.groundHeight(this.spec.base.x + this.halfW + 10, this.spec.base.z);
    const pivot = vec3(
      this.spec.base.x + this.halfW + 9 * s,
      this.hingeHeight + (ground + 0.5 - this.hingeHeight) * s,
      this.spec.base.z,
    );
    const n = this.sections.length - 1;
    const poses: Pose[] = [this.intactPose(0)];
    // First tipping section: rotate about its bottom east edge at the pivot.
    let pose = composePose(
      composePose(translate(pivot), rotate(tip(phi))),
      translate(vec3(-this.halfW, 0, 0)),
    );
    // Bend is defined along the tower's height (absolute angle grows with
    // s², with a gentle seeded wave), so the shape is the same whether the
    // tower is split into 5 layers or 77.
    const L = this.spec.floorHeight * (this.spec.floors - this.spec.breakFloor);
    const lag = -0.05 * Math.sin(Math.PI * p);
    const whip = 0.08 * smoothstep(0.82, 1, p);
    const bendAt = (sFrac: number) =>
      (lag + whip) *
      this.bendAmp *
      sFrac *
      sFrac *
      (1 + 0.15 * Math.sin(2 * Math.PI * 1.5 * sFrac + this.bendPhase));
    let height = 0;
    let prevBend = 0;
    for (let k = 1; k <= n; k++) {
      if (k > 1) {
        const prev = this.sections[k - 1]!;
        height += prev.height;
        const abs = bendAt(height / L);
        const bend = abs - prevBend;
        prevBend = abs;
        const hinge = vec3(bend >= 0 ? this.halfW : -this.halfW, prev.height, 0);
        pose = composePose(
          pose,
          composePose(composePose(translate(hinge), rotate(tip(bend))), translate(vec3(-hinge.x, 0, 0))),
        );
      }
      poses.push(pose);
    }
    return poses;
  }

  private intactPose(k: number): Pose {
    const s = this.sections[k]!;
    return translate(
      vec3(this.spec.base.x, this.spec.base.y + s.firstFloor * this.spec.floorHeight, this.spec.base.z),
    );
  }

  private buildWreck(rng: SeededRandom): Pose[] {
    const out: Pose[] = [this.intactPose(0)];
    let gap = 0;
    for (let k = 1; k < this.sections.length; k++) {
      const sec = this.sections[k]!;
      const end = this.fallEnd[k]!;
      gap += k === 1 ? 2 : rng.range(3, 8);
      const roll = rng.range(-6, 6) * DEG;
      const yaw = rng.range(-7, 7) * DEG;
      const extraTip = rng.range(1, 5) * DEG;
      // Lying on its east face: local +x points down after a 90° tip.
      const q = quatMultiply(
        quatFromAxisAngle(yAxis, yaw),
        quatMultiply(tip(Math.PI / 2 + extraTip), quatFromAxisAngle(yAxis, roll)),
      );
      const along = end.position.x + gap;
      const mid = along + sec.height / 2;
      const z = end.position.z + rng.range(-6, 6) + (k - 1) * Math.sin(yaw) * 6;
      const seabed = Math.min(this.opts.groundHeight(mid, z), this.opts.groundHeight(along + 2, z));
      // Section origin is its bottom centre; after the tip its east face is
      // halfW below the origin. Bury it slightly.
      const sink = 2 + rng.range(0, 2);
      out.push({ position: vec3(along, seabed + this.halfW - sink, z), quaternion: q });
    }
    return out;
  }

  /**
   * Section poses (index 0 = stump) at event time t. `t === null` means intact.
   * During restoration the collapse is played backwards to the intact tower.
   */
  /**
   * The collapse time that is displayed: event time normally, and during
   * restoration a rewind from the settled wreck back to the start of failure.
   */
  effectiveTime(t: number | null, restorationProgress: number | null = null): number | null {
    if (restorationProgress === null) return t;
    const { failureStart, settleEnd } = this.timeline;
    return settleEnd + (failureStart - settleEnd) * easeInOutCubic(clamp(restorationProgress, 0, 1));
  }

  posesAt(t: number | null, restorationProgress: number | null = null): Pose[] {
    if (restorationProgress !== null) return this.posesAt(this.effectiveTime(t, restorationProgress), null);
    if (t === null) return this.sections.map((_, k) => this.intactPose(k));
    const { failureStart, impact, settleEnd } = this.timeline;
    if (t < failureStart) return this.sections.map((_, k) => this.intactPose(k));
    if (this.layers && t >= this.releaseTime) {
      // Hold at the newest keyframe if a streaming solve has not caught up yet.
      const after = Math.min(t - this.releaseTime, (this.framesReady - 1) / this.layers.hz);
      return this.sections.map((_, k) =>
        k === 0 ? this.intactPose(0) : layerPoseAt(this.layers!, k - 1, after),
      );
    }
    if (t < impact) return this.authoredFall(t);
    // Impact into wreck: a hard slam, then a slow settle onto the seabed.
    const age = t - impact;
    const slam = easeOutCubic(clamp(age / 2.2, 0, 1));
    const settle = smoothstep(impact + 2.2, settleEnd, t);
    const blend = 0.8 * slam + 0.2 * settle;
    const bounce = 1.8 * Math.exp(-age / 0.9) * Math.sin(age * 5) * (1 - settle);
    return this.fallEnd.map((from, k) => {
      if (k === 0) return from;
      const to = this.wreck[k]!;
      const pos = lerpVec(from.position, to.position, blend);
      pos.y += bounce * (k / this.sections.length);
      return { position: pos, quaternion: quatSlerp(from.quaternion, to.quaternion, blend) };
    });
  }

  /** The wreck's resting poses (for collision and navigation). */
  wreckPoses(): Pose[] {
    return this.wreck.map((p) => ({ position: { ...p.position }, quaternion: { ...p.quaternion } }));
  }

  /** Centre of a section's box in the scene frame for a section pose. */
  sectionCenter(k: number, pose: Pose): Vec3 {
    const s = this.sections[k]!;
    return add(pose.position, quatRotate(pose.quaternion, vec3(0, s.height / 2, 0)));
  }

  sectionHalfExtents(k: number): Vec3 {
    return vec3(this.halfW, this.sections[k]!.height / 2, this.spec.depth / 2);
  }

  /** Where and how the tower meets the water, for WaterImpactEvent cues. */
  impactFootprint(seaLevel = 0) {
    const first = this.fallEnd[1]!;
    const last = this.fallEnd[this.fallEnd.length - 1]!;
    const lastSec = this.sections[this.sections.length - 1]!;
    const top = add(last.position, quatRotate(last.quaternion, vec3(0, lastSec.height, 0)));
    // Only the part over water matters; find where the ground drops below sea level.
    let shoreX = first.position.x;
    while (shoreX < top.x && this.opts.groundHeight(shoreX, first.position.z) > seaLevel) shoreX += 2;
    const startX = Math.max(first.position.x, shoreX);
    const halfLength = Math.max(10, (top.x - startX) / 2);
    return {
      position: vec3(startX + halfLength, seaLevel, (first.position.z + top.z) / 2),
      halfLength,
      halfWidth: this.spec.depth / 2 + 4,
      yaw: Math.atan2(top.z - first.position.z, top.x - first.position.x),
      direction: normalize({ ...quatRotate(first.quaternion, yAxis), y: 0 }),
    };
  }
}
