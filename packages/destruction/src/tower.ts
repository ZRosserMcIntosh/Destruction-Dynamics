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
  normalize,
  smoothstep,
  vec3,
} from '@destruction-dynamics/core';

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
  /** θ(s) samples of the pendulum fall, s in [0, 1]. */
  private readonly fallTable: Float64Array;
  private readonly wreck: Pose[];
  private readonly fallEnd: Pose[];
  private readonly bendSign: number[];

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
    this.bendSign = this.sections.map(() => rng.range(0.7, 1.3));
    this.fallEnd = this.fallPoses(1);
    this.wreck = this.buildWreck(rng);
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
    for (let k = 1; k <= n; k++) {
      if (k > 1) {
        const prev = this.sections[k - 1]!;
        const lag = -0.05 * Math.sin(Math.PI * p);
        const whip = 0.08 * smoothstep(0.82, 1, p);
        const bend = ((k - 1) / Math.max(1, n - 1)) * (lag + whip) * this.bendSign[k]!;
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
    const { failureStart, tipStart, impact, settleEnd } = this.timeline;
    if (t < failureStart) return this.sections.map((_, k) => this.intactPose(k));
    if (t < impact) {
      const p = t < tipStart ? 0 : (t - tipStart) / (impact - tipStart);
      return this.fallPoses(p, t);
    }
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
