// The Impact Lab simulation: a crash-test car driven into a breakable wall,
// or a cannon firing everyday objects at breakable panels. Pure simulation,
// no rendering: the view reads poses (interpolated between fixed steps) and
// drains fracture events for effects and sound.

import { type Pose, type Quat, type Vec3, quatSlerp } from '@destruction-dynamics/core';
import {
  CrashCar,
  FRACTURE_MATERIALS as M,
  type FractureEvent,
  type FractureMaterial,
  FractureWorld,
  type ProjectileDesc,
  buildPanel,
  buildScoredSphere,
  buildWall,
} from '@destruction-dynamics/destruction';
import type RAPIER from '@dimforge/rapier3d-compat';

export const STEP = FractureWorld.STEP;

export interface WallSpec {
  label: string;
  material: FractureMaterial;
  unit: { length: number; height: number; thickness: number };
  bond: 'running' | 'stack';
  note: string;
}

/** Walls are 6 m long and 2.1 m high, built on a concrete sill at x = 0. */
export const WALLS = {
  brick: {
    label: 'Brick',
    material: M.brick,
    unit: { length: 0.225, height: 0.075, thickness: 0.1125 },
    bond: 'running',
    note: 'Single-skin clay brick, running bond',
  },
  cinder: {
    label: 'Concrete block',
    material: M.cinder,
    unit: { length: 0.4, height: 0.2, thickness: 0.2 },
    bond: 'running',
    note: 'Hollow concrete blocks',
  },
  glassBlock: {
    label: 'Glass block',
    material: M.glassBlock,
    unit: { length: 0.2, height: 0.2, thickness: 0.1 },
    bond: 'stack',
    note: 'Glass blocks, stack bond',
  },
  hay: {
    label: 'Hay bales',
    material: M.hay,
    unit: { length: 1.0, height: 0.42, thickness: 0.5 },
    bond: 'running',
    note: 'Straw bales, stacked loose',
  },
  cheese: {
    label: 'Indestructible cheese',
    material: M.cheese,
    unit: { length: 1.5, height: 1.05, thickness: 0.8 },
    bond: 'stack',
    note: 'Day 1 callback: blocks of indestructible Swiss',
  },
} as const satisfies Record<string, WallSpec>;
export type WallId = keyof typeof WALLS;

export interface TargetSpec {
  label: string;
  material: FractureMaterial;
  kind: 'panel' | 'wall';
  size: { width: number; height: number; thickness: number };
}

/** Targets stand in a steel frame centred 1.4 m up at x = 0. */
export const TARGETS = {
  glass: {
    label: 'Window glass',
    material: M.glass,
    kind: 'panel',
    size: { width: 1.2, height: 1.2, thickness: 0.006 },
  },
  drywall: {
    label: 'Plasterboard',
    material: M.drywall,
    kind: 'panel',
    size: { width: 1.2, height: 1.2, thickness: 0.0125 },
  },
  plywood: {
    label: 'Plywood',
    material: M.plywood,
    kind: 'panel',
    size: { width: 1.2, height: 1.2, thickness: 0.018 },
  },
  ice: {
    label: 'Ice slab',
    material: M.ice,
    kind: 'panel',
    size: { width: 0.9, height: 0.9, thickness: 0.1 },
  },
  brick: {
    label: 'Brick panel',
    material: M.brick,
    kind: 'wall',
    size: { width: 1.2, height: 1.2, thickness: 0.1125 },
  },
  cheese: {
    label: 'Indestructible cheese',
    material: M.cheese,
    kind: 'panel',
    size: { width: 1.2, height: 1.2, thickness: 0.25 },
  },
} as const satisfies Record<string, TargetSpec>;
export type TargetId = keyof typeof TARGETS;

export interface ProjectileSpec {
  label: string;
  /** kg, m: real everyday objects. */
  mass: number;
  radius: number;
  restitution: number;
  shape: 'ball' | 'chicken' | 'melon';
}

export const PROJECTILES = {
  tennis: { label: 'Tennis ball', mass: 0.057, radius: 0.0335, restitution: 0.75, shape: 'ball' },
  golf: { label: 'Golf ball', mass: 0.0459, radius: 0.0214, restitution: 0.7, shape: 'ball' },
  bearing: {
    label: 'Steel ball bearing (50 mm)',
    mass: 0.51,
    radius: 0.025,
    restitution: 0.5,
    shape: 'ball',
  },
  bowling: { label: 'Bowling ball', mass: 6.35, radius: 0.109, restitution: 0.1, shape: 'ball' },
  melon: { label: 'Watermelon', mass: 5.3, radius: 0.11, restitution: 0.05, shape: 'melon' },
  chicken: { label: 'Frozen chicken', mass: 1.6, radius: 0.12, restitution: 0.2, shape: 'chicken' },
} as const satisfies Record<string, ProjectileSpec>;
export type ProjectileId = keyof typeof PROJECTILES;

/** Convex outline of a (frozen, oven-ready) chicken: an egg with a keel. */
export const CHICKEN_POINTS: Vec3[] = (() => {
  const pts: Vec3[] = [];
  for (let i = 0; i < 10; i++)
    for (let j = 1; j < 6; j++) {
      const th = (i / 10) * Math.PI * 2;
      const ph = (j / 6) * Math.PI;
      const x = Math.cos(ph) * 0.12 * (Math.cos(ph) > 0 ? 1 : 0.85);
      const r = Math.sin(ph);
      pts.push({ x, y: r * Math.cos(th) * 0.075, z: r * Math.sin(th) * 0.09 });
    }
  pts.push({ x: 0.13, y: 0, z: 0 }, { x: -0.11, y: 0, z: 0 });
  return pts;
})();

export const CANNON_X = -6;
export const TARGET_CENTRE: Vec3 = { x: 0, y: 1.4, z: 0 };
export const CAR_START_REAR = -16;

export interface Shot {
  projectile: ProjectileId;
  speed: number;
  /** Sim time it was fired (for replays). */
  time: number;
}

export interface LiveProjectile {
  id: ProjectileId;
  rb?: RAPIER.RigidBody;
  /** Melon pieces (it is a breakable body itself). */
  pieces?: number[];
  firedAt: number;
}

export type LabSetup =
  { mode: 'crash'; wall: WallId; speedKmh: number } | { mode: 'gallery'; target: TargetId };

interface BodyPose {
  p: Vec3;
  q: Quat;
}

export class ImpactLab {
  fw!: FractureWorld;
  car: CrashCar | null = null;
  setup: LabSetup;
  readonly projectiles: LiveProjectile[] = [];
  readonly shots: Shot[] = [];
  /** Shots to re-fire during a replay. */
  private replayQueue: Shot[] = [];
  time = 0;
  /** Sim time the car was launched, or the first shot fired. */
  startedAt: number | null = null;
  private prev = new Map<number, BodyPose>();
  private carPrev = { rear: 0, crush: 0 };
  private stepMs = 0;

  private constructor(setup: LabSetup) {
    this.setup = setup;
  }

  static async create(setup: LabSetup): Promise<ImpactLab> {
    const lab = new ImpactLab(setup);
    await lab.reset(setup);
    return lab;
  }

  async reset(setup: LabSetup = this.setup, keepShots = false): Promise<void> {
    this.fw?.dispose();
    this.setup = setup;
    this.fw = await FractureWorld.create(7);
    this.fw.addGround(0);
    this.car = null;
    this.projectiles.length = 0;
    if (!keepShots) this.shots.length = 0;
    this.replayQueue = [];
    this.time = 0;
    this.startedAt = null;
    this.prev.clear();
    this.lastEventTime = -1;
    if (setup.mode === 'crash') {
      const w = WALLS[setup.wall];
      // Concrete sill under the wall (visual and a step for debris to tumble over).
      this.fw.addStaticBox(
        { x: 0.3, y: 0.05, z: 3.2 },
        { position: { x: 0, y: 0.05, z: 0 }, quaternion: { x: 0, y: 0, z: 0, w: 1 } },
      );
      buildWall(this.fw.graph, {
        material: w.material,
        base: { x: 0, y: 0.1, z: 0 },
        length: 6,
        height: 2.1,
        unit: w.unit,
        bond: w.bond,
      });
      this.car = new CrashCar(this.fw, { rear: CAR_START_REAR, speed: 0 });
    } else {
      const t = TARGETS[setup.target];
      const c = TARGET_CENTRE;
      if (t.kind === 'wall')
        buildWall(this.fw.graph, {
          material: t.material,
          base: { x: c.x, y: c.y - t.size.height / 2, z: c.z },
          length: t.size.width,
          height: t.size.height,
          unit: WALLS.brick.unit,
          framed: true,
        });
      else
        buildPanel(this.fw.graph, {
          material: t.material,
          centre: c,
          width: t.size.width,
          height: t.size.height,
          thickness: t.size.thickness,
        });
      // The stand's base beam (the frame itself is drawn around the panel).
      this.fw.addStaticBox(
        { x: 0.3, y: 0.05, z: 1 },
        { position: { x: 0, y: 0.05, z: 0 }, quaternion: { x: 0, y: 0, z: 0, w: 1 } },
      );
    }
    this.fw.sync();
    this.carPrev = { rear: this.car?.rear ?? 0, crush: 0 };
  }

  /** Crash mode: tow the car up to speed. */
  launch(): void {
    if (this.setup.mode !== 'crash' || !this.car || this.startedAt !== null) return;
    this.car.launch(this.setup.speedKmh / 3.6);
    this.startedAt = this.time;
  }

  /** Gallery mode: fire one projectile at the target's centre. */
  fire(projectile: ProjectileId, speed: number): void {
    if (this.setup.mode !== 'gallery') return;
    this.shots.push({ projectile, speed, time: this.time });
    this.spawn(projectile, speed);
  }

  private spawn(projectile: ProjectileId, speed: number): void {
    const spec = PROJECTILES[projectile];
    const d = TARGET_CENTRE.x - CANNON_X;
    // Aim so the ballistic arc passes through the target centre.
    const v: Vec3 = { x: speed, y: (9.81 * d) / (2 * speed), z: 0 };
    const start: Vec3 = { x: CANNON_X + 0.6, y: TARGET_CENTRE.y, z: 0 };
    this.startedAt ??= this.time;
    if (spec.shape === 'melon') {
      const pieces = buildScoredSphere(this.fw.graph, {
        material: M.melon,
        centre: start,
        radius: spec.radius,
        pieces: 10,
        seed: this.shots.length + 3,
        tag: 'melon',
      });
      this.fw.sync();
      const rb = this.fw.bodyOfPiece(pieces[0]!)!;
      rb.setLinvel(v, true);
      rb.setAngvel({ x: 0, y: 0, z: 2 }, true);
      rb.enableCcd(true);
      this.projectiles.push({ id: projectile, pieces, firedAt: this.time });
    } else {
      const desc: ProjectileDesc = {
        shape:
          spec.shape === 'chicken'
            ? { kind: 'convex', points: CHICKEN_POINTS }
            : { kind: 'ball', radius: spec.radius },
        mass: spec.mass,
        position: start,
        velocity: v,
        angularVelocity:
          spec.shape === 'chicken' ? { x: 1.5, y: 3, z: 0.5 } : { x: 0, y: 0, z: -speed / spec.radius / 4 },
        restitution: spec.restitution,
      };
      this.projectiles.push({ id: projectile, rb: this.fw.addProjectile(desc), firedAt: this.time });
    }
    // Keep the gallery tidy.
    while (this.projectiles.length > 16) {
      const old = this.projectiles.shift()!;
      if (old.rb) this.fw.removeProjectile(old.rb);
    }
  }

  /** Restarts the same setup and repeats what happened (deterministic). */
  async replay(): Promise<void> {
    const shots = [...this.shots];
    const launched = this.startedAt !== null;
    await this.reset(this.setup, true);
    if (this.setup.mode === 'crash') {
      if (launched) this.launch();
    } else this.replayQueue = shots.map((s) => ({ ...s }));
  }

  get replaying(): boolean {
    return this.replayQueue.length > 0;
  }

  step(): void {
    const t0 = performance.now();
    // Remember where every moving body was, for interpolation.
    this.prev.clear();
    this.fw.world.forEachRigidBody((rb) => {
      if (!rb.isDynamic() || rb.isSleeping()) return;
      const p = rb.translation();
      const q = rb.rotation();
      this.prev.set(rb.handle, { p: { x: p.x, y: p.y, z: p.z }, q: { x: q.x, y: q.y, z: q.z, w: q.w } });
    });
    while (this.replayQueue.length && this.replayQueue[0]!.time <= this.time + 1e-9) {
      const s = this.replayQueue.shift()!;
      this.spawn(s.projectile, s.speed);
    }
    if (this.car) {
      this.carPrev = { rear: this.car.rear, crush: this.car.crush };
      this.car.preStep(STEP);
    }
    this.fw.step(STEP);
    this.car?.postStep(STEP);
    this.time += STEP;
    this.stepMs = performance.now() - t0;
  }

  /** Body pose between the last two steps (alpha in [0, 1]). */
  bodyPose(rb: RAPIER.RigidBody, alpha: number): Pose {
    const t = rb.translation();
    const r = rb.rotation();
    const cur: BodyPose = { p: { x: t.x, y: t.y, z: t.z }, q: { x: r.x, y: r.y, z: r.z, w: r.w } };
    const prev = this.prev.get(rb.handle);
    if (!prev || alpha >= 1) return { position: cur.p, quaternion: cur.q };
    return {
      position: {
        x: prev.p.x + (cur.p.x - prev.p.x) * alpha,
        y: prev.p.y + (cur.p.y - prev.p.y) * alpha,
        z: prev.p.z + (cur.p.z - prev.p.z) * alpha,
      },
      quaternion: quatSlerp(prev.q, cur.q, alpha),
    };
  }

  /** World pose of a piece, interpolated. */
  piecePose(id: number, alpha: number): Pose {
    const fw = this.fw;
    if (!fw.isLoose(id)) return fw.graph.pieces[id]!.pose;
    const rb = fw.bodyOfPiece(id)!;
    const body = this.bodyPose(rb, alpha);
    const l = fw.pieceLocal(id)!;
    const q = body.quaternion;
    // Rotate the local offset by the body's rotation.
    const lx = l.position.x,
      ly = l.position.y,
      lz = l.position.z;
    const tx = 2 * (q.y * lz - q.z * ly),
      ty = 2 * (q.z * lx - q.x * lz),
      tz = 2 * (q.x * ly - q.y * lx);
    return {
      position: {
        x: body.position.x + lx + q.w * tx + (q.y * tz - q.z * ty),
        y: body.position.y + ly + q.w * ty + (q.z * tx - q.x * tz),
        z: body.position.z + lz + q.w * tz + (q.x * ty - q.y * tx),
      },
      quaternion: {
        x: q.w * l.quaternion.x + q.x * l.quaternion.w + q.y * l.quaternion.z - q.z * l.quaternion.y,
        y: q.w * l.quaternion.y - q.x * l.quaternion.z + q.y * l.quaternion.w + q.z * l.quaternion.x,
        z: q.w * l.quaternion.z + q.x * l.quaternion.y - q.y * l.quaternion.x + q.z * l.quaternion.w,
        w: q.w * l.quaternion.w - q.x * l.quaternion.x - q.y * l.quaternion.y - q.z * l.quaternion.z,
      },
    };
  }

  /** Car rear position and crush, interpolated. */
  carState(alpha: number): { rear: number; crush: number } {
    const c = this.car!;
    return {
      rear: this.carPrev.rear + (c.rear - this.carPrev.rear) * alpha,
      crush: this.carPrev.crush + (c.crush - this.carPrev.crush) * alpha,
    };
  }

  /** Fracture events since the last call. */
  takeEvents(): FractureEvent[] {
    const ev = this.fw.events;
    // The world keeps a bounded list; read what is new by time.
    const out = ev.filter((e) => e.time > this.lastEventTime);
    if (out.length) this.lastEventTime = out[out.length - 1]!.time;
    return out;
  }
  private lastEventTime = -1;

  /** Distance from the impactor to the target, for automatic slow motion (m). */
  approach(): number {
    if (this.setup.mode === 'crash') {
      const c = this.car!;
      if (this.startedAt === null) return Infinity;
      return c.impactTime !== null ? 0 : -0.1 - c.front;
    }
    let best = Infinity;
    for (const p of this.projectiles) {
      const rb = p.rb ?? (p.pieces ? this.fw.bodyOfPiece(p.pieces[0]!) : undefined);
      if (!rb) continue;
      const t = rb.translation();
      const v = rb.linvel();
      // Only shots still on their way in.
      if (v.x <= 0.5 || t.x > TARGET_CENTRE.x + 0.3) continue;
      best = Math.min(best, Math.max(0, TARGET_CENTRE.x - 0.1 - t.x));
    }
    return best;
  }

  stats() {
    const fw = this.fw;
    const live = fw.livePieces();
    const loose = live.filter((p) => fw.isLoose(p.id)).length;
    let broken = 0;
    for (const b of fw.graph.bonds) if (b.broken) broken++;
    const car = this.car;
    return {
      mode: this.setup.mode,
      time: this.time,
      started: this.startedAt !== null,
      pieces: live.length,
      loose,
      bodies: fw.bodyCount,
      jointsBroken: broken,
      joints: fw.graph.bonds.length,
      stepMs: this.stepMs,
      solverMs: fw.stats.solverMs,
      car: car
        ? {
            speedKmh: car.speed * 3.6,
            crush: car.crush,
            front: car.front,
            decelG: car.decel,
            peakG: car.peakDecel,
            chestG: Math.max(...car.occupants.map((o) => o.peakChest)),
            impact: car.impactTime !== null,
            airbags: car.airbagTime !== null,
          }
        : null,
      shots: this.shots.length,
    };
  }

  dispose(): void {
    this.fw.dispose();
  }
}
