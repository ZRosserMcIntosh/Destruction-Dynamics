// The Deep End, wired together without any rendering: event director, authored
// tower, facade failures, budgeted debris physics, floating props with
// buoyancy, the hybrid ocean and particles. The scene view and the cameras
// read from this object; the e2e tests read its stats.

import {
  BudgetGovernor,
  type DegradationLevel,
  FrameProfiler,
  InMemoryTileHost,
  type Pose,
  type QualityPresetId,
  type QualitySettings,
  SeededRandom,
  type TileLease,
  type Vec3,
  WorldStateStore,
  add,
  composePose,
  length,
  qualityPreset,
  quatFromAxisAngle,
  quatIdentity,
  quatRotate,
  scale,
  smoothstep,
  sub,
  tilesForBounds,
  vec3,
} from '@destruction-dynamics/core';
import {
  DEEP_END_TOWER,
  DebrisManager,
  type FacadePlan,
  type PanelLocal,
  TowerCollapse,
  TowerCollision,
  buildFacadePlan,
  facadePanelGeometry,
  panelsDetachingBetween,
  towerCollisionModeAt,
} from '@destruction-dynamics/destruction';
import {
  type DirectorSaveState,
  EventDirector,
  type FiredCue,
  stageWindows,
} from '@destruction-dynamics/events';
import { HybridOcean, type WaterImpactEvent, computeBuoyancy, makeWaves } from '@destruction-dynamics/fluids';
import {
  type BodyHandle,
  type PhysicsAdapter,
  RapierPhysicsAdapter,
  SimplePhysicsAdapter,
} from '@destruction-dynamics/physics';
import { PROP_HALF_EXTENTS, type PropKind, ParticleSim } from '@destruction-dynamics/rendering';
import {
  DEEP_END_LAYOUT,
  SEA_LEVEL,
  deepEndDepth,
  deepEndEvent,
  deepEndGroundHeight,
  deepEndTimeline,
} from '@destruction-dynamics/scenarios';
import {
  type Footprint,
  emitBubbles,
  emitDust,
  emitFoam,
  emitGlass,
  emitMist,
  emitSparks,
  emitSplash,
  emitSpraySheet,
} from './effects';

export interface Floater {
  kind: PropKind;
  handle: BodyHandle;
  pose: Pose;
}

export interface PlaygroundSave {
  schema: 'dd-playground-save/1';
  director: DirectorSaveState;
  world: ReturnType<WorldStateStore['toJSON']>;
  quality: QualityPresetId;
}

const MAX_SPAWNS_PER_FRAME = 30;
/** Cue particle counts are authored for the High preset and scaled from it. */
const AUTHORED_PARTICLE_BUDGET = 15000;

export class DeepEndSimulation {
  readonly def = deepEndEvent;
  readonly timeline = deepEndTimeline();
  readonly director: EventDirector;
  readonly world = new WorldStateStore();
  readonly tiles = new InMemoryTileHost();
  readonly profiler = new FrameProfiler();
  readonly ocean: HybridOcean;
  physics!: PhysicsAdapter;
  tower!: TowerCollapse;
  plan!: FacadePlan;
  panelLocal!: PanelLocal[];
  collision!: TowerCollision;
  debris!: DebrisManager;
  particles!: ParticleSim;
  floaters: Floater[] = [];
  quality: QualitySettings;
  governor: BudgetGovernor;
  degradation: DegradationLevel;
  poses: Pose[] = [];
  /** Bumped whenever tower layout (seed, facade density) changes; views rebuild. */
  layoutVersion = 0;
  /** Bumped on every collision-representation change (stand-in for navmesh rebuilds). */
  navigationVersion = 0;
  readonly cueLog: string[] = [];
  peakWave = 0;
  private prevPoses: Pose[] = [];
  private lease: TileLease | null = null;
  private physicsAccumulator = 0;
  private seeked = true;
  private readonly aboveWater = new Map<number, boolean>();
  private readonly effects: SeededRandom;
  private readonly fluidCues: { time: number; params: Record<string, unknown> }[];

  constructor(qualityId: QualityPresetId, seed: number | string = 'the-deep-end') {
    this.quality = qualityPreset(qualityId);
    this.governor = new BudgetGovernor({ targetFrameMs: this.quality.targetFrameMs });
    this.degradation = this.governor.current;
    this.director = new EventDirector(this.def, { seed });
    this.effects = new SeededRandom(this.director.seed ^ 0x5eed);
    this.ocean = new HybridOcean({
      waves: makeWaves('moderate', 0.35, 7),
      depth: deepEndDepth,
      nearField: { size: this.quality.fluidGridSize, extent: 840, centerX: 260, centerZ: 0, maxDepth: 45 },
      seaLevel: SEA_LEVEL,
    });
    const windows = stageWindows(this.def);
    this.fluidCues = this.def.stages.flatMap((s, i) =>
      s.cues
        .filter((c) => c.channel === 'fluid' && c.type === 'impact')
        .map((c) => ({ time: windows[i]!.start + c.at, params: c.params ?? {} })),
    );
    this.director.on('cue', (c) => this.onCue(c));
    this.director.on('seek', () => (this.seeked = true));
    this.director.on('state', (s) => this.onState(s.to));
  }

  async init(): Promise<void> {
    try {
      const rapier = new RapierPhysicsAdapter();
      await rapier.init();
      this.physics = rapier;
    } catch (err) {
      console.warn('Rapier unavailable, using the simple physics fallback', err);
      this.physics = new SimplePhysicsAdapter();
      await this.physics.init();
    }
    this.buildStaticWorld();
    this.buildTower();
    this.particles = new ParticleSim(this.quality.maxParticles);
    this.debris = new DebrisManager(this.physics, {
      maxBodies: this.quality.maxDebrisBodies,
      killHeight: -80,
    });
    this.collision = new TowerCollision(this.physics, this.tower);
    this.spawnFloaters();
    this.poses = this.tower.posesAt(null);
    this.prevPoses = this.poses;
    this.collision.update('intact', this.poses);
  }

  // -- construction ---------------------------------------------------------

  /** Ground, sand and seabed as static boxes following the bathymetry. */
  private buildStaticWorld(): void {
    const L = DEEP_END_LAYOUT;
    this.physics.createBody({
      kind: 'static',
      shape: { type: 'box', halfExtents: vec3(330, 5, 1600) },
      pose: { position: vec3(L.road.maxX - 330, L.cityLevel - 5, 0), quaternion: quatIdentity() },
    });
    const xs = [L.road.maxX, 60, L.shoreX, 120, 150, 180, 215, L.shelfX, 1200];
    for (let i = 0; i < xs.length - 1; i++) {
      const x0 = xs[i]!,
        x1 = xs[i + 1]!;
      const y0 = deepEndGroundHeight(x0, 0),
        y1 = deepEndGroundHeight(x1, 0);
      const len = Math.hypot(x1 - x0, y1 - y0);
      const angle = Math.atan2(y1 - y0, x1 - x0);
      const q = quatFromAxisAngle(vec3(0, 0, 1), angle);
      const centre = add(vec3((x0 + x1) / 2, (y0 + y1) / 2, 0), quatRotate(q, vec3(0, -3, 0)));
      this.physics.createBody({
        kind: 'static',
        shape: { type: 'box', halfExtents: vec3(len / 2 + 1, 3, 1600) },
        pose: { position: centre, quaternion: q },
      });
    }
  }

  private buildTower(): void {
    this.tower = new TowerCollapse(DEEP_END_TOWER, this.timeline, {
      groundHeight: deepEndGroundHeight,
      seed: this.director.seed,
    });
    this.plan = buildFacadePlan(
      DEEP_END_TOWER,
      this.tower.sections,
      this.timeline,
      this.quality.facadeColumns,
      this.director.seed,
    );
    this.panelLocal = facadePanelGeometry(DEEP_END_TOWER, this.tower.sections, this.plan);
    this.layoutVersion++;
  }

  private spawnFloaters(): void {
    for (const f of this.floaters) this.physics.removeBody(f.handle);
    this.floaters = [];
    const r = new SeededRandom(this.director.seed ^ 0xf10a7);
    const density: Record<PropKind, number> = { boat: 260, buoy: 240, crate: 520 };
    for (let i = 0; i < this.quality.floatingObjects; i++) {
      const kind: PropKind = i % 5 === 0 ? 'buoy' : i % 3 === 0 ? 'crate' : 'boat';
      const x = kind === 'buoy' ? r.range(120, 200) : r.range(150, 650);
      const z = r.range(-420, 420);
      const q = quatFromAxisAngle(vec3(0, 1, 0), r.range(0, Math.PI * 2));
      const pose = { position: vec3(x, 0.2, z), quaternion: q };
      const handle = this.physics.createBody({
        kind: 'dynamic',
        shape: { type: 'box', halfExtents: PROP_HALF_EXTENTS[kind] },
        pose,
        density: density[kind],
        linearDamping: 0.1,
        angularDamping: 0.6,
      });
      this.floaters.push({ kind, handle, pose });
    }
  }

  // -- controls -------------------------------------------------------------

  setQuality(id: QualityPresetId): void {
    const prev = this.quality;
    this.quality = qualityPreset(id);
    this.governor = new BudgetGovernor({ targetFrameMs: this.quality.targetFrameMs });
    this.degradation = this.governor.current;
    if (prev.fluidGridSize !== this.quality.fluidGridSize) {
      this.ocean.resize(this.quality.fluidGridSize);
      this.rebuildFluid(this.director.time);
    }
    if (prev.maxParticles !== this.quality.maxParticles)
      this.particles = new ParticleSim(this.quality.maxParticles);
    this.debris.setLimit(this.quality.maxDebrisBodies);
    if (prev.facadeColumns !== this.quality.facadeColumns) this.rebuildTower();
    if (prev.floatingObjects !== this.quality.floatingObjects) this.spawnFloaters();
  }

  private rebuildTower(): void {
    this.buildTower();
    this.collision.clear();
    this.collision = new TowerCollision(this.physics, this.tower);
    this.seeked = true;
  }

  setSeed(seed: number): void {
    this.director.reset();
    this.director.setSeed(seed);
    this.rebuildTower();
    this.spawnFloaters();
  }

  save(): PlaygroundSave {
    return {
      schema: 'dd-playground-save/1',
      director: this.director.save(),
      world: this.world.toJSON(),
      quality: this.quality.id,
    };
  }

  load(s: PlaygroundSave): void {
    if (s.schema !== 'dd-playground-save/1') throw new Error('not a playground save');
    if (s.director.seed !== this.director.seed) {
      this.director.reset();
      this.director.setSeed(s.director.seed);
      this.rebuildTower();
    }
    this.director.load(s.director);
    const restored = WorldStateStore.fromJSON(s.world);
    this.world.clear(this.def.id);
    for (const t of tilesForBounds(this.def.bounds)) {
      for (const o of restored.forTile(t)) this.world.set(o.eventId, [t], o.state, o.restoreAtMs);
    }
  }

  // -- event reactions ------------------------------------------------------

  private onState(to: string): void {
    if (to === 'PREPARING' || to === 'TRIGGERED') {
      this.lease ??= this.tiles.acquire(tilesForBounds(this.def.bounds), this.def.id);
    }
    if (to === 'INTACT') {
      this.lease?.release();
      this.lease = null;
      this.world.clear(this.def.id);
      this.peakWave = 0;
      this.seeked = true;
    }
    if (to === 'RESTORATION') {
      this.debris?.clear();
      this.particles?.clear();
    }
  }

  private particleScale(): number {
    return (this.quality.maxParticles / AUTHORED_PARTICLE_BUDGET) * this.degradation.particleScale;
  }

  private footprint(): Footprint & { direction: Vec3 } {
    return this.tower.impactFootprint(SEA_LEVEL);
  }

  private impactEvent(params: Record<string, unknown>): WaterImpactEvent {
    const f = this.footprint();
    return {
      position: f.position,
      footprint: { kind: 'box', halfLength: f.halfLength, halfWidth: f.halfWidth, yaw: f.yaw },
      intensity: typeof params.intensity === 'number' ? params.intensity : 1,
      duration: typeof params.duration === 'number' ? params.duration : 1.5,
      direction: f.direction,
      visualProfile: typeof params.visualProfile === 'string' ? params.visualProfile : 'landmark-collapse',
    };
  }

  private onCue(c: FiredCue): void {
    this.cueLog.push(`${c.time.toFixed(1)}s ${c.cue.channel}:${c.cue.type}`);
    if (this.cueLog.length > 10) this.cueLog.shift();
    const p = c.cue.params ?? {};
    const r = new SeededRandom(c.seed);
    const n = (k: unknown) => Math.round((typeof k === 'number' ? k : 100) * this.particleScale());
    const breakY = DEEP_END_TOWER.base.y + DEEP_END_TOWER.breakFloor * DEEP_END_TOWER.floorHeight;
    switch (`${c.cue.channel}:${c.cue.type}`) {
      case 'particles:dust':
        if (c.lateBy < 1) emitDust(this.particles, r, vec3(0, breakY, 0), Number(p.spread ?? 20), n(p.count));
        break;
      case 'particles:sparks':
        if (c.lateBy < 1) emitSparks(this.particles, r, vec3(0, breakY, 0), n(p.count));
        break;
      case 'particles:spray-sheet':
        if (c.lateBy < 1) emitSpraySheet(this.particles, r, this.footprint(), n(p.count), 1);
        break;
      case 'particles:mist':
        if (c.lateBy < 1) emitMist(this.particles, r, this.footprint(), n(p.count));
        break;
      case 'particles:bubbles':
        if (c.lateBy < 1)
          emitBubbles(this.particles, r, this.wreckCentres(), deepEndGroundHeight, n(p.count));
        break;
      case 'fluid:impact':
        this.ocean.applyImpact(this.impactEvent(p), Math.max(0, c.lateBy));
        if (c.lateBy < 1) emitFoam(this.particles, r, this.footprint(), n(800));
        break;
      case 'world:override':
        this.world.set(
          this.def.id,
          tilesForBounds(this.def.bounds),
          String(p.state ?? 'wreck'),
          this.director.restorationDueAtMs,
        );
        break;
    }
  }

  private wreckCentres(): Vec3[] {
    return this.poses.slice(1).map((pose, k) => this.tower.sectionCenter(k + 1, pose));
  }

  /** After a seek or load: drop transient state and re-create surges that should still be running. */
  private rebuildFluid(t: number): void {
    this.ocean.reset();
    if (!this.director.committed) return;
    for (const c of this.fluidCues) {
      const age = t - c.time;
      if (age >= 0 && age < 90) this.ocean.applyImpact(this.impactEvent(c.params), age);
    }
  }

  private onSeek(): void {
    this.debris.clear();
    this.particles.clear();
    this.aboveWater.clear();
    this.rebuildFluid(this.director.time);
  }

  // -- per frame ------------------------------------------------------------

  /** Event time used for the tower (null while intact). */
  get towerTime(): number | null {
    const d = this.director;
    return d.state === 'INTACT' || d.state === 'PREPARING' ? null : d.time;
  }

  update(dtReal: number, frameMs?: number): void {
    const d = this.director;
    const pf = this.profiler;
    pf.beginFrame();
    const prevTime = d.time;
    const wasCommitted = d.committed;
    pf.measure('director', () => d.update(dtReal));
    const simDt = d.paused ? 0 : dtReal * d.timeScale;
    if (this.seeked) {
      this.onSeek();
      this.seeked = false;
      this.prevPoses = this.tower.posesAt(this.towerTime, d.restorationProgress);
    } else this.prevPoses = this.poses;
    this.poses = this.tower.posesAt(this.towerTime, d.restorationProgress);

    if (wasCommitted && d.committed && d.time > prevTime && simDt > 0)
      this.detachPanels(prevTime, d.time, simDt);

    const mode = towerCollisionModeAt(d.state, d.time, this.timeline);
    if (mode !== this.collision.mode) this.navigationVersion++;
    this.collision.update(mode, this.poses);

    pf.measure('physics', () => this.stepPhysics(simDt));
    pf.measure('fluid', () => {
      if (simDt > 0) this.ocean.step(simDt);
      this.peakWave = Math.max(this.peakWave, this.ocean.maxDisturbance());
    });
    pf.measure('particles', () => {
      this.continuousEffects(simDt);
      this.particles.update(simDt, {
        gravity: 9.81,
        waterHeight: (x, z) => this.ocean.heightAt(x, z),
        groundHeight: deepEndGroundHeight,
      });
    });
    const ms = frameMs ?? 0;
    if (frameMs !== undefined) {
      this.degradation = this.governor.observe(ms);
      this.particles.limit = Math.floor(this.quality.maxParticles * this.degradation.particleScale);
      const debrisLimit = Math.floor(this.quality.maxDebrisBodies * this.degradation.debrisScale);
      if (debrisLimit !== this.debris.maxBodies) this.debris.setLimit(debrisLimit);
      this.ocean.stepDivisor = this.degradation.fluidStepDivisor;
    }
  }

  private detachPanels(from: number, to: number, dt: number): void {
    const ids = panelsDetachingBetween(this.plan, from, to);
    if (ids.length === 0) return;
    let spawned = 0;
    const glassPer = Math.max(1, Math.round(8 * this.particleScale()));
    for (const i of ids) {
      const panel = this.plan.panels[i]!;
      const local = this.panelLocal[i]!;
      const now = composePose(this.poses[panel.section]!, {
        position: local.center,
        quaternion: quatIdentity(),
      });
      const before = composePose(this.prevPoses[panel.section]!, {
        position: local.center,
        quaternion: quatIdentity(),
      });
      let vel = scale(sub(now.position, before.position), 1 / dt);
      if (length(vel) > 80) vel = scale(vel, 80 / length(vel));
      const normal = quatRotate(now.quaternion, local.normal);
      const r = new SeededRandom(this.director.seed ^ (i * 2654435761));
      emitGlass(this.particles, r, now.position, vel, normal, glassPer);
      if (spawned >= MAX_SPAWNS_PER_FRAME) continue;
      const chunk = r.chance(0.25);
      const body = this.debris.spawn({
        kind: chunk ? 'chunk' : 'panel',
        pose: now,
        halfExtents: chunk
          ? vec3(r.range(0.4, 1.3), r.range(0.3, 0.8), r.range(0.4, 1.3))
          : local.halfExtents,
        velocity: add(vel, scale(normal, r.range(1, 4))),
        angularVelocity: vec3(r.range(-2, 2), r.range(-2, 2), r.range(-2, 2)),
        lifetime: r.range(20, 40),
        density: chunk ? 2400 : 1300,
      });
      if (body) spawned++;
    }
  }

  private stepPhysics(simDt: number): void {
    if (simDt <= 0) return;
    const h = 1 / this.quality.physicsHz;
    this.physicsAccumulator += simDt;
    let steps = 0;
    while (this.physicsAccumulator >= h && steps < 6) {
      this.applyWaterForces();
      this.physics.step(h);
      this.physicsAccumulator -= h;
      steps++;
    }
    // Secondary simulation may fall behind under heavy time scaling; drop the backlog.
    if (steps === 6) this.physicsAccumulator = 0;
    this.debris.update(steps * h);
    for (const f of this.floaters) f.pose = this.physics.getPose(f.handle);
    this.checkSplashes();
  }

  private applyWaterForces(): void {
    for (const f of this.floaters) {
      const pose = this.physics.getPose(f.handle);
      const b = computeBuoyancy(
        {
          pose,
          halfExtents: PROP_HALF_EXTENTS[f.kind],
          velocity: this.physics.getLinearVelocity(f.handle),
          mass: this.physics.getMass(f.handle),
        },
        this.ocean,
        { resolution: 2, drag: 1.2 },
      );
      for (const force of b.forces) this.physics.addForce(f.handle, force.force, force.point);
    }
    for (const body of this.debris.active) {
      const y = body.pose.position.y;
      if (y > 6 || y < -60 || this.physics.isSleeping(body.handle)) continue;
      const b = computeBuoyancy(
        {
          pose: body.pose,
          halfExtents: body.halfExtents,
          velocity: this.physics.getLinearVelocity(body.handle),
          mass: this.physics.getMass(body.handle),
        },
        this.ocean,
        { resolution: 1, drag: 2.5 },
      );
      for (const force of b.forces) this.physics.addForce(body.handle, force.force, force.point);
    }
  }

  private checkSplashes(): void {
    for (const body of this.debris.active) {
      const p = body.pose.position;
      const above = p.y > this.ocean.heightAt(p.x, p.z);
      const was = this.aboveWater.get(body.id) ?? true;
      this.aboveWater.set(body.id, above);
      if (was && !above) {
        const v = this.physics.getLinearVelocity(body.handle);
        if (v.y < -3) {
          this.ocean.applyDisturbance(p, 2.5, Math.min(3, -v.y * 0.15));
          emitSplash(this.particles, this.effects, p, -v.y, Math.round(12 * this.particleScale()) + 2);
        }
      }
    }
    if (this.aboveWater.size > this.debris.active.length * 2 + 64) {
      const live = new Set(this.debris.active.map((b) => b.id));
      for (const id of this.aboveWater.keys()) if (!live.has(id)) this.aboveWater.delete(id);
    }
  }

  /** Spray while the tower ploughs in, bubbles while the wreck settles. */
  private continuousEffects(simDt: number): void {
    if (simDt <= 0 || !this.director.committed) return;
    const t = this.director.time;
    const age = t - this.timeline.impact;
    const s = this.particleScale();
    if (age >= 0 && age < 2.5) {
      const count = Math.round(2500 * s * simDt * (1 - age / 2.5));
      if (count > 0) emitSpraySheet(this.particles, this.effects, this.footprint(), count, 0.6);
    }
    if (t >= this.timeline.impact + 1 && t < this.timeline.settleEnd + 20) {
      const count = Math.round(120 * s * simDt);
      if (count > 0)
        emitBubbles(this.particles, this.effects, this.wreckCentres(), deepEndGroundHeight, count);
    }
  }

  /** 0..1 suspended material in the water, for underwater visibility. */
  get turbidity(): number {
    const t = this.towerTime;
    if (t === null) return 0.1;
    return (
      0.1 +
      0.9 *
        smoothstep(this.timeline.impact - 0.5, this.timeline.impact + 1, t) *
        (1 - smoothstep(this.timeline.impact + 4, this.timeline.settleEnd + 40, t))
    );
  }

  stats() {
    const counts = this.physics.counts();
    return {
      state: this.director.state,
      time: this.director.time,
      stage: this.director.stage?.id ?? null,
      paused: this.director.paused,
      timeScale: this.director.timeScale,
      seed: this.director.seed,
      tiltDeg: (this.tower.tiltAt(this.towerTime) * 180) / Math.PI,
      bodies: counts,
      debris: { ...this.debris.stats, limit: this.debris.maxBodies },
      particles: this.particles.count,
      particleLimit: this.particles.limit,
      fluid: {
        grid: this.ocean.near.n,
        substeps: this.ocean.near.lastSubsteps,
        maxDisturbance: this.ocean.maxDisturbance(),
        peak: this.peakWave,
        surges: this.ocean.surges.length,
      },
      collisionMode: this.collision.mode,
      navigationVersion: this.navigationVersion,
      degradation: this.degradation.level,
      activeTiles: this.tiles.activeTiles(),
      worldOverrides: tilesForBounds(this.def.bounds).flatMap((t) => this.world.forTile(t)).length,
      costs: this.profiler.costs(),
      physicsBackend: this.physics.name,
    };
  }
}
