// Binds a BondGraph to Rapier. Each island of joined pieces is one rigid body
// (fixed while it is still joined to the world, dynamic once it is not), with
// one collider per piece. After every step, contact impulses above a static
// threshold are applied to the graph as impacts; when joints break or pieces
// shatter, the affected islands are rebuilt as new bodies.
//
// Breaking through: a rigid solver stops an impactor against a fixed wall in
// one step, before the wall has had a chance to break. When an impact frees
// pieces near the contact, the impactor's pre-step momentum is shared with the
// freed pieces instead (a perfectly inelastic hit), so a fast enough impactor
// carries on through.

import { type Pose, type Vec3, quatRotate } from '@destruction-dynamics/core';
import { initRapier } from '@destruction-dynamics/physics';
import RAPIER from '@dimforge/rapier3d-compat';
import { BondGraph, type Piece, toWorld } from './bondGraph';

interface BodyRec {
  rb: RAPIER.RigidBody;
  pieces: number[];
  fixed: boolean;
}

export interface FractureEvent {
  kind: 'hit' | 'break' | 'shatter';
  point: Vec3;
  impulse: number;
  material: string;
  /** Joints broken plus cells made. */
  count: number;
  time: number;
}

export type ProjectileShape = { kind: 'ball'; radius: number } | { kind: 'convex'; points: Vec3[] };

export interface ProjectileDesc {
  shape: ProjectileShape;
  mass: number;
  position: Vec3;
  velocity: Vec3;
  angularVelocity?: Vec3;
  friction?: number;
  restitution?: number;
}

export interface StrikeResult {
  broke: boolean;
  /** Dynamic bodies freed near the strike point by this strike. */
  freed: RAPIER.RigidBody[];
}

const G = 9.81;

// Collision groups (membership << 16 | filter). Small chips skip each other
// and the larger debris: they only land on the ground and get hit by
// impactors. That removes most contact pairs in a fresh debris field.
const STATIC = 1,
  DEBRIS = 2,
  CHIP = 4,
  IMPACTOR = 8;
const groups = (member: number, filter: number) => (member << 16) | filter;
export const GROUP_STATIC = groups(STATIC, STATIC | DEBRIS | CHIP | IMPACTOR);
export const GROUP_DEBRIS = groups(DEBRIS, STATIC | DEBRIS | IMPACTOR);
export const GROUP_CHIP = groups(CHIP, STATIC | IMPACTOR);
export const GROUP_IMPACTOR = groups(IMPACTOR, STATIC | DEBRIS | CHIP | IMPACTOR);
/** Steps between gravity sag passes, and how fast hanging joints wear (×). */
const SAG_EVERY = 6;
const SAG_RATE = 6;
/** Loose single pieces smaller than this (bounding radius, m) are chips. */
export const CHIP_RADIUS = 0.06;
/** Contact forces below this many body weights are load, not impact. */
const STATIC_LOAD_FACTOR = 20;

export class FractureWorld {
  /**
   * The step the materials are tuned for (s). Shattering compares the impulse
   * of one step against a threshold, so always step with this.
   */
  static readonly STEP = 1 / 120;
  readonly graph: BondGraph;
  readonly world: RAPIER.World;
  private readonly queue: RAPIER.EventQueue;
  private readonly bodies = new Map<number, BodyRec>();
  private readonly pieceBody: (number | undefined)[] = [];
  private readonly localPose: Pose[] = [];
  private readonly colliderPiece = new Map<number, number>();
  private readonly projectiles = new Map<number, RAPIER.RigidBody>();
  private readonly colliderProjectile = new Map<number, number>();
  private readonly fixedBounds = new Map<number, { min: Vec3; max: Vec3 }>();
  private pendingRollback: { rb: RAPIER.RigidBody; v0: Vec3; point: Vec3; reach: number }[] = [];
  private dirty = false;
  /** Bodies whose piece poses are current this step. */
  private readonly refreshed = new Set<number>();
  /** Body a freshly made shard came from (for its starting velocity). */
  private readonly origin = new Map<number, number>();
  private createdThisSync = new Set<number>();
  /** Recent events, for sound and effects. The caller drains it. */
  readonly events: FractureEvent[] = [];
  time = 0;
  private stepCount = 0;
  /** Timings of the last step (ms) and the contact events it handled. */
  readonly stats = { solverMs: 0, eventsMs: 0, syncMs: 0, events: 0 };
  /** Bumped whenever bodies are created or removed. */
  version = 0;

  static async create(seed = 1): Promise<FractureWorld> {
    await initRapier();
    return new FractureWorld(seed);
  }

  /** Use create() unless Rapier is already initialised. */
  constructor(seed = 1) {
    this.graph = new BondGraph(seed);
    this.world = new RAPIER.World({ x: 0, y: -G, z: 0 });
    this.world.integrationParameters.numSolverIterations = 4;
    this.queue = new RAPIER.EventQueue(true);
  }

  dispose(): void {
    this.world.free();
    this.queue.free();
  }

  /** An infinite-ish flat ground with its top at y. */
  addGround(y = 0, friction = 0.8): void {
    const rb = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, y - 1, 0));
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(500, 1, 500).setFriction(friction).setCollisionGroups(GROUP_STATIC),
      rb,
    );
  }

  /** A fixed, unbreakable box (frames, stands, sills). */
  addStaticBox(half: Vec3, pose: Pose, friction = 0.6): RAPIER.RigidBody {
    const rb = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(pose.position.x, pose.position.y, pose.position.z)
        .setRotation(pose.quaternion),
    );
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z)
        .setFriction(friction)
        .setCollisionGroups(GROUP_STATIC),
      rb,
    );
    return rb;
  }

  /** A rigid, unbreakable projectile with continuous collision detection. */
  addProjectile(d: ProjectileDesc): RAPIER.RigidBody {
    const rb = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(d.position.x, d.position.y, d.position.z)
        .setLinvel(d.velocity.x, d.velocity.y, d.velocity.z)
        .setAngvel(d.angularVelocity ?? { x: 0, y: 0, z: 0 })
        .setCcdEnabled(true),
    );
    const cd =
      d.shape.kind === 'ball'
        ? RAPIER.ColliderDesc.ball(d.shape.radius)
        : (RAPIER.ColliderDesc.convexHull(flat(d.shape.points)) ?? RAPIER.ColliderDesc.ball(0.1));
    cd.setMass(d.mass)
      .setFriction(d.friction ?? 0.5)
      .setRestitution(d.restitution ?? 0.3)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(STATIC_LOAD_FACTOR * d.mass * G)
      .setCollisionGroups(GROUP_IMPACTOR);
    const col = this.world.createCollider(cd, rb);
    this.projectiles.set(rb.handle, rb);
    this.colliderProjectile.set(col.handle, rb.handle);
    return rb;
  }

  removeProjectile(rb: RAPIER.RigidBody): void {
    for (let i = 0; i < rb.numColliders(); i++) this.colliderProjectile.delete(rb.collider(i).handle);
    this.projectiles.delete(rb.handle);
    this.world.removeRigidBody(rb);
  }

  /**
   * Brings bodies in line with the graph's islands. An island whose pieces all
   * sit in one body of the right kind keeps that body (the largest such island
   * claims it) and only gains or loses colliders; other islands get new bodies
   * that start with the velocity of the body they came from.
   */
  sync(): void {
    const islands = this.graph.islands().sort((a, b) => b.pieces.length - a.pieces.length);
    const claimed = new Map<number, (typeof islands)[number]>();
    const pending: typeof islands = [];
    for (const isl of islands) {
      let h: number | undefined;
      let one = true;
      for (const p of isl.pieces) {
        const ph = this.pieceBody[p];
        if (ph === undefined || !this.bodies.has(ph)) continue;
        if (h === undefined) h = ph;
        else if (h !== ph) {
          one = false;
          break;
        }
      }
      if (one && h !== undefined && !claimed.has(h) && this.bodies.get(h)!.fixed === isl.anchored)
        claimed.set(h, isl);
      else pending.push(isl);
    }
    // Which bodies change at all?
    const changed = new Set<number>();
    for (const [h, rec] of this.bodies) {
      const isl = claimed.get(h);
      if (
        !isl ||
        isl.pieces.length !== rec.pieces.length ||
        !isl.pieces.every((p) => this.pieceBody[p] === h)
      )
        changed.add(h);
    }
    if (!changed.size && !pending.length) return;
    const old = new Map<number, { lin: Vec3; ang: Vec3; com: Vec3 }>();
    for (const h of changed) {
      const rec = this.bodies.get(h)!;
      this.refreshBody(h);
      const t = rec.rb.translation();
      old.set(h, {
        lin: rec.fixed ? { x: 0, y: 0, z: 0 } : rec.rb.linvel(),
        ang: rec.fixed ? { x: 0, y: 0, z: 0 } : rec.rb.angvel(),
        com: rec.fixed ? { x: t.x, y: t.y, z: t.z } : rec.rb.worldCom(),
      });
    }
    this.createdThisSync = new Set();
    // Origin body of every piece, before anything moves.
    const fromBody = (p: number) => this.pieceBody[p] ?? this.origin.get(p);
    const pendingFrom = pending.map((isl) => {
      const src = isl.pieces.find((p) => fromBody(p) !== undefined);
      return src === undefined ? undefined : fromBody(src);
    });
    // Claimed bodies shed the pieces that left and take on new cells.
    for (const [h, isl] of claimed) {
      if (!changed.has(h)) continue;
      const rec = this.bodies.get(h)!;
      const keep = new Set(isl.pieces);
      for (const p of rec.pieces) if (!keep.has(p)) this.detach(p);
      for (const p of isl.pieces) if (this.pieceBody[p] !== h) this.attach(rec, p);
      rec.pieces = isl.pieces;
      if (rec.fixed) this.updateBounds(h);
    }
    pending.forEach((isl, i) => {
      const f = pendingFrom[i];
      for (const p of isl.pieces)
        if (this.pieceBody[p] !== undefined && this.bodies.has(this.pieceBody[p]!)) this.detach(p);
      this.createBody(isl.pieces, isl.anchored, f !== undefined ? old.get(f) : undefined);
    });
    // Bodies left with nothing.
    for (const [h, rec] of [...this.bodies]) {
      if (claimed.has(h) || this.createdThisSync.has(h)) continue;
      for (const p of rec.pieces) if (this.pieceBody[p] === h) this.detach(p);
      this.world.removeRigidBody(rec.rb);
      this.bodies.delete(h);
      this.fixedBounds.delete(h);
    }
    this.origin.clear();
    this.version++;
  }

  private readonly pieceCollider: (RAPIER.Collider | undefined)[] = [];

  private detach(p: number): void {
    const col = this.pieceCollider[p];
    if (col) {
      this.colliderPiece.delete(col.handle);
      if (this.world.getCollider(col.handle)) this.world.removeCollider(col, true);
    }
    this.pieceCollider[p] = undefined;
    this.pieceBody[p] = undefined;
  }

  /** Adds a piece's collider to a body, at the piece's current world pose. */
  private attach(rec: BodyRec, id: number): void {
    const p = this.graph.pieces[id]!;
    const t = rec.rb.translation();
    const q = rec.rb.rotation();
    const body: Pose = {
      position: { x: t.x, y: t.y, z: t.z },
      quaternion: { x: q.x, y: q.y, z: q.z, w: q.w },
    };
    const inv = { x: -q.x, y: -q.y, z: -q.z, w: q.w };
    const local: Pose = {
      position: quatRotate(inv, {
        x: p.pose.position.x - body.position.x,
        y: p.pose.position.y - body.position.y,
        z: p.pose.position.z - body.position.z,
      }),
      quaternion: quatMul(inv, p.pose.quaternion),
    };
    const cd = colliderFor(p);
    cd.setTranslation(local.position.x, local.position.y, local.position.z)
      .setRotation(local.quaternion)
      .setDensity(p.material.density)
      .setFriction(p.material.friction)
      .setRestitution(p.material.restitution)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(rec.fixed ? 1 : Math.max(1, STATIC_LOAD_FACTOR * p.mass * G))
      .setCollisionGroups(
        rec.fixed
          ? GROUP_STATIC
          : rec.pieces.length <= 1 && pieceRadius(p) < CHIP_RADIUS
            ? GROUP_CHIP
            : GROUP_DEBRIS,
      );
    const col = this.world.createCollider(cd, rec.rb);
    this.colliderPiece.set(col.handle, id);
    this.pieceCollider[id] = col;
    this.pieceBody[id] = rec.rb.handle;
    this.localPose[id] = local;
  }

  private updateBounds(h: number): void {
    let min = { x: Infinity, y: Infinity, z: Infinity };
    let max = { x: -Infinity, y: -Infinity, z: -Infinity };
    for (const id of this.bodies.get(h)!.pieces) {
      const b = this.fixedPieceBounds(id);
      min = { x: Math.min(min.x, b.min.x), y: Math.min(min.y, b.min.y), z: Math.min(min.z, b.min.z) };
      max = { x: Math.max(max.x, b.max.x), y: Math.max(max.y, b.max.y), z: Math.max(max.z, b.max.z) };
    }
    this.fixedBounds.set(h, { min, max });
  }

  private createBody(
    pieces: number[],
    anchored: boolean,
    from: { lin: Vec3; ang: Vec3; com: Vec3 } | undefined,
  ): void {
    let mass = 0;
    let cx = 0,
      cy = 0,
      cz = 0;
    for (const id of pieces) {
      const p = this.graph.pieces[id]!;
      mass += p.mass;
      cx += p.mass * p.pose.position.x;
      cy += p.mass * p.pose.position.y;
      cz += p.mass * p.pose.position.z;
    }
    const com = { x: cx / mass, y: cy / mass, z: cz / mass };
    const desc = anchored ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic();
    desc.setTranslation(com.x, com.y, com.z);
    if (!anchored && from) {
      const r = { x: com.x - from.com.x, y: com.y - from.com.y, z: com.z - from.com.z };
      const w = from.ang;
      desc
        .setLinvel(
          from.lin.x + w.y * r.z - w.z * r.y,
          from.lin.y + w.z * r.x - w.x * r.z,
          from.lin.z + w.x * r.y - w.y * r.x,
        )
        .setAngvel(w);
    }
    const rb = this.world.createRigidBody(desc);
    const rec: BodyRec = { rb, pieces, fixed: anchored };
    this.bodies.set(rb.handle, rec);
    this.createdThisSync.add(rb.handle);
    for (const id of pieces) this.attach(rec, id);
    if (anchored) this.updateBounds(rb.handle);
  }

  /** Writes the current world pose of every piece in a body back to the graph. */
  private refreshBody(h: number | undefined): void {
    if (h === undefined) return;
    const rec = this.bodies.get(h);
    // Fixed bodies never move; a moving one only needs it once per step.
    if (!rec || rec.fixed || this.refreshed.has(h)) return;
    this.refreshed.add(h);
    const t = rec.rb.translation();
    const q = rec.rb.rotation();
    const body: Pose = {
      position: { x: t.x, y: t.y, z: t.z },
      quaternion: { x: q.x, y: q.y, z: q.z, w: q.w },
    };
    for (const id of rec.pieces) {
      const l = this.localPose[id]!;
      const p = this.graph.pieces[id]!;
      p.pose = {
        position: toWorld(body, l.position),
        quaternion: quatMul(body.quaternion, l.quaternion),
      };
    }
  }

  /** Current world pose of a piece. */
  piecePose(id: number): Pose {
    const h = this.pieceBody[id];
    const rec = h !== undefined ? this.bodies.get(h) : undefined;
    const p = this.graph.pieces[id]!;
    if (!rec || rec.fixed) return p.pose;
    const t = rec.rb.translation();
    const q = rec.rb.rotation();
    const l = this.localPose[id]!;
    const qq = { x: q.x, y: q.y, z: q.z, w: q.w };
    const r = quatRotate(qq, l.position);
    return {
      position: { x: t.x + r.x, y: t.y + r.y, z: t.z + r.z },
      quaternion: quatMul(qq, l.quaternion),
    };
  }

  /** Whether a piece is currently part of a moving body. */
  isLoose(id: number): boolean {
    const h = this.pieceBody[id];
    const rec = h !== undefined ? this.bodies.get(h) : undefined;
    return !!rec && !rec.fixed;
  }

  /** A piece's pose relative to its body (for interpolated rendering). */
  pieceLocal(id: number): Pose | undefined {
    return this.localPose[id];
  }

  bodyOfPiece(id: number): RAPIER.RigidBody | undefined {
    const h = this.pieceBody[id];
    return h !== undefined ? this.bodies.get(h)?.rb : undefined;
  }

  private readonly boundsCache: ({ min: Vec3; max: Vec3 } | undefined)[] = [];

  /** Bounds of a piece in a fixed body (cached: fixed pieces never move). */
  fixedPieceBounds(id: number): { min: Vec3; max: Vec3 } {
    let b = this.boundsCache[id];
    if (!b) {
      b = pieceBounds(this.graph.pieces[id]!);
      this.boundsCache[id] = b;
    }
    return b;
  }

  /** Live pieces of fixed (still standing) bodies whose bounds overlap the box. */
  fixedPiecesInBox(min: Vec3, max: Vec3): number[] {
    const out: number[] = [];
    for (const [h, b] of this.fixedBounds) {
      if (
        b.max.x < min.x ||
        b.min.x > max.x ||
        b.max.y < min.y ||
        b.min.y > max.y ||
        b.max.z < min.z ||
        b.min.z > max.z
      )
        continue;
      for (const id of this.bodies.get(h)!.pieces) {
        const pb = this.fixedPieceBounds(id);
        if (
          pb.max.x < min.x ||
          pb.min.x > max.x ||
          pb.max.y < min.y ||
          pb.min.y > max.y ||
          pb.max.z < min.z ||
          pb.min.z > max.z
        )
          continue;
        out.push(id);
      }
    }
    return out;
  }

  /**
   * Applies impulses from outside the rigid-body solver (a crushing car, a
   * scripted blow) in one batch, rebuilds bodies once, and returns the bodies
   * this freed. A hit's `radius` overrides the material's damage radius.
   */
  strike(hits: readonly { piece: number; point: Vec3; impulse: number; radius?: number }[]): StrikeResult {
    const before = this.version;
    this.refreshed.clear();
    for (const h of hits)
      if (this.graph.pieces[h.piece]?.alive) this.impact(h.piece, h.point, h.impulse, h.radius);
    if (this.dirty) {
      this.dirty = false;
      this.sync();
    }
    const freed: RAPIER.RigidBody[] = [];
    if (this.version !== before)
      for (const h of this.createdThisSync) {
        const rec = this.bodies.get(h);
        if (rec && !rec.fixed) freed.push(rec.rb);
      }
    return { broke: this.version !== before, freed };
  }

  private freedNear(point: Vec3, reach: number): RAPIER.RigidBody[] {
    const out: RAPIER.RigidBody[] = [];
    for (const h of this.createdThisSync) {
      const rec = this.bodies.get(h);
      if (!rec || rec.fixed) continue;
      const near = rec.pieces.some((id) => {
        const p = this.graph.pieces[id]!.pose.position;
        return (
          Math.hypot(p.x - point.x, p.y - point.y, p.z - point.z) <
          reach + pieceRadius(this.graph.pieces[id]!)
        );
      });
      if (near) out.push(rec.rb);
    }
    return out;
  }

  private impact(piece: number, point: Vec3, impulse: number, radius?: number): boolean {
    this.refreshBody(this.pieceBody[piece]);
    const p = this.graph.pieces[piece]!;
    const r = this.graph.applyImpact(piece, point, impulse, radius);
    const src = this.pieceBody[piece];
    if (src !== undefined) for (const c of r.created) this.origin.set(c, src);
    const changed = r.broken > 0 || r.shattered.length > 0;
    if (changed) this.dirty = true;
    this.events.push({
      kind: r.shattered.length ? 'shatter' : r.broken ? 'break' : 'hit',
      point,
      impulse,
      material: p.material.id,
      count: r.broken + r.created.length,
      time: this.time,
    });
    if (this.events.length > 512) this.events.splice(0, this.events.length - 512);
    return changed;
  }

  step(dt: number): void {
    const v0 = new Map<number, Vec3>();
    for (const [h, rb] of this.projectiles) v0.set(h, rb.linvel());
    const t0 = performance.now();
    this.refreshed.clear();
    this.world.timestep = dt;
    this.world.step(this.queue);
    this.time += dt;
    const t1 = performance.now();
    const hits: { c1: number; c2: number; f: number }[] = [];
    this.queue.drainContactForceEvents((e) => {
      hits.push({ c1: e.collider1(), c2: e.collider2(), f: e.totalForceMagnitude() });
    });
    for (const { c1, c2, f } of hits) {
      const p1 = this.colliderPiece.get(c1);
      const p2 = this.colliderPiece.get(c2);
      if (p1 === undefined && p2 === undefined) continue;
      const col1 = this.world.getCollider(c1);
      const col2 = this.world.getCollider(c2);
      if (!col1 || !col2) continue;
      // Ignore load: forces a resting pile carries.
      const m1 = bodyMass(col1),
        m2 = bodyMass(col2);
      const light = Math.min(m1, m2);
      if (f < STATIC_LOAD_FACTOR * light * G) continue;
      const point = this.contactPoint(col1, col2);
      const impulse = f * dt;
      // A projectile concentrates its load on a patch about its own size.
      const ph1 = this.colliderProjectile.get(c1),
        ph2 = this.colliderProjectile.get(c2);
      const proj =
        ph1 !== undefined
          ? this.projectiles.get(ph1)
          : ph2 !== undefined
            ? this.projectiles.get(ph2)
            : undefined;
      const radiusFor = (p: number) =>
        proj ? Math.min(this.graph.pieces[p]!.material.damageRadius, 3 * projectileRadius(proj)) : undefined;
      let changed = false;
      if (p1 !== undefined && this.graph.pieces[p1]?.alive)
        changed = this.impact(p1, point, impulse, radiusFor(p1)) || changed;
      if (p2 !== undefined && this.graph.pieces[p2]?.alive)
        changed = this.impact(p2, point, impulse, radiusFor(p2)) || changed;
      if (changed) {
        for (const c of [c1, c2]) {
          const ph = this.colliderProjectile.get(c);
          const rb = ph !== undefined ? this.projectiles.get(ph) : undefined;
          if (rb && v0.has(ph!))
            this.pendingRollback.push({ rb, v0: v0.get(ph!)!, point, reach: projectileRadius(rb) + 0.25 });
        }
      }
    }
    if (++this.stepCount % SAG_EVERY === 0) this.sag(dt * SAG_EVERY);
    const t2 = performance.now();
    if (this.dirty) {
      this.dirty = false;
      this.sync();
      for (const r of this.pendingRollback) {
        const freed = this.freedNear(r.point, r.reach);
        if (freed.length) shareMomentum(r.rb, r.v0, freed);
      }
    }
    this.pendingRollback = [];
    const t3 = performance.now();
    this.stats.solverMs = t1 - t0;
    this.stats.eventsMs = t2 - t1;
    this.stats.syncMs = t3 - t2;
    this.stats.events = hits.length;
  }

  /**
   * Masonry that has lost everything underneath it hangs on its side and top
   * joints, which are weak in shear: its weight wears them down until it
   * drops. Over an opening in running bond this peels away the units that are
   * not overlapped by a supported unit below, leaving the stepped triangle real
   * walls leave above a hole.
   */
  private sag(interval: number): void {
    for (const rec of this.bodies.values()) {
      if (!rec.fixed) continue;
      for (const id of rec.pieces) {
        const p = this.graph.pieces[id]!;
        if (!Number.isFinite(p.material.bondStrength)) continue;
        let below = false;
        let n = 0;
        for (const b of this.graph.intact(id)) {
          n++;
          if (b.b === -1 && b.faceA !== 3) {
            below = true;
            break;
          }
          const at = this.graph.bondPosition(b);
          if (at.y < p.pose.position.y - 0.02) {
            below = true;
            break;
          }
        }
        if (below || n === 0) continue;
        const load = (p.mass * G * interval * SAG_RATE) / n;
        for (const b of this.graph.intact(id)) {
          b.damage += load;
          if (b.damage >= b.capacity) {
            b.broken = true;
            this.dirty = true;
          }
        }
      }
    }
  }

  private contactPoint(c1: RAPIER.Collider, c2: RAPIER.Collider): Vec3 {
    let n = 0,
      x = 0,
      y = 0,
      z = 0;
    this.world.contactPair(c1, c2, (m) => {
      for (let i = 0; i < m.numSolverContacts(); i++) {
        const p = m.solverContactPoint(i);
        if (!p) continue;
        x += p.x;
        y += p.y;
        z += p.z;
        n++;
      }
    });
    if (n) return { x: x / n, y: y / n, z: z / n };
    const a = c1.translation(),
      b = c2.translation();
    return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2, z: (a.z + b.z) / 2 };
  }

  /** Live pieces (for rendering). */
  livePieces(): Piece[] {
    return this.graph.pieces.filter((p) => p.alive);
  }

  get bodyCount(): number {
    return this.bodies.size;
  }

  get looseBodyCount(): number {
    let n = 0;
    for (const r of this.bodies.values()) if (!r.fixed) n++;
    return n;
  }
}

/** Perfectly inelastic: the impactor and the freed bodies leave together. */
export function shareMomentum(impactor: RAPIER.RigidBody, v0: Vec3, freed: RAPIER.RigidBody[]): void {
  const mi = impactor.mass();
  let m = mi;
  let px = mi * v0.x,
    py = mi * v0.y,
    pz = mi * v0.z;
  for (const rb of freed) {
    const v = rb.linvel();
    const mf = rb.mass();
    m += mf;
    px += mf * v.x;
    py += mf * v.y;
    pz += mf * v.z;
  }
  const v = { x: px / m, y: py / m, z: pz / m };
  if (impactor.isDynamic()) impactor.setLinvel(v, true);
  for (const rb of freed) rb.setLinvel(v, true);
}

const bodyMass = (c: RAPIER.Collider): number => {
  const rb = c.parent();
  if (!rb || !rb.isDynamic()) return Infinity;
  return rb.mass();
};

function projectileRadius(rb: RAPIER.RigidBody): number {
  const c = rb.collider(0);
  const s = c.shape as unknown as { radius?: number };
  return typeof s.radius === 'number' ? s.radius : 0.15;
}

const flat = (ps: readonly Vec3[]) => {
  const a = new Float32Array(ps.length * 3);
  ps.forEach((p, i) => {
    a[i * 3] = p.x;
    a[i * 3 + 1] = p.y;
    a[i * 3 + 2] = p.z;
  });
  return a;
};

function colliderFor(p: Piece): RAPIER.ColliderDesc {
  if (p.shape.kind === 'box')
    return RAPIER.ColliderDesc.cuboid(p.shape.half.x, p.shape.half.y, p.shape.half.z);
  // Slivers make hull construction fragile; use their bounding box instead.
  let lo = { x: Infinity, y: Infinity, z: Infinity },
    hi = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const q of p.shape.points) {
    lo = { x: Math.min(lo.x, q.x), y: Math.min(lo.y, q.y), z: Math.min(lo.z, q.z) };
    hi = { x: Math.max(hi.x, q.x), y: Math.max(hi.y, q.y), z: Math.max(hi.z, q.z) };
  }
  const ext = Math.min(hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
  if (ext < 0.004 || p.shape.points.length < 4)
    return RAPIER.ColliderDesc.cuboid(
      Math.max(0.002, (hi.x - lo.x) / 2),
      Math.max(0.002, (hi.y - lo.y) / 2),
      Math.max(0.002, (hi.z - lo.z) / 2),
    );
  const hull = RAPIER.ColliderDesc.convexHull(flat(p.shape.points));
  if (hull) return hull;
  const r = Math.max(0.005, pieceRadius(p) * 0.5);
  return RAPIER.ColliderDesc.ball(r);
}

/** Radius of a sphere around the piece centre that contains it. */
export function pieceRadius(p: Piece): number {
  if (p.shape.kind === 'box') return Math.hypot(p.shape.half.x, p.shape.half.y, p.shape.half.z);
  let r = 0;
  for (const q of p.shape.points) r = Math.max(r, Math.hypot(q.x, q.y, q.z));
  return r;
}

/** World-space bounds of a piece at its current pose. */
export function pieceBounds(p: Piece): { min: Vec3; max: Vec3 } {
  const pts =
    p.shape.kind === 'convex'
      ? p.shape.points
      : (() => {
          const h = p.shape.half;
          const out: Vec3[] = [];
          for (const sx of [-1, 1])
            for (const sy of [-1, 1])
              for (const sz of [-1, 1]) out.push({ x: sx * h.x, y: sy * h.y, z: sz * h.z });
          return out;
        })();
  let min = { x: Infinity, y: Infinity, z: Infinity };
  let max = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const q of pts) {
    const w = toWorld(p.pose, q);
    min = { x: Math.min(min.x, w.x), y: Math.min(min.y, w.y), z: Math.min(min.z, w.z) };
    max = { x: Math.max(max.x, w.x), y: Math.max(max.y, w.y), z: Math.max(max.z, w.z) };
  }
  return { min, max };
}

const quatMul = (a: Pose['quaternion'], b: Pose['quaternion']) => ({
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
});
