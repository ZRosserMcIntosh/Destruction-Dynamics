// The support graph behind breakable structures: pieces (boxes, or convex
// shards made at run time), the joints between them and the joints to the
// world (foundations, frames). Impacts load the joints around the point hit;
// joints that run out of capacity break, and a box hit hard enough shatters
// into Voronoi cells around the impact point. Islands of pieces still joined
// to the world stay fixed; the rest become rigid bodies (FractureWorld).
//
// The model is in the spirit of NVIDIA Blast's support graph and impulse
// damage. It knows nothing about the physics backend.

import {
  type Pose,
  type Quat,
  SeededRandom,
  type Vec3,
  quatRotate,
  smoothstep,
} from '@destruction-dynamics/core';
import type { FractureMaterial } from './materials';
import { type BoxFace, voronoiBox } from './voronoi';

/** The other end of a joint to the ground or a frame. */
export const WORLD = -1;

export type PieceShape = { kind: 'box'; half: Vec3 } | { kind: 'convex'; points: Vec3[] };

export interface Piece {
  readonly id: number;
  readonly material: FractureMaterial;
  readonly shape: PieceShape;
  /** World pose of the piece's centre. FractureWorld keeps it current. */
  pose: Pose;
  readonly volume: number;
  readonly mass: number;
  /** False once shattered into cells. */
  alive: boolean;
  /** Cells never shatter again. */
  readonly shard: boolean;
  /** Joints this piece takes part in (indices into BondGraph.bonds). */
  readonly bonds: number[];
  /** Free-form tag set by builders (e.g. 'panel', 'melon'). */
  readonly tag: string;
}

export interface Bond {
  readonly a: number;
  /** A piece id, or WORLD. */
  readonly b: number;
  /** Box face of a / b the joint sits on, or −1 for a shard. */
  readonly faceA: BoxFace | -1;
  readonly faceB: BoxFace | -1;
  readonly area: number;
  /** Joint centre in piece a's frame. */
  readonly localA: Vec3;
  /** Impulse it can carry before failing (N·s). */
  capacity: number;
  damage: number;
  broken: boolean;
}

export interface PieceInit {
  material: FractureMaterial;
  shape: PieceShape;
  pose: Pose;
  tag?: string;
}

export interface ImpactResult {
  /** Joints that broke. */
  broken: number;
  /** Pieces that shattered (now dead) and the cells that replaced them. */
  shattered: number[];
  created: number[];
}

export interface Island {
  pieces: number[];
  anchored: boolean;
}

const conj = (q: Quat): Quat => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });

export const toPieceLocal = (pose: Pose, p: Vec3): Vec3 =>
  quatRotate(conj(pose.quaternion), {
    x: p.x - pose.position.x,
    y: p.y - pose.position.y,
    z: p.z - pose.position.z,
  });

export const toWorld = (pose: Pose, p: Vec3): Vec3 => {
  const r = quatRotate(pose.quaternion, p);
  return { x: r.x + pose.position.x, y: r.y + pose.position.y, z: r.z + pose.position.z };
};

const grow = (a: Uint32Array, n: number) => {
  const b = new Uint32Array(Math.max(n, a.length * 2, 64));
  b.set(a);
  return b;
};

const boxFaceArea = (h: Vec3, f: BoxFace) => (f < 2 ? 4 * h.y * h.z : f < 4 ? 4 * h.x * h.z : 4 * h.x * h.y);

/** Point on a box face (local frame), at the face centre. */
export const boxFaceCentre = (h: Vec3, f: BoxFace): Vec3 => {
  const s = f % 2 === 0 ? -1 : 1;
  return f < 2 ? { x: s * h.x, y: 0, z: 0 } : f < 4 ? { x: 0, y: s * h.y, z: 0 } : { x: 0, y: 0, z: s * h.z };
};

function shapeVolume(s: PieceShape): number {
  if (s.kind === 'box') return 8 * s.half.x * s.half.y * s.half.z;
  // Bounding-box estimate; convex pieces made by shatter() pass their exact volume.
  let lo = { x: Infinity, y: Infinity, z: Infinity },
    hi = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const p of s.points) {
    lo = { x: Math.min(lo.x, p.x), y: Math.min(lo.y, p.y), z: Math.min(lo.z, p.z) };
    hi = { x: Math.max(hi.x, p.x), y: Math.max(hi.y, p.y), z: Math.max(hi.z, p.z) };
  }
  return 0.5 * (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z);
}

export class BondGraph {
  readonly pieces: Piece[] = [];
  readonly bonds: Bond[] = [];
  private readonly rng: SeededRandom;
  private epoch = 0;
  private pieceStamp = new Uint32Array(0);
  private bondStamp = new Uint32Array(0);
  private readonly queue: number[] = [];
  private readonly near: Bond[] = [];
  private readonly weights: number[] = [];

  constructor(seed = 1) {
    this.rng = new SeededRandom(seed);
  }

  addPiece(init: PieceInit, volume = shapeVolume(init.shape), shard = false): number {
    const id = this.pieces.length;
    this.pieces.push({
      id,
      material: init.material,
      shape: init.shape,
      pose: { position: { ...init.pose.position }, quaternion: { ...init.pose.quaternion } },
      volume,
      mass: volume * init.material.density,
      alive: true,
      shard,
      bonds: [],
      tag: init.tag ?? '',
    });
    return id;
  }

  /**
   * Joins two pieces (or a piece and the world) across a joint of the given
   * area. `at` is the joint centre in world space. Strength comes from the
   * weaker material.
   */
  addBond(
    a: number,
    b: number,
    area: number,
    at: Vec3,
    faceA: BoxFace | -1 = -1,
    faceB: BoxFace | -1 = -1,
    strengthScale = 1,
  ): number {
    const pa = this.pieces[a]!;
    const sa = pa.material.bondStrength;
    const sb = b === WORLD ? sa : this.pieces[b]!.material.bondStrength;
    const id = this.bonds.length;
    this.bonds.push({
      a,
      b,
      faceA,
      faceB,
      area,
      localA: toPieceLocal(pa.pose, at),
      capacity: Math.min(sa, sb) * area * strengthScale,
      damage: 0,
      broken: false,
    });
    pa.bonds.push(id);
    if (b !== WORLD) this.pieces[b]!.bonds.push(id);
    return id;
  }

  bondPosition(bond: Bond): Vec3 {
    return toWorld(this.pieces[bond.a]!.pose, bond.localA);
  }

  /** The intact joints of a piece. */
  *intact(piece: number): Generator<Bond> {
    for (const i of this.pieces[piece]!.bonds) {
      const b = this.bonds[i]!;
      if (!b.broken) yield b;
    }
  }

  /** Impulse that shatters this piece in one hit (Infinity for shards). */
  shatterImpulse(piece: number): number {
    const p = this.pieces[piece]!;
    if (p.shard || p.shape.kind !== 'box') return Infinity;
    const h = p.shape.half;
    const t = 2 * Math.min(h.x, h.y, h.z);
    return p.material.toughness * t * t;
  }

  /**
   * Loads the structure with an impulse J (N·s) delivered to `piece` at the
   * world point `at` in one step (FractureWorld.STEP). The struck box shatters
   * if J exceeds its shatter impulse; the impulse is then shared among nearby joints of the same
   * island with Gaussian weights over the material's damage radius, and
   * accumulates as damage. A smaller `radius` keeps the load on the struck
   * piece's own joints (a wide impactor strikes every piece it touches).
   */
  applyImpact(piece: number, at: Vec3, impulse: number, radius?: number): ImpactResult {
    const result: ImpactResult = { broken: 0, shattered: [], created: [] };
    let p = this.pieces[piece];
    if (!p || !p.alive || !(impulse > 0)) return result;
    const crit = this.shatterImpulse(piece);
    const overload = impulse / crit;
    if (impulse > crit) {
      const cells = this.shatter(piece, at, impulse / crit);
      result.shattered.push(piece);
      result.created.push(...cells);
      // Continue with the cell that contains the impact point.
      let best = cells[0]!;
      let bd = Infinity;
      for (const c of cells) {
        const q = this.pieces[c]!.pose.position;
        const d = (q.x - at.x) ** 2 + (q.y - at.y) ** 2 + (q.z - at.z) ** 2;
        if (d < bd) {
          bd = d;
          best = c;
        }
      }
      p = this.pieces[best]!;
    }
    const R = radius ?? p.material.damageRadius;
    // A hit well past the shatter impulse also tears out the joints around it.
    const punch2 = overload > 1 ? (R * Math.min(2, 0.35 * Math.sqrt(overload))) ** 2 : 0;
    const reach2 = (2.5 * R) ** 2;
    // Breadth-first over intact joints, staying within reach of the impact.
    // Stamps instead of sets: this runs for every contact event.
    const stamp = ++this.epoch;
    if (this.pieceStamp.length < this.pieces.length)
      this.pieceStamp = grow(this.pieceStamp, this.pieces.length);
    if (this.bondStamp.length < this.bonds.length) this.bondStamp = grow(this.bondStamp, this.bonds.length);
    const queue = this.queue;
    let head = 0,
      tail = 0;
    queue[tail++] = p.id;
    this.pieceStamp[p.id] = stamp;
    // After a shatter, start from every cell: the struck one may have come
    // away with no joints left.
    for (const c of result.created) {
      if (this.pieceStamp[c] === stamp) continue;
      this.pieceStamp[c] = stamp;
      queue[tail++] = c;
    }
    const near = this.near;
    near.length = 0;
    const weights = this.weights;
    weights.length = 0;
    while (head < tail) {
      const id = queue[head++]!;
      const piece = this.pieces[id]!;
      const pose = piece.pose;
      for (const bi of piece.bonds) {
        if (this.bondStamp[bi] === stamp) continue;
        const b = this.bonds[bi]!;
        if (b.broken) continue;
        // Joint centre: rotate from whichever end we are standing on is not
        // possible (localA is in a's frame), so use a's pose.
        const pa = b.a === id ? pose : this.pieces[b.a]!.pose;
        const q = pa.quaternion;
        const l = b.localA;
        // quatRotate inlined.
        const tx = 2 * (q.y * l.z - q.z * l.y),
          ty = 2 * (q.z * l.x - q.x * l.z),
          tz = 2 * (q.x * l.y - q.y * l.x);
        const px = pa.position.x + l.x + q.w * tx + (q.y * tz - q.z * ty) - at.x;
        const py = pa.position.y + l.y + q.w * ty + (q.z * tx - q.x * tz) - at.y;
        const pz = pa.position.z + l.z + q.w * tz + (q.x * ty - q.y * tx) - at.z;
        const d2 = px * px + py * py + pz * pz;
        if (d2 > reach2) continue;
        this.bondStamp[bi] = stamp;
        if (d2 < punch2 && Number.isFinite(b.capacity)) {
          b.broken = true;
          result.broken++;
          continue;
        }
        near.push(b);
        weights.push(Math.exp(-d2 / (R * R)));
        const other = b.a === id ? b.b : b.a;
        if (other !== WORLD && this.pieceStamp[other] !== stamp) {
          this.pieceStamp[other] = stamp;
          queue[tail++] = other;
        }
      }
    }
    let sum = 0;
    for (const w of weights) sum += w;
    if (sum <= 0) return result;
    // Far-away joints (sum < 1) only take their own share, so a hit in the
    // middle of a pane does not pull it out of its frame.
    sum = Math.max(sum, 1);
    for (let i = 0; i < near.length; i++) {
      const bond = near[i]!;
      if (!Number.isFinite(bond.capacity)) continue;
      bond.damage += (impulse * weights[i]!) / sum;
      if (bond.damage >= bond.capacity) {
        bond.broken = true;
        result.broken++;
      }
    }
    return result;
  }

  /**
   * Replaces a box with Voronoi cells seeded around the impact point. Plates
   * get a radial pattern in their mid-plane (spider-web cracks); chunky boxes
   * get seeds clustered towards the impact. Cells are joined to each other and
   * inherit the box's joints across the faces they touch, except within the
   * punch radius, which grows with how far the hit exceeded the shatter
   * impulse (`overload` ≥ 1): there the cells come away.
   */
  shatter(piece: number, at: Vec3, overload = 1): number[] {
    const p = this.pieces[piece]!;
    if (p.shape.kind !== 'box') return [piece];
    const h = p.shape.half;
    const mat = p.material;
    const local = toPieceLocal(p.pose, at);
    const n = Math.max(2, mat.shards);
    const seeds: Vec3[] = [];
    const dims = [h.x, h.y, h.z];
    const thin = dims.indexOf(Math.min(...dims));
    const sorted = [...dims].sort((a, b) => a - b);
    const plate = sorted[0]! < 0.25 * sorted[1]!;
    const extent = Math.max(...dims) * 2;
    for (let i = 0; i < n; i++) {
      if (plate) {
        const r = extent * 0.9 * ((i + 0.35 * this.rng.next()) / n) ** 1.7;
        const ang = i * 2.399963 + this.rng.range(-0.3, 0.3);
        const u = r * Math.cos(ang),
          v = r * Math.sin(ang);
        seeds.push(
          thin === 0
            ? { x: 0, y: local.y + u, z: local.z + v }
            : thin === 1
              ? { x: local.x + u, y: 0, z: local.z + v }
              : { x: local.x + u, y: local.y + v, z: 0 },
        );
      } else {
        const r = Math.max(...dims) * this.rng.next() ** 1.5;
        const th = this.rng.range(0, 2 * Math.PI);
        const ph = Math.acos(this.rng.range(-1, 1));
        seeds.push({
          x: local.x + r * Math.sin(ph) * Math.cos(th),
          y: local.y + r * Math.sin(ph) * Math.sin(th),
          z: local.z + r * Math.cos(ph),
        });
      }
    }
    const all = voronoiBox(h, seeds);
    if (all.filter((c) => c.volume > 1e-10 && c.points.length >= 4).length < 2) return [piece];
    p.alive = false;
    const ids: number[] = [];
    const index = new Map<number, number>(); // cell index → piece id
    for (let ci = 0; ci < all.length; ci++) {
      const c = all[ci]!;
      if (c.volume <= 1e-10 || c.points.length < 4) continue;
      const pose: Pose = {
        position: toWorld(p.pose, c.centroid),
        quaternion: { ...p.pose.quaternion },
      };
      const id = this.addPiece(
        {
          material: mat,
          shape: {
            kind: 'convex',
            points: c.points.map((q) => ({
              x: q.x - c.centroid.x,
              y: q.y - c.centroid.y,
              z: q.z - c.centroid.z,
            })),
          },
          pose,
          tag: p.tag,
        },
        c.volume,
        true,
      );
      index.set(ci, id);
      ids.push(id);
    }
    // Joints between cells: none inside the punch radius, weaker close to it.
    const R = mat.damageRadius;
    const punch = Math.min(0.45 * extent, R * Math.sqrt(overload));
    const outside = (q: Vec3) => Math.hypot(q.x - local.x, q.y - local.y, q.z - local.z) > punch;
    for (let ci = 0; ci < all.length; ci++) {
      const a = index.get(ci);
      if (a === undefined) continue;
      for (const [cj, patch] of all[ci]!.neighbours) {
        const b = index.get(cj);
        if (b === undefined || cj < ci) continue;
        const d = Math.hypot(
          patch.centroid.x - local.x,
          patch.centroid.y - local.y,
          patch.centroid.z - local.z,
        );
        if (d <= punch) continue;
        this.addBond(
          a,
          b,
          patch.area,
          toWorld(p.pose, patch.centroid),
          -1,
          -1,
          0.3 + 0.7 * smoothstep(0, 2 * R, d),
        );
      }
    }
    // Inherit the box's joints across the faces each cell touches.
    for (const bi of [...p.bonds]) {
      const old = this.bonds[bi]!;
      if (old.broken) continue;
      const mine = old.a === piece;
      const face = mine ? old.faceA : old.faceB;
      const other = mine ? old.b : old.a;
      const otherFace = mine ? old.faceB : old.faceA;
      old.broken = true;
      if (face === -1) continue;
      const faceArea = boxFaceArea(h, face);
      for (let ci = 0; ci < all.length; ci++) {
        const id = index.get(ci);
        const patch = all[ci]!.boxFaces.get(face);
        if (id === undefined || !patch || !outside(patch.centroid)) continue;
        const area = (old.area * patch.area) / faceArea;
        if (area < 1e-7) continue;
        const nb = this.addBond(id, other, area, toWorld(p.pose, patch.centroid), -1, otherFace);
        const bond = this.bonds[nb]!;
        // Carry over the damage already done, in proportion.
        bond.damage = (old.damage * area) / old.area;
        // Keep the original per-area strength.
        bond.capacity = (old.capacity * area) / old.area;
      }
    }
    return ids;
  }

  /** Connected groups of live pieces over intact joints, and whether each is held by the world. */
  islands(): Island[] {
    const parent = new Map<number, number>();
    const find = (x: number): number => {
      let r = x;
      while (parent.get(r) !== r) r = parent.get(r)!;
      let c = x;
      while (c !== r) {
        const next = parent.get(c)!;
        parent.set(c, r);
        c = next;
      }
      return r;
    };
    for (const p of this.pieces) if (p.alive) parent.set(p.id, p.id);
    const anchoredPieces = new Set<number>();
    for (const b of this.bonds) {
      if (b.broken) continue;
      if (b.b === WORLD) {
        anchoredPieces.add(b.a);
        continue;
      }
      if (!parent.has(b.a) || !parent.has(b.b)) continue;
      const ra = find(b.a),
        rb = find(b.b);
      if (ra !== rb) parent.set(ra, rb);
    }
    const groups = new Map<number, Island>();
    for (const p of this.pieces) {
      if (!p.alive) continue;
      const r = find(p.id);
      let g = groups.get(r);
      if (!g) {
        g = { pieces: [], anchored: false };
        groups.set(r, g);
      }
      g.pieces.push(p.id);
      if (anchoredPieces.has(p.id)) g.anchored = true;
    }
    return [...groups.values()];
  }
}
