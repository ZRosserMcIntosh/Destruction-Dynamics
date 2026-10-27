// Builders for breakable structures: masonry walls (running or stack bond),
// framed panels, and a pre-scored sphere (the watermelon).

import { SeededRandom, type Vec3, quatIdentity } from '@destruction-dynamics/core';
import { type BondGraph, WORLD } from './bondGraph';
import type { FractureMaterial } from './materials';
import { voronoiBox } from './voronoi';

export interface WallOptions {
  material: FractureMaterial;
  /** Bottom centre of the wall. The wall runs along z, faces along x. */
  base: Vec3;
  length: number;
  height: number;
  /** Size of one unit including its joint. */
  unit: { length: number; height: number; thickness: number };
  /** Joint thickness (the visible mortar gap), m. */
  joint?: number;
  /** Running bond (courses offset by half a unit) or stack bond. */
  bond?: 'running' | 'stack';
  /** Also hold the ends and the top course (a panel in a frame). */
  framed?: boolean;
  /** Foundation strength relative to the material's joints. */
  foundation?: number;
  tag?: string;
}

/** Builds a wall of box units joined by mortar joints; returns the piece ids. */
export function buildWall(g: BondGraph, o: WallOptions): number[] {
  const j = o.joint ?? 0.01;
  const u = o.unit;
  const courses = Math.max(1, Math.round(o.height / u.height));
  const ids: number[] = [];
  const rows: { id: number; z0: number; z1: number }[][] = [];
  const hy = (u.height - j) / 2;
  const hx = (u.thickness - j) / 2;
  for (let k = 0; k < courses; k++) {
    const offset = (o.bond ?? 'running') === 'running' && k % 2 === 1 ? u.length / 2 : 0;
    const cuts = [-o.length / 2];
    for (let z = -o.length / 2 + (offset || u.length); z < o.length / 2 - 0.04; z += u.length) cuts.push(z);
    cuts.push(o.length / 2);
    const row: { id: number; z0: number; z1: number }[] = [];
    for (let i = 0; i + 1 < cuts.length; i++) {
      const z0 = cuts[i]!,
        z1 = cuts[i + 1]!;
      const id = g.addPiece({
        material: o.material,
        shape: { kind: 'box', half: { x: hx, y: hy, z: (z1 - z0 - j) / 2 } },
        pose: {
          position: { x: o.base.x, y: o.base.y + (k + 0.5) * u.height, z: o.base.z + (z0 + z1) / 2 },
          quaternion: quatIdentity(),
        },
        tag: o.tag ?? 'wall',
      });
      row.push({ id, z0, z1 });
      ids.push(id);
    }
    // Head joints along the course.
    for (let i = 0; i + 1 < row.length; i++) {
      const a = row[i]!;
      g.addBond(
        a.id,
        row[i + 1]!.id,
        2 * hy * 2 * hx,
        { x: o.base.x, y: o.base.y + (k + 0.5) * u.height, z: o.base.z + a.z1 },
        5,
        4,
      );
    }
    // Bed joints to the course below.
    const below = rows[k - 1];
    if (below) {
      for (const a of below)
        for (const b of row) {
          const lo = Math.max(a.z0, b.z0),
            hi = Math.min(a.z1, b.z1);
          if (hi - lo < 0.01) continue;
          g.addBond(
            a.id,
            b.id,
            (hi - lo) * 2 * hx,
            { x: o.base.x, y: o.base.y + k * u.height, z: o.base.z + (lo + hi) / 2 },
            3,
            2,
          );
        }
    } else {
      for (const b of row)
        g.addBond(
          b.id,
          WORLD,
          (b.z1 - b.z0) * 2 * hx,
          { x: o.base.x, y: o.base.y, z: o.base.z + (b.z0 + b.z1) / 2 },
          2,
          -1,
          o.foundation ?? 1.5,
        );
    }
    if (o.framed) {
      const y = o.base.y + (k + 0.5) * u.height;
      const first = row[0]!,
        last = row[row.length - 1]!;
      g.addBond(first.id, WORLD, 2 * hy * 2 * hx, { x: o.base.x, y, z: o.base.z + first.z0 }, 4, -1, 1.5);
      g.addBond(last.id, WORLD, 2 * hy * 2 * hx, { x: o.base.x, y, z: o.base.z + last.z1 }, 5, -1, 1.5);
      if (k === courses - 1)
        for (const b of row)
          g.addBond(
            b.id,
            WORLD,
            (b.z1 - b.z0) * 2 * hx,
            { x: o.base.x, y: o.base.y + o.height, z: o.base.z + (b.z0 + b.z1) / 2 },
            3,
            -1,
            1.5,
          );
    }
    rows.push(row);
  }
  return ids;
}

export interface PanelOptions {
  material: FractureMaterial;
  /** Centre of the panel. It faces along x and spans y (height) and z (width). */
  centre: Vec3;
  width: number;
  height: number;
  thickness: number;
  /** Held on all four edges (true) or standing free on its bottom edge. */
  framed?: boolean;
  tag?: string;
}

/** One plate held by a frame along its edges. */
export function buildPanel(g: BondGraph, o: PanelOptions): number {
  const h = { x: o.thickness / 2, y: o.height / 2, z: o.width / 2 };
  const id = g.addPiece({
    material: o.material,
    shape: { kind: 'box', half: h },
    pose: { position: o.centre, quaternion: quatIdentity() },
    tag: o.tag ?? 'panel',
  });
  const c = o.centre;
  g.addBond(id, WORLD, o.width * o.thickness, { x: c.x, y: c.y - h.y, z: c.z }, 2, -1, 2);
  if (o.framed ?? true) {
    g.addBond(id, WORLD, o.width * o.thickness, { x: c.x, y: c.y + h.y, z: c.z }, 3, -1, 2);
    g.addBond(id, WORLD, o.height * o.thickness, { x: c.x, y: c.y, z: c.z - h.z }, 4, -1, 2);
    g.addBond(id, WORLD, o.height * o.thickness, { x: c.x, y: c.y, z: c.z + h.z }, 5, -1, 2);
  }
  return id;
}

export interface SphereOptions {
  material: FractureMaterial;
  centre: Vec3;
  radius: number;
  /** Number of pre-scored pieces. */
  pieces?: number;
  seed?: number;
  tag?: string;
}

/**
 * A sphere pre-scored into convex pieces joined by weak joints (it bursts
 * rather than shattering on demand). Cube Voronoi cells are pushed out onto
 * the sphere radially.
 */
export function buildScoredSphere(g: BondGraph, o: SphereOptions): number[] {
  const r = o.radius;
  const rng = new SeededRandom(o.seed ?? 7);
  const seeds: Vec3[] = [];
  for (let i = 0; i < (o.pieces ?? 10); i++)
    seeds.push({ x: rng.range(-0.8, 0.8) * r, y: rng.range(-0.8, 0.8) * r, z: rng.range(-0.8, 0.8) * r });
  const cells = voronoiBox({ x: r, y: r, z: r }, seeds);
  const toSphere = (p: Vec3): Vec3 => {
    const linf = Math.max(Math.abs(p.x), Math.abs(p.y), Math.abs(p.z));
    const l2 = Math.hypot(p.x, p.y, p.z);
    const k = l2 > 1e-9 ? linf / l2 : 1;
    return { x: p.x * k, y: p.y * k, z: p.z * k };
  };
  const volumeScale = ((4 / 3) * Math.PI) / 8;
  const ids = new Map<number, number>();
  cells.forEach((c, i) => {
    if (c.volume <= 1e-10 || c.points.length < 4) return;
    const pts = c.points.map(toSphere);
    const centre = toSphere(c.centroid);
    const id = g.addPiece(
      {
        material: o.material,
        shape: {
          kind: 'convex',
          points: pts.map((p) => ({ x: p.x - centre.x, y: p.y - centre.y, z: p.z - centre.z })),
        },
        pose: {
          position: { x: o.centre.x + centre.x, y: o.centre.y + centre.y, z: o.centre.z + centre.z },
          quaternion: quatIdentity(),
        },
        tag: o.tag ?? 'sphere',
      },
      c.volume * volumeScale,
      true,
    );
    ids.set(i, id);
  });
  cells.forEach((c, i) => {
    const a = ids.get(i);
    if (a === undefined) return;
    for (const [jn, patch] of c.neighbours) {
      const b = ids.get(jn);
      if (b === undefined || jn < i) continue;
      const p = toSphere(patch.centroid);
      g.addBond(a, b, patch.area * volumeScale, {
        x: o.centre.x + p.x,
        y: o.centre.y + p.y,
        z: o.centre.z + p.z,
      });
    }
  });
  return [...ids.values()];
}
