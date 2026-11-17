// Splits a box into convex Voronoi cells by plane intersection (no mesh
// clipping, no dependencies). Each cell is the set of points of the box closer
// to its seed than to any other seed: the intersection of the six box
// half-spaces with one bisector half-space per other seed. Cell vertices are
// the triple-plane intersections that satisfy every half-space.

import type { Vec3 } from '@destruction-dynamics/core';

/** A box face: 0 −x, 1 +x, 2 −y, 3 +y, 4 −z, 5 +z. */
export type BoxFace = 0 | 1 | 2 | 3 | 4 | 5;

export interface FacePatch {
  area: number;
  centroid: Vec3;
}

export interface ConvexCell {
  seed: Vec3;
  /** Hull vertices in the box's local frame. */
  points: Vec3[];
  centroid: Vec3;
  volume: number;
  /** Neighbouring cell index → shared face. */
  neighbours: Map<number, FacePatch>;
  /** Box face → the part of it this cell covers. */
  boxFaces: Map<BoxFace, FacePatch>;
}

interface Plane {
  n: Vec3;
  d: number;
  /** ≥ 0: neighbouring cell index; < 0: box face −(1 + face). */
  tag: number;
}

const EPS = 1e-7;

function boxPlanes(h: Vec3): Plane[] {
  return [
    { n: { x: -1, y: 0, z: 0 }, d: h.x, tag: -1 },
    { n: { x: 1, y: 0, z: 0 }, d: h.x, tag: -2 },
    { n: { x: 0, y: -1, z: 0 }, d: h.y, tag: -3 },
    { n: { x: 0, y: 1, z: 0 }, d: h.y, tag: -4 },
    { n: { x: 0, y: 0, z: -1 }, d: h.z, tag: -5 },
    { n: { x: 0, y: 0, z: 1 }, d: h.z, tag: -6 },
  ];
}

/** Intersection of three planes, or null when they are (nearly) parallel. */
function intersect3(a: Plane, b: Plane, c: Plane): Vec3 | null {
  const bc = cross(b.n, c.n);
  const det = dot3(a.n, bc);
  if (Math.abs(det) < 1e-9) return null;
  const ca = cross(c.n, a.n);
  const ab = cross(a.n, b.n);
  return {
    x: (a.d * bc.x + b.d * ca.x + c.d * ab.x) / det,
    y: (a.d * bc.y + b.d * ca.y + c.d * ab.y) / det,
    z: (a.d * bc.z + b.d * ca.z + c.d * ab.z) / det,
  };
}

const cross = (a: Vec3, b: Vec3): Vec3 => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x,
});
const dot3 = (a: Vec3, b: Vec3) => a.x * b.x + a.y * b.y + a.z * b.z;

/** Area and centroid of a planar convex polygon given its (unordered) vertices. */
function polygon(points: Vec3[], n: Vec3): FacePatch & { order: Vec3[] } {
  const c = average(points);
  // Order around the centroid in the plane.
  const u = Math.abs(n.x) < 0.9 ? cross(n, { x: 1, y: 0, z: 0 }) : cross(n, { x: 0, y: 1, z: 0 });
  const ul = Math.hypot(u.x, u.y, u.z);
  const ux = { x: u.x / ul, y: u.y / ul, z: u.z / ul };
  const vy = cross(n, ux);
  const sorted = points
    .map((p) => {
      const r = { x: p.x - c.x, y: p.y - c.y, z: p.z - c.z };
      return { p, a: Math.atan2(dot3(r, vy), dot3(r, ux)) };
    })
    .sort((a, b) => a.a - b.a)
    .map((e) => e.p);
  let area = 0;
  let cx = 0,
    cy = 0,
    cz = 0;
  for (let i = 0; i < sorted.length; i++) {
    const a = sorted[i]!;
    const b = sorted[(i + 1) % sorted.length]!;
    const t = cross(
      { x: a.x - c.x, y: a.y - c.y, z: a.z - c.z },
      { x: b.x - c.x, y: b.y - c.y, z: b.z - c.z },
    );
    const ta = 0.5 * Math.abs(dot3(t, n));
    area += ta;
    cx += (ta * (a.x + b.x + c.x)) / 3;
    cy += (ta * (a.y + b.y + c.y)) / 3;
    cz += (ta * (a.z + b.z + c.z)) / 3;
  }
  return { area, centroid: area > 0 ? { x: cx / area, y: cy / area, z: cz / area } : c, order: sorted };
}

const average = (ps: Vec3[]): Vec3 => {
  let x = 0,
    y = 0,
    z = 0;
  for (const p of ps) {
    x += p.x;
    y += p.y;
    z += p.z;
  }
  return { x: x / ps.length, y: y / ps.length, z: z / ps.length };
};

/**
 * Splits the box [−half, +half] into one convex cell per seed. Seeds outside
 * the box are clamped into it; coincident seeds are nudged apart.
 */
export function voronoiBox(half: Vec3, seedsIn: readonly Vec3[]): ConvexCell[] {
  const seeds = seedsIn.map((s, i) => ({
    x: Math.max(-half.x * 0.999, Math.min(half.x * 0.999, s.x)) + i * 1e-9,
    y: Math.max(-half.y * 0.999, Math.min(half.y * 0.999, s.y)),
    z: Math.max(-half.z * 0.999, Math.min(half.z * 0.999, s.z)),
  }));
  const scaleEps = EPS * Math.max(half.x, half.y, half.z, 1);
  const cells: ConvexCell[] = [];
  for (let i = 0; i < seeds.length; i++) {
    const si = seeds[i]!;
    const planes = boxPlanes(half);
    for (let j = 0; j < seeds.length; j++) {
      if (j === i) continue;
      const sj = seeds[j]!;
      const dx = sj.x - si.x,
        dy = sj.y - si.y,
        dz = sj.z - si.z;
      const l = Math.hypot(dx, dy, dz);
      if (l < 1e-12) continue;
      const n = { x: dx / l, y: dy / l, z: dz / l };
      planes.push({
        n,
        d: dot3(n, { x: (si.x + sj.x) / 2, y: (si.y + sj.y) / 2, z: (si.z + sj.z) / 2 }),
        tag: j,
      });
    }
    const inside = (p: Vec3) => planes.every((pl) => dot3(pl.n, p) <= pl.d + scaleEps);
    const verts: Vec3[] = [];
    const P = planes.length;
    for (let a = 0; a < P; a++)
      for (let b = a + 1; b < P; b++)
        for (let c = b + 1; c < P; c++) {
          const p = intersect3(planes[a]!, planes[b]!, planes[c]!);
          if (!p || !inside(p)) continue;
          if (
            verts.some((q) => Math.abs(q.x - p.x) + Math.abs(q.y - p.y) + Math.abs(q.z - p.z) < scaleEps * 10)
          )
            continue;
          verts.push(p);
        }
    const neighbours = new Map<number, FacePatch>();
    const boxFaces = new Map<BoxFace, FacePatch>();
    let volume = 0;
    let vx = 0,
      vy = 0,
      vz = 0;
    const inner = average(verts);
    for (const pl of planes) {
      const on = verts.filter((p) => Math.abs(dot3(pl.n, p) - pl.d) < scaleEps * 10);
      if (on.length < 3) continue;
      const patch = polygon(on, pl.n);
      if (patch.area < 1e-12) continue;
      if (pl.tag >= 0) neighbours.set(pl.tag, { area: patch.area, centroid: patch.centroid });
      else boxFaces.set((-pl.tag - 1) as BoxFace, { area: patch.area, centroid: patch.centroid });
      // Volume and centroid by tetrahedra from the inner point over the face fan.
      const o = patch.order;
      for (let k = 1; k + 1 < o.length; k++) {
        const a = o[0]!,
          b = o[k]!,
          c = o[k + 1]!;
        const v =
          Math.abs(
            dot3(
              { x: a.x - inner.x, y: a.y - inner.y, z: a.z - inner.z },
              cross(
                { x: b.x - inner.x, y: b.y - inner.y, z: b.z - inner.z },
                { x: c.x - inner.x, y: c.y - inner.y, z: c.z - inner.z },
              ),
            ),
          ) / 6;
        volume += v;
        vx += (v * (a.x + b.x + c.x + inner.x)) / 4;
        vy += (v * (a.y + b.y + c.y + inner.y)) / 4;
        vz += (v * (a.z + b.z + c.z + inner.z)) / 4;
      }
    }
    cells.push({
      seed: si,
      points: verts,
      centroid: volume > 0 ? { x: vx / volume, y: vy / volume, z: vz / volume } : inner,
      volume,
      neighbours,
      boxFaces,
    });
  }
  return cells;
}
