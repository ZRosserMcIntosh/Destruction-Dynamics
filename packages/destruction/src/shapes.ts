// Convex shapes for static, indestructible obstacles. A shape is a set of
// outward planes (a point is inside when it is behind all of them) plus its
// vertices and edges, which contact sampling walks along.

import { type Quat, type Vec3, add, quatRotate, vec3 } from '@destruction-dynamics/core';

export interface ConvexShape {
  /** Outward planes in the shape's frame: inside when n·p < d for every plane. */
  planes: { n: Vec3; d: number }[];
  vertices: Vec3[];
  edges: [number, number][];
}

/** An immovable convex obstacle placed in the scene frame. */
export interface StaticObstacle {
  position: Vec3;
  quaternion: Quat;
  shape: ConvexShape;
}

export function boxShape(he: Vec3): ConvexShape {
  const vertices: Vec3[] = [];
  for (const x of [-1, 1])
    for (const y of [-1, 1]) for (const z of [-1, 1]) vertices.push(vec3(x * he.x, y * he.y, z * he.z));
  const edges: [number, number][] = [];
  for (let i = 0; i < 8; i++)
    for (let j = i + 1; j < 8; j++) if ([1, 2, 4].includes(i ^ j)) edges.push([i, j]);
  return {
    planes: [
      { n: vec3(1, 0, 0), d: he.x },
      { n: vec3(-1, 0, 0), d: he.x },
      { n: vec3(0, 1, 0), d: he.y },
      { n: vec3(0, -1, 0), d: he.y },
      { n: vec3(0, 0, 1), d: he.z },
      { n: vec3(0, 0, -1), d: he.z },
    ],
    vertices,
    edges,
  };
}

export interface WedgeOptions {
  /** Distance from the apex to the rind, m. */
  radius: number;
  /** Opening angle of the wedge, radians. */
  angle: number;
  /** Thickness of the wheel, m. */
  height: number;
  /** Flat facets approximating the curved rind (convex either way). */
  rindSegments?: number;
}

/**
 * A wedge cut from a wheel: apex at the origin, opening toward +x, from y = 0
 * up to `height`. The rind is approximated by flat facets.
 */
export function wedgeShape(o: WedgeOptions): ConvexShape {
  const segs = Math.max(1, o.rindSegments ?? 6);
  const ring: Vec3[] = [vec3(0, 0, 0)];
  for (let i = 0; i <= segs; i++) {
    const a = -o.angle / 2 + (o.angle * i) / segs;
    ring.push(vec3(o.radius * Math.cos(a), 0, o.radius * Math.sin(a)));
  }
  const m = ring.length;
  const vertices = [...ring, ...ring.map((p) => vec3(p.x, o.height, p.z))];
  const edges: [number, number][] = [];
  for (let i = 0; i < m; i++) {
    edges.push([i, (i + 1) % m], [m + i, m + ((i + 1) % m)], [i, m + i]);
  }
  // Side planes from consecutive ring points (counter-clockwise seen from +y).
  const planes: ConvexShape['planes'] = [
    { n: vec3(0, 1, 0), d: o.height },
    { n: vec3(0, -1, 0), d: 0 },
  ];
  for (let i = 0; i < m; i++) {
    const a = ring[i]!;
    const b = ring[(i + 1) % m]!;
    // Outward normal of edge a→b in the x/z plane.
    let nx = b.z - a.z,
      nz = -(b.x - a.x);
    const l = Math.hypot(nx, nz);
    nx /= l;
    nz /= l;
    // Make it point away from the centroid of the ring.
    const cx = ring.reduce((s, p) => s + p.x, 0) / m;
    const cz = ring.reduce((s, p) => s + p.z, 0) / m;
    if (nx * (a.x - cx) + nz * (a.z - cz) < 0) {
      nx = -nx;
      nz = -nz;
    }
    planes.push({ n: vec3(nx, 0, nz), d: nx * a.x + nz * a.z });
  }
  return { planes, vertices, edges };
}

const inv = (q: Quat): Quat => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });

export function toObstacleLocal(o: StaticObstacle, p: Vec3): Vec3 {
  return quatRotate(inv(o.quaternion), vec3(p.x - o.position.x, p.y - o.position.y, p.z - o.position.z));
}

/** Deepest-plane penetration of a local point: depth > 0 inside, with the plane to push out through. */
export function shapePenetration(s: ConvexShape, l: Vec3): { depth: number; plane: number } {
  let best = -Infinity,
    plane = 0;
  for (let k = 0; k < s.planes.length; k++) {
    const pl = s.planes[k]!;
    const d = pl.n.x * l.x + pl.n.y * l.y + pl.n.z * l.z - pl.d;
    if (d > best) {
      best = d;
      plane = k;
    }
  }
  return { depth: -best, plane };
}

export function pointInObstacle(p: Vec3, o: StaticObstacle): boolean {
  return shapePenetration(o.shape, toObstacleLocal(o, p)).depth > 0;
}

/** Local points along every edge of a shape, at most `spacing` apart. */
export function shapeEdgePoints(s: ConvexShape, spacing: number): Vec3[] {
  const out: Vec3[] = [];
  for (const [i, j] of s.edges) {
    const a = s.vertices[i]!,
      b = s.vertices[j]!;
    const len = Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
    const steps = Math.max(1, Math.ceil(len / spacing));
    for (let k = 0; k <= steps; k++) {
      const t = k / steps;
      out.push(vec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t));
    }
  }
  return out;
}

/** World points along an obstacle's edges. */
export function obstacleEdgePoints(o: StaticObstacle, spacing: number): Vec3[] {
  return shapeEdgePoints(o.shape, spacing).map((p) => add(o.position, quatRotate(o.quaternion, p)));
}

/** Bounding sphere of an obstacle in the world. */
export function obstacleBounds(o: StaticObstacle): { center: Vec3; radius: number } {
  const v = o.shape.vertices;
  const c = vec3(
    v.reduce((s, p) => s + p.x, 0) / v.length,
    v.reduce((s, p) => s + p.y, 0) / v.length,
    v.reduce((s, p) => s + p.z, 0) / v.length,
  );
  const radius = Math.max(...v.map((p) => Math.hypot(p.x - c.x, p.y - c.y, p.z - c.z)));
  return { center: add(o.position, quatRotate(o.quaternion, c)), radius };
}

/** Top of an upright prism obstacle above (x, z), or -Infinity outside its footprint. */
export function obstacleTopAt(o: StaticObstacle, x: number, z: number): number {
  const ys = o.shape.vertices.map((p) => p.y);
  const midY = (Math.min(...ys) + Math.max(...ys)) / 2;
  // Test at mid-height in the obstacle's frame, against the side planes only.
  const l = toObstacleLocal(o, vec3(x, o.position.y + midY, z));
  for (const pl of o.shape.planes) {
    if (Math.abs(pl.n.y) > 0.5) continue;
    if (pl.n.x * l.x + pl.n.z * l.z - pl.d >= 0) return -Infinity;
  }
  return o.position.y + Math.max(...ys);
}

/** Moves a world point out of an obstacle (plus `pad`), through the nearest plane. */
export function pushOutOfObstacle(o: StaticObstacle, p: Vec3, pad = 0): Vec3 {
  const l = toObstacleLocal(o, p);
  let best = -Infinity,
    plane = 0;
  o.shape.planes.forEach((pl, k) => {
    const d = pl.n.x * l.x + pl.n.y * l.y + pl.n.z * l.z - pl.d - pad;
    if (d > best) {
      best = d;
      plane = k;
    }
  });
  if (best >= 0) return p;
  const n = quatRotate(o.quaternion, o.shape.planes[plane]!.n);
  return vec3(p.x - n.x * best, p.y - n.y * best, p.z - n.z * best);
}
