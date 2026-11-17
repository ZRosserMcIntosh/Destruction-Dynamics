// Coordinate frames.
//
// World frame (ENU): X east, Y north, Z up, metres. This is the frame of the
// Brazil open-world tiles and of Aviation Dynamics (`avd::world`).
//
// Scene frame: X east, Y up, Z south, metres. Right-handed and Y-up, so it maps
// directly onto Three.js. All simulation packages in this repository work in
// the scene frame, relative to an event-local origin; conversion to world
// coordinates happens only at integration boundaries.
//
//   ENU -> scene: (e, n, u) -> (e, u, -n)   (a proper rotation, det = +1)

import type { Vec3 } from './math';

export interface EnuPoint {
  east: number;
  north: number;
  up: number;
}

export const sceneFromEnu = (p: EnuPoint): Vec3 => ({ x: p.east, y: p.up, z: -p.north });
export const enuFromScene = (v: Vec3): EnuPoint => ({ east: v.x, north: -v.z, up: v.y });

/** An event-local origin placed in the world. Large-world positions stay in doubles here only. */
export interface LocalOrigin {
  east: number;
  north: number;
  up: number;
}

export const sceneFromWorld = (origin: LocalOrigin, p: EnuPoint): Vec3 =>
  sceneFromEnu({ east: p.east - origin.east, north: p.north - origin.north, up: p.up - origin.up });

export const worldFromScene = (origin: LocalOrigin, v: Vec3): EnuPoint => {
  const l = enuFromScene(v);
  return { east: l.east + origin.east, north: l.north + origin.north, up: l.up + origin.up };
};
