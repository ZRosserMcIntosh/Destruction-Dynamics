// Facade failure plan: when each facade panel on the tower lets go.
//
// Computed once per (layout, seed) as a flat array of detach times. A panel is
// attached at event time t iff t < detachTime, so the plan is scrubbable and
// reversible (restoration replays it backwards) and identical on every client
// with the same seed. Physics only decides where a detached panel goes.

import { SeededRandom, type Vec3, vec3 } from '@destruction-dynamics/core';
import type { CollapseTimeline, TowerSection, TowerSpec } from './tower';

/** Faces in section-local terms: +x is east (the fall side). */
export const FACES = ['east', 'west', 'north', 'south'] as const;
export type Face = (typeof FACES)[number];

export interface FacadePanel {
  section: number;
  floor: number;
  face: Face;
  column: number;
}

export interface FacadePlan {
  readonly columns: number;
  readonly panels: readonly FacadePanel[];
  /** Event time each panel detaches; Infinity = never. */
  readonly detachTime: Float32Array;
  /** Panels sorted by detach time (indices), for incremental queries. */
  readonly order: Uint32Array;
}

export function buildFacadePlan(
  spec: TowerSpec,
  sections: readonly TowerSection[],
  timeline: CollapseTimeline,
  columns: number,
  seed: number,
): FacadePlan {
  const rng = new SeededRandom(seed);
  const panels: FacadePanel[] = [];
  const times: number[] = [];
  const { failureStart, tipStart, impact } = timeline;
  for (const sec of sections) {
    for (let f = 0; f < sec.floorCount; f++) {
      const floor = sec.firstFloor + f;
      const nearBreak = Math.abs(floor - spec.breakFloor) <= 3;
      for (const face of FACES) {
        for (let c = 0; c < columns; c++) {
          panels.push({ section: sec.index, floor, face, column: c });
          const r = rng.next();
          let t = Infinity;
          if (nearBreak && r < 0.65) t = failureStart + 0.5 + rng.next() * (tipStart - failureStart);
          else if (sec.stump) {
            if (r < 0.12) t = failureStart + rng.next() * (impact - failureStart);
          } else {
            // Compression on the leading (east) face sheds most during the tip;
            // the rest goes at impact.
            const tipChance = face === 'east' ? 0.4 : face === 'west' ? 0.25 : 0.2;
            const impactChance = face === 'east' ? 0.5 : 0.35;
            if (r < tipChance) {
              const height = (floor - spec.breakFloor) / Math.max(1, spec.floors - spec.breakFloor);
              const p = Math.min(0.98, Math.pow(rng.next(), 0.6) * (0.6 + 0.4 * height));
              t = tipStart + p * (impact - tipStart);
            } else if (r < tipChance + impactChance) t = impact + rng.next() * 1.8;
          }
          times.push(t);
        }
      }
    }
  }
  const detachTime = Float32Array.from(times);
  const order = Uint32Array.from(times.map((_, i) => i)).sort((a, b) => detachTime[a]! - detachTime[b]!);
  return { columns, panels, detachTime, order };
}

/** Panel indices whose detach time lies in [from, to). */
export function panelsDetachingBetween(plan: FacadePlan, from: number, to: number): number[] {
  const out: number[] = [];
  if (!(to > from)) return out;
  // Binary search the first index with time >= from.
  let lo = 0,
    hi = plan.order.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (plan.detachTime[plan.order[mid]!]! < from) lo = mid + 1;
    else hi = mid;
  }
  for (let i = lo; i < plan.order.length; i++) {
    const idx = plan.order[i]!;
    if (plan.detachTime[idx]! >= to) break;
    out.push(idx);
  }
  return out;
}

export interface PanelLocal {
  /** Centre in section-local coordinates (origin at the section's bottom centre). */
  center: Vec3;
  halfExtents: Vec3;
  /** Outward face normal, section-local. */
  normal: Vec3;
}

/** Section-local placement of every panel in a plan (shared by simulation and rendering). */
export function facadePanelGeometry(
  spec: TowerSpec,
  sections: readonly TowerSection[],
  plan: FacadePlan,
): PanelLocal[] {
  const fh = spec.floorHeight;
  const hw = spec.width / 2;
  const hd = spec.depth / 2;
  const thickness = 0.08;
  return plan.panels.map((p) => {
    const sec = sections[p.section]!;
    const alongX = p.face === 'north' || p.face === 'south';
    const faceSpan = alongX ? spec.width : spec.depth;
    const span = faceSpan / plan.columns;
    const u = -faceSpan / 2 + (p.column + 0.5) * span;
    const y = (p.floor - sec.firstFloor + 0.5) * fh;
    const normal =
      p.face === 'east'
        ? vec3(1, 0, 0)
        : p.face === 'west'
          ? vec3(-1, 0, 0)
          : p.face === 'south'
            ? vec3(0, 0, 1)
            : vec3(0, 0, -1);
    return {
      center: vec3(alongX ? u : normal.x * hw, y, alongX ? normal.z * hd : u),
      halfExtents: vec3(
        alongX ? span / 2 - 0.12 : thickness,
        fh / 2 - 0.3,
        alongX ? thickness : span / 2 - 0.12,
      ),
      normal,
    };
  });
}
