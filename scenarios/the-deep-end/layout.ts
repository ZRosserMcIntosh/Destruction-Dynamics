// "The Deep End": a fictional 85-storey tower on a Balneário Camboriú-inspired
// beachfront. The city runs north-south along the beach; the Atlantic is to
// the east (+x in the scene frame). Distances are in metres from the tower base.

import type { LocalOrigin } from '@destruction-dynamics/core';
import { type CollapseTimeline, DEEP_END_TOWER, type TowerSpec } from '@destruction-dynamics/destruction';
import { type EventDefinition, parseEventDefinition, windowFor } from '@destruction-dynamics/events';
import eventJson from './event.json';

export const deepEndEvent: EventDefinition = parseEventDefinition(eventJson);

export const SEA_LEVEL = 0;

export const DEEP_END_LAYOUT = {
  tower: DEEP_END_TOWER as TowerSpec,
  /** Promenade road (Avenida Atlântica-style) between the tower row and the sand. */
  road: { minX: 20, maxX: 36 },
  /** Sand from the road to the waterline. */
  beach: { minX: 36, maxX: 92 },
  /** Waterline x; depth grows from here to `shelfX`, then flattens. */
  shoreX: 92,
  shelfX: 250,
  maxDepth: 42,
  /** Ground level of the city block. */
  cityLevel: 1.5,
  /** Event origin in the world (ENU) frame; illustrative placement. */
  origin: { east: 0, north: 0, up: 0 } as LocalOrigin,
} as const;

/** Ground or seabed height in the scene frame (y up). */
export function deepEndGroundHeight(x: number, _z: number): number {
  const L = DEEP_END_LAYOUT;
  if (x <= L.road.maxX) return L.cityLevel;
  if (x <= L.shoreX) {
    // Sand slopes gently from the promenade down to the waterline.
    const t = Math.max(0, (x - L.beach.minX) / (L.shoreX - L.beach.minX));
    return L.cityLevel * (1 - t);
  }
  if (x >= L.shelfX) return -L.maxDepth;
  const t = (x - L.shoreX) / (L.shelfX - L.shoreX);
  // Concave profile: shallow surf zone, then a steeper drop to the shelf.
  return -L.maxDepth * Math.pow(t, 1.4);
}

/** Water depth below sea level (<= 0 on land). */
export function deepEndDepth(x: number, z: number): number {
  return SEA_LEVEL - deepEndGroundHeight(x, z);
}

/** Collapse anchors taken from the event's stage windows. */
export function deepEndTimeline(def: EventDefinition = deepEndEvent): CollapseTimeline {
  const failure = windowFor(def, 'INITIAL_FAILURE');
  const active = windowFor(def, 'ACTIVE_DESTRUCTION');
  const impact = windowFor(def, 'WATER_IMPACT');
  const settling = windowFor(def, 'SETTLING');
  if (!failure || !active || !impact || !settling)
    throw new Error('event lacks the stages a tower collapse needs');
  return {
    failureStart: failure.start,
    tipStart: active.start,
    impact: impact.start,
    settleEnd: settling.end,
  };
}
