// Variants of The Deep End. Same event, same tower, different world around it.
//
// "cheese": requested by a subscriber — "A giant block of cheese in the
// center forever (make it indestructible pls)". A wedge of Swiss cheese,
// cut from a 200 m wheel, sits in the shallows on the tower's fall line. It is a static
// collider for the floor layers and for debris, a wall for the water, and it
// survives every collapse, reset and restoration.

import { DEG, quatFromAxisAngle, vec3 } from '@destruction-dynamics/core';
import { type StaticObstacle, obstacleTopAt, wedgeShape } from '@destruction-dynamics/destruction';
import { SEA_LEVEL, deepEndDepth, deepEndGroundHeight } from './layout';

export type DeepEndVariantId = 'classic' | 'cheese';

export interface DeepEndVariant {
  id: DeepEndVariantId;
  title: string;
  /** Who asked for it, if a viewer did. */
  requestedBy?: string;
  obstacles: StaticObstacle[];
  /** Draw the first obstacle as a cheese wedge with these proportions. */
  cheeseWedge?: { radius: number; angle: number; height: number };
}

/** The wedge as cut from the wheel (also used to draw it). */
export const CHEESE_WEDGE = { radius: 100, angle: 50 * DEG, height: 60 } as const;

/**
 * Apex toward the beach at x = 105, opening out to sea along the tower's fall
 * line; the wheel's bottom is buried in the seabed, its top 32 m above the water.
 */
export const CHEESE_OBSTACLE: StaticObstacle = {
  position: vec3(105, -28, 0),
  quaternion: quatFromAxisAngle(vec3(0, 1, 0), 8 * DEG),
  shape: wedgeShape({ ...CHEESE_WEDGE, rindSegments: 6 }),
};

export const DEEP_END_VARIANTS: Record<DeepEndVariantId, DeepEndVariant> = {
  classic: { id: 'classic', title: 'The Deep End', obstacles: [] },
  cheese: {
    id: 'cheese',
    title: 'The Deep End: indestructible cheese',
    requestedBy: '@Fartman32',
    obstacles: [CHEESE_OBSTACLE],
    cheeseWedge: CHEESE_WEDGE,
  },
};

export function isDeepEndVariant(id: string | null | undefined): id is DeepEndVariantId {
  return id === 'classic' || id === 'cheese';
}

/** Ground including obstacle tops (for particles and cameras; not for the layer solver). */
export function groundWithObstacles(obstacles: readonly StaticObstacle[]) {
  return (x: number, z: number): number => {
    let g = deepEndGroundHeight(x, z);
    for (const o of obstacles) g = Math.max(g, obstacleTopAt(o, x, z));
    return g;
  };
}

/** Water depth with obstacles that break the surface treated as land (walls for the wave solver). */
export function depthWithObstacles(obstacles: readonly StaticObstacle[]) {
  return (x: number, z: number): number => {
    for (const o of obstacles) if (obstacleTopAt(o, x, z) > SEA_LEVEL) return 0;
    return deepEndDepth(x, z);
  };
}
