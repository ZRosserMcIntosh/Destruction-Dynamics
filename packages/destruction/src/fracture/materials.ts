// Fracture materials. Values are tuned for a fictional, entertaining result
// at the right order of magnitude (densities are real); they are not
// engineering data and must not be used as such.

export interface FractureMaterial {
  id: string;
  name: string;
  /** kg/m³ (real densities). */
  density: number;
  /**
   * Impulse a joint carries per square metre of joint area before it fails
   * (N·s/m²). Damage accumulates over the impact. Infinity: never fails.
   */
  bondStrength: number;
  /**
   * A box piece shatters when one impact delivers more than
   * toughness × t² (N·s), t being its thinnest dimension: plates punch
   * through with thickness squared. Infinity: never shatters.
   */
  toughness: number;
  /** Cells a piece breaks into when it shatters. */
  shards: number;
  /** Radius over which an impact loads the joints around it (m). */
  damageRadius: number;
  friction: number;
  restitution: number;
  /** Display colour (sRGB hex). */
  color: string;
  /** Rendering hint. */
  look: 'brick' | 'block' | 'glass' | 'hay' | 'cheese' | 'board' | 'ice' | 'melon' | 'plain';
}

const m = (x: FractureMaterial) => x;

export const FRACTURE_MATERIALS = {
  brick: m({
    id: 'brick',
    name: 'Clay brick, cement mortar',
    density: 1900,
    bondStrength: 4_500,
    toughness: 10_700,
    shards: 4,
    damageRadius: 0.55,
    friction: 0.7,
    restitution: 0.05,
    color: '#a5482f',
    look: 'brick',
  }),
  cinder: m({
    id: 'cinder',
    name: 'Hollow concrete block',
    density: 1300,
    bondStrength: 4_100,
    toughness: 4_400,
    shards: 5,
    damageRadius: 0.7,
    friction: 0.7,
    restitution: 0.05,
    color: '#9b9c97',
    look: 'block',
  }),
  glassBlock: m({
    id: 'glassBlock',
    name: 'Glass block',
    density: 2500,
    bondStrength: 3_700,
    toughness: 7_400,
    shards: 9,
    damageRadius: 0.45,
    friction: 0.4,
    restitution: 0.1,
    color: '#bfe3ec',
    look: 'glass',
  }),
  hay: m({
    id: 'hay',
    name: 'Hay bale',
    density: 160,
    bondStrength: 300,
    toughness: Infinity,
    shards: 1,
    damageRadius: 0.8,
    friction: 0.9,
    restitution: 0.02,
    color: '#d9b65a',
    look: 'hay',
  }),
  cheese: m({
    id: 'cheese',
    name: 'Indestructible Swiss',
    density: 1100,
    bondStrength: Infinity,
    toughness: Infinity,
    shards: 1,
    damageRadius: 0.5,
    friction: 0.8,
    restitution: 0.15,
    color: '#f2c94c',
    look: 'cheese',
  }),
  glass: m({
    id: 'glass',
    name: 'Float glass, 6 mm',
    density: 2500,
    bondStrength: 18_000,
    toughness: 40_000,
    shards: 26,
    damageRadius: 0.12,
    friction: 0.3,
    restitution: 0.2,
    color: '#cfe8ee',
    look: 'glass',
  }),
  drywall: m({
    id: 'drywall',
    name: 'Plasterboard, 12.5 mm',
    density: 700,
    bondStrength: 9_000,
    toughness: 35_000,
    shards: 14,
    damageRadius: 0.15,
    friction: 0.6,
    restitution: 0.05,
    color: '#ecebe6',
    look: 'board',
  }),
  plywood: m({
    id: 'plywood',
    name: 'Plywood, 18 mm',
    density: 600,
    bondStrength: 60_000,
    toughness: 300_000,
    shards: 10,
    damageRadius: 0.25,
    friction: 0.6,
    restitution: 0.15,
    color: '#c99a62',
    look: 'board',
  }),
  ice: m({
    id: 'ice',
    name: 'Ice slab',
    density: 917,
    bondStrength: 20_000,
    toughness: 4_000,
    shards: 16,
    damageRadius: 0.2,
    friction: 0.05,
    restitution: 0.1,
    color: '#dff3fb',
    look: 'ice',
  }),
  melon: m({
    id: 'melon',
    name: 'Watermelon',
    density: 960,
    bondStrength: 400,
    toughness: Infinity,
    shards: 1,
    damageRadius: 0.3,
    friction: 0.6,
    restitution: 0.05,
    color: '#2f7d32',
    look: 'melon',
  }),
} as const satisfies Record<string, FractureMaterial>;

export type FractureMaterialId = keyof typeof FRACTURE_MATERIALS;

/** Material by id (throws on an unknown id). */
export function fractureMaterial(id: string): FractureMaterial {
  const mat = (FRACTURE_MATERIALS as Record<string, FractureMaterial>)[id];
  if (!mat) throw new Error(`unknown fracture material ${id}`);
  return mat;
}
