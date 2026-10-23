// Visual profiles turn an abstract WaterImpactEvent into concrete responses.
// Values are artistic, tuned for readability at city scale.

export interface ImpactProfile {
  /** Peak pressure head pushed into the near-field solver, m. */
  pressureHead: number;
  /** Peak authored surge crest, m (0 = none). */
  surgeAmplitude: number;
  surgeSpeed: number;
  surgeWidth: number;
  surgeLifetime: number;
  /** Momentum pushed along the direction of travel, m/s². */
  push: number;
  /** Relative spray and mist emission (consumed by the effects layer). */
  spray: number;
  mist: number;
}

export const IMPACT_PROFILES: Record<string, ImpactProfile> = {
  'landmark-collapse': {
    pressureHead: 16,
    surgeAmplitude: 9,
    surgeSpeed: 19,
    surgeWidth: 22,
    surgeLifetime: 26,
    push: 6,
    spray: 1,
    mist: 1,
  },
  'secondary-surge': {
    pressureHead: 8,
    surgeAmplitude: 4,
    surgeSpeed: 14,
    surgeWidth: 16,
    surgeLifetime: 14,
    push: 2,
    spray: 0.3,
    mist: 0.3,
  },
  'debris-splash': {
    pressureHead: 2,
    surgeAmplitude: 0,
    surgeSpeed: 0,
    surgeWidth: 1,
    surgeLifetime: 1,
    push: 0,
    spray: 0.05,
    mist: 0,
  },
};

export function impactProfile(name: string): ImpactProfile {
  return IMPACT_PROFILES[name] ?? (IMPACT_PROFILES['debris-splash'] as ImpactProfile);
}
