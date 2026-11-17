// Quality presets. Every field is a real budget consumed by a subsystem; the
// e2e suite checks that switching presets changes these values in the running
// playground.

export type QualityPresetId = 'dorito' | 'balanced' | 'high' | 'cinematic';

export interface QualitySettings {
  id: QualityPresetId;
  label: string;
  /** Target frame time the budget governor defends, ms. */
  targetFrameMs: number;
  /** Fraction of the device-pixel resolution rendered. */
  renderScale: number;
  maxPixelRatio: number;
  shadows: boolean;
  shadowMapSize: number;
  /** Rings of the geometric ocean grid (vertices per side ≈ 2 × this). */
  oceanGridResolution: number;
  /** Cells per side of the near-field shallow-water grid. */
  fluidGridSize: number;
  /** Near-field fluid update rate, Hz (before governor degradation). */
  fluidHz: number;
  /** Fixed physics step rate, Hz. */
  physicsHz: number;
  /** Maximum simultaneously simulated debris rigid bodies. */
  maxDebrisBodies: number;
  /** Maximum live particles across spray, mist, dust, glass and bubbles. */
  maxParticles: number;
  /** Facade panel columns per floor per face on the hero tower. */
  facadeColumns: number;
  /** 0..1 fraction of background city buildings drawn. */
  cityDensity: number;
  /** Suspended sediment particles around an underwater camera. */
  underwaterParticles: number;
  caustics: boolean;
  /** Floating props (buoys, crates, boats) simulated with buoyancy. */
  floatingObjects: number;
}

export const QUALITY_PRESETS: Readonly<Record<QualityPresetId, QualitySettings>> = {
  dorito: {
    id: 'dorito',
    label: 'Potato (Dorito Mode)',
    targetFrameMs: 1000 / 30,
    renderScale: 0.6,
    maxPixelRatio: 1,
    shadows: false,
    shadowMapSize: 512,
    oceanGridResolution: 48,
    fluidGridSize: 96,
    fluidHz: 30,
    physicsHz: 30,
    maxDebrisBodies: 40,
    maxParticles: 1500,
    facadeColumns: 3,
    cityDensity: 0.4,
    underwaterParticles: 150,
    caustics: false,
    floatingObjects: 6,
  },
  balanced: {
    id: 'balanced',
    label: 'Balanced',
    targetFrameMs: 1000 / 60,
    renderScale: 0.85,
    maxPixelRatio: 1.5,
    shadows: true,
    shadowMapSize: 1024,
    oceanGridResolution: 96,
    fluidGridSize: 160,
    fluidHz: 60,
    physicsHz: 60,
    maxDebrisBodies: 150,
    maxParticles: 6000,
    facadeColumns: 6,
    cityDensity: 0.7,
    underwaterParticles: 600,
    caustics: true,
    floatingObjects: 16,
  },
  high: {
    id: 'high',
    label: 'High',
    targetFrameMs: 1000 / 60,
    renderScale: 1,
    maxPixelRatio: 2,
    shadows: true,
    shadowMapSize: 2048,
    oceanGridResolution: 140,
    fluidGridSize: 256,
    fluidHz: 60,
    physicsHz: 60,
    maxDebrisBodies: 400,
    maxParticles: 15000,
    facadeColumns: 8,
    cityDensity: 1,
    underwaterParticles: 1500,
    caustics: true,
    floatingObjects: 30,
  },
  cinematic: {
    id: 'cinematic',
    label: 'Cinematic',
    targetFrameMs: 1000 / 30,
    renderScale: 1,
    maxPixelRatio: 2,
    shadows: true,
    shadowMapSize: 4096,
    oceanGridResolution: 200,
    fluidGridSize: 384,
    fluidHz: 60,
    physicsHz: 120,
    maxDebrisBodies: 1000,
    maxParticles: 40000,
    facadeColumns: 10,
    cityDensity: 1,
    underwaterParticles: 3000,
    caustics: true,
    floatingObjects: 48,
  },
};

export const QUALITY_PRESET_IDS = Object.keys(QUALITY_PRESETS) as QualityPresetId[];

export function qualityPreset(id: QualityPresetId): QualitySettings {
  return { ...QUALITY_PRESETS[id] };
}
