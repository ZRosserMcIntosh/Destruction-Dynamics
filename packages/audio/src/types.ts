import type { Vec3 } from '@destruction-dynamics/core';

/** Sound layers the destruction framework knows how to request. */
export const SOUND_TYPES = [
  'groan',
  'metal',
  'glass',
  'rumble',
  'debris',
  'water-impact',
  'splash',
  'echo',
  'bubbles',
  'cable-snap',
  'cable-hum',
] as const;
export type SoundType = (typeof SOUND_TYPES)[number];

export interface SoundCue {
  type: SoundType;
  /** Scene-frame position; omitted = non-positional. */
  position?: Vec3;
  gain: number;
  /** Seconds for sustained layers. */
  duration?: number;
  /** Seed for procedural variation. */
  seed?: number;
}

export interface ListenerState {
  position: Vec3;
  forward: Vec3;
  up: Vec3;
  underwater: boolean;
}

/**
 * Anything that can play destruction sound cues. Physics and destruction code
 * never call this directly: the cue router turns director cues into SoundCues.
 */
export interface AudioEngine {
  readonly enabled: boolean;
  /** Must be called from a user gesture in browsers. */
  start(): Promise<void>;
  play(cue: SoundCue): void;
  setListener(l: ListenerState): void;
  /** Silence everything (reset, scrub). */
  stopAll(): void;
  readonly activeVoices: number;
}
