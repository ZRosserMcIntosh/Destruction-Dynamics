// Turns event-director audio cues into engine SoundCues. Kept separate from
// the engine so routing is testable without WebAudio and so a game can swap
// in its own middleware (Wwise, FMOD, Unreal MetaSounds) behind AudioEngine.

import type { Vec3 } from '@destruction-dynamics/core';
import { SOUND_TYPES, type AudioEngine, type SoundCue, type SoundType } from './types';

export interface DirectorCueLike {
  cue: { id: string; channel: string; type: string; params?: Record<string, unknown> };
  seed: number;
  lateBy: number;
}

export interface RouterOptions {
  /** Default emitter position when a cue carries none (e.g. the event centre). */
  defaultPosition: () => Vec3;
  /** Cues fired later than this (s) are dropped (e.g. after a long frame or a seek). */
  maxLateness?: number;
}

const isVec3 = (v: unknown): v is Vec3 =>
  typeof v === 'object' &&
  v !== null &&
  ['x', 'y', 'z'].every((k) => typeof (v as Record<string, unknown>)[k] === 'number');

export class AudioCueRouter {
  routed = 0;
  dropped = 0;

  constructor(
    private readonly engine: AudioEngine,
    private readonly opts: RouterOptions,
  ) {}

  /** Returns the SoundCue sent to the engine, or null if the cue was not audio or was dropped. */
  handle(fired: DirectorCueLike): SoundCue | null {
    const { cue } = fired;
    if (cue.channel !== 'audio') return null;
    if (!SOUND_TYPES.includes(cue.type as SoundType) || fired.lateBy > (this.opts.maxLateness ?? 0.75)) {
      this.dropped++;
      return null;
    }
    const p = cue.params ?? {};
    const sound: SoundCue = {
      type: cue.type as SoundType,
      gain: typeof p.gain === 'number' ? p.gain : 1,
      position: isVec3(p.position) ? p.position : this.opts.defaultPosition(),
      seed: fired.seed,
      ...(typeof p.duration === 'number' ? { duration: p.duration } : {}),
    };
    this.engine.play(sound);
    this.routed++;
    return sound;
  }
}

/** Records cues; for tests and headless runs. */
export class NullAudioEngine implements AudioEngine {
  readonly enabled = false;
  readonly played: SoundCue[] = [];
  listener: Parameters<AudioEngine['setListener']>[0] | null = null;
  get activeVoices(): number {
    return 0;
  }
  async start(): Promise<void> {}
  play(cue: SoundCue): void {
    this.played.push(cue);
  }
  setListener(l: Parameters<AudioEngine['setListener']>[0]): void {
    this.listener = l;
  }
  stopAll(): void {}
}
