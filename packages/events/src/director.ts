// The cinematic event director: owns an event's lifecycle and timeline and
// tells every other subsystem what happens when. It knows nothing about
// rendering, physics or audio; those subscribe to its cues and read its
// continuous state (state, time, stage progress) each frame.

import { Emitter, SeededRandom, deriveSeed, systemClock, type WallClock } from '@destruction-dynamics/core';
import {
  type CueDefinition,
  type EventDefinition,
  type LifecycleState,
  type StageWindow,
  parseEventDefinition,
  stageWindows,
} from './schema';

export class InvalidTransitionError extends Error {
  constructor(
    public readonly from: LifecycleState,
    public readonly action: string,
  ) {
    super(`cannot ${action} while ${from}`);
  }
}

export interface FiredCue {
  cue: CueDefinition;
  /** Stage id, or "preparation". */
  stageId: string;
  /** Authored absolute event time of the cue. */
  time: number;
  /** How far past its authored time the cue fired (large after long frames). */
  lateBy: number;
  /** Seed for any procedural randomness this cue drives. */
  seed: number;
}

export type ChangeCause = 'update' | 'control' | 'seek' | 'load' | 'authority';

export interface StateChange {
  from: LifecycleState;
  to: LifecycleState;
  time: number;
  cause: ChangeCause;
}

export type ControlOp =
  | { op: 'arm' }
  | { op: 'trigger' }
  | { op: 'cancel' }
  | { op: 'pause' }
  | { op: 'resume' }
  | { op: 'reset' }
  | { op: 'restoreNow' }
  | { op: 'timeScale'; value: number }
  | { op: 'seek'; time: number; fireCues: boolean }
  | { op: 'update'; dt: number };

export interface DirectorEvents {
  cue: FiredCue;
  state: StateChange;
  seek: { from: number; to: number; cause: ChangeCause };
  control: ControlOp;
}

export interface DirectorSaveState {
  schema: 'dd-director-save/1';
  eventId: string;
  definitionVersion: number;
  seed: number;
  state: LifecycleState;
  time: number;
  paused: boolean;
  timeScale: number;
  aftermathEnteredAtMs: number | null;
  restoreAtMs: number | null;
  restorationStartTime: number | null;
}

export interface DirectorOptions {
  seed?: number | string;
  clock?: WallClock;
}

interface ScheduledCue extends FiredCue {
  lateBy: 0;
}

const COMMITTED: ReadonlySet<LifecycleState> = new Set([
  'TRIGGERED',
  'INITIAL_FAILURE',
  'ACTIVE_DESTRUCTION',
  'WATER_IMPACT',
  'SETTLING',
  'AFTERMATH',
]);

export class EventDirector {
  readonly definition: EventDefinition;
  readonly windows: readonly StageWindow[];
  private readonly emitter = new Emitter<DirectorEvents>();
  private readonly clock: WallClock;
  private readonly schedule: ScheduledCue[];
  private _seed: number;
  private _state: LifecycleState = 'INTACT';
  private _time: number;
  private _paused = false;
  private _timeScale = 1;
  private aftermathEnteredAtMs: number | null = null;
  private restoreAtMs: number | null = null;
  private restorationStartTime: number | null = null;
  /** When true, restoration is decided by a remote authority (see syncToAuthority). */
  followsAuthority = false;

  constructor(definition: EventDefinition | unknown, opts: DirectorOptions = {}) {
    this.definition = parseEventDefinition(definition);
    this.windows = stageWindows(this.definition);
    this.clock = opts.clock ?? systemClock;
    this._seed = this.normalizeSeed(opts.seed ?? this.definition.id);
    this._time = -this.definition.preparation.duration;
    this.schedule = this.buildSchedule();
  }

  // -- read-only state ------------------------------------------------------

  get state(): LifecycleState {
    return this._state;
  }
  /** Event time in seconds; 0 is the commit instant. */
  get time(): number {
    return this._time;
  }
  get seed(): number {
    return this._seed;
  }
  get paused(): boolean {
    return this._paused;
  }
  get timeScale(): number {
    return this._timeScale;
  }
  get committed(): boolean {
    return COMMITTED.has(this._state);
  }
  /** Seconds of event time from commit to the start of AFTERMATH. */
  get committedDuration(): number {
    const last = this.windows[this.windows.length - 1] as StageWindow;
    return last.start;
  }
  get restorationDueAtMs(): number | null {
    return this.restoreAtMs;
  }

  /** The stage the timeline is in, with progress 0..1 (0 for open-ended). */
  get stage(): (StageWindow & { progress: number }) | null {
    if (!this.committed && this._state !== 'RESTORATION') return null;
    const w = this.windowAt(this._time);
    if (!w) return null;
    const progress = Number.isFinite(w.end) ? (this._time - w.start) / (w.end - w.start) : 0;
    return { ...w, progress: Math.min(1, Math.max(0, progress)) };
  }

  /** Progress 0..1 through the restoration transition, null outside it. */
  get restorationProgress(): number | null {
    if (this._state !== 'RESTORATION' || this.restorationStartTime === null) return null;
    const d = this.definition.restoration.transitionSeconds;
    return d > 0 ? Math.min(1, (this._time - this.restorationStartTime) / d) : 1;
  }

  on<K extends keyof DirectorEvents>(type: K, fn: (p: DirectorEvents[K]) => void): () => void {
    return this.emitter.on(type, fn);
  }

  /** A deterministic random stream for a named secondary effect. */
  rng(stream: string): SeededRandom {
    return new SeededRandom(deriveSeed(this._seed, stream));
  }

  setSeed(seed: number | string): void {
    if (this._state !== 'INTACT') throw new InvalidTransitionError(this._state, 'change the seed');
    this._seed = this.normalizeSeed(seed);
    this.schedule.splice(0, this.schedule.length, ...this.buildSchedule());
  }

  // -- controls -------------------------------------------------------------

  /** Starts the cancellable preparation window. */
  arm(): void {
    this.control({ op: 'arm' });
    if (this._state !== 'INTACT') throw new InvalidTransitionError(this._state, 'arm');
    this._time = -this.definition.preparation.duration;
    if (this.definition.preparation.duration > 0) this.setState('PREPARING', 'control');
    else this.commit('control');
  }

  /** Commits immediately, skipping whatever preparation remains. */
  trigger(): void {
    this.control({ op: 'trigger' });
    if (this._state !== 'INTACT' && this._state !== 'PREPARING')
      throw new InvalidTransitionError(this._state, 'trigger');
    this.commit('control');
  }

  /** Aborts during preparation. Committed events cannot be cancelled. */
  cancel(): void {
    this.control({ op: 'cancel' });
    if (this._state !== 'PREPARING') throw new InvalidTransitionError(this._state, 'cancel');
    this._time = -this.definition.preparation.duration;
    this.setState('INTACT', 'control');
  }

  pause(): void {
    this.control({ op: 'pause' });
    this._paused = true;
  }

  resume(): void {
    this.control({ op: 'resume' });
    this._paused = false;
  }

  setTimeScale(value: number): void {
    if (!(value >= 0) || !Number.isFinite(value))
      throw new RangeError('time scale must be a finite number >= 0');
    this.control({ op: 'timeScale', value });
    this._timeScale = value;
  }

  /** Hard reset to INTACT (testing and editor use). */
  reset(): void {
    this.control({ op: 'reset' });
    const from = this._time;
    this.clearRestoration();
    this._time = -this.definition.preparation.duration;
    if (this._state !== 'INTACT') this.setState('INTACT', 'control');
    this.emitter.emit('seek', { from, to: this._time, cause: 'control' });
  }

  /** Begins restoration now instead of waiting for the policy (AFTERMATH only). */
  restoreNow(): void {
    this.control({ op: 'restoreNow' });
    if (this._state !== 'AFTERMATH') throw new InvalidTransitionError(this._state, 'restore');
    this.beginRestoration('control');
  }

  /** Reset and commit again from t = 0. */
  restart(): void {
    this.reset();
    this.trigger();
  }

  /** Advances by `dtSeconds` of real time, scaled by the time scale. */
  update(dtSeconds: number): void {
    this.control({ op: 'update', dt: dtSeconds });
    if (this._paused || this._state === 'INTACT') return;
    this.advanceTo(this._time + Math.max(0, dtSeconds) * this._timeScale);
    this.checkRestoration();
  }

  /**
   * Jumps the timeline (editor scrubbing, late join). One-shot cues are skipped
   * unless `fireCues` is set; continuous subsystems should resample on 'seek'.
   */
  seek(time: number, opts: { fireCues?: boolean } = {}): void {
    const fireCues = opts.fireCues ?? false;
    this.control({ op: 'seek', time, fireCues });
    this.seekInternal(time, fireCues, 'seek');
  }

  // -- persistence ----------------------------------------------------------

  save(): DirectorSaveState {
    return {
      schema: 'dd-director-save/1',
      eventId: this.definition.id,
      definitionVersion: this.definition.version,
      seed: this._seed,
      state: this._state,
      time: this._time,
      paused: this._paused,
      timeScale: this._timeScale,
      aftermathEnteredAtMs: this.aftermathEnteredAtMs,
      restoreAtMs: this.restoreAtMs,
      restorationStartTime: this.restorationStartTime,
    };
  }

  load(s: DirectorSaveState): void {
    if (s.schema !== 'dd-director-save/1') throw new Error(`unsupported save schema: ${String(s.schema)}`);
    if (s.eventId !== this.definition.id) throw new Error(`save is for event ${s.eventId}`);
    if (s.definitionVersion !== this.definition.version)
      throw new Error(`save is for version ${s.definitionVersion}, definition is ${this.definition.version}`);
    const from = this._time;
    this._seed = s.seed >>> 0;
    this.schedule.splice(0, this.schedule.length, ...this.buildSchedule());
    this._time = s.time;
    this._paused = s.paused;
    this._timeScale = s.timeScale;
    this.aftermathEnteredAtMs = s.aftermathEnteredAtMs;
    this.restoreAtMs = s.restoreAtMs;
    this.restorationStartTime = s.restorationStartTime;
    if (s.state !== this._state) this.setState(s.state, 'load');
    this.emitter.emit('seek', { from, to: this._time, cause: 'load' });
  }

  // -- internals ------------------------------------------------------------

  /** Seek used by the authority sync; bypasses control logging. */
  seekInternal(time: number, fireCues: boolean, cause: ChangeCause): void {
    const prep = this.definition.preparation.duration;
    const to = Math.max(-prep, time);
    const from = this._time;
    if (fireCues && to > from && this._state !== 'INTACT') this.fireRange(from, to);
    this._time = to;
    const target: LifecycleState = to < 0 ? 'PREPARING' : (this.windowAt(to)?.state ?? 'AFTERMATH');
    if (target === 'AFTERMATH' && this.aftermathEnteredAtMs === null) this.enterAftermathTimers();
    if (target !== 'AFTERMATH') this.clearRestoration();
    if (target !== this._state) this.setState(target, cause);
    this.emitter.emit('seek', { from, to, cause });
  }

  /** Puts the director into RESTORATION at a given progress (authority sync). */
  forceRestoration(progress: number, cause: ChangeCause): void {
    if (this._state !== 'RESTORATION') {
      if (this._state !== 'AFTERMATH') this.seekInternal(this.committedDuration, false, cause);
      this.beginRestoration(cause);
    }
    const d = this.definition.restoration.transitionSeconds;
    this._time = (this.restorationStartTime ?? this._time) + Math.min(1, Math.max(0, progress)) * d;
  }

  private normalizeSeed(seed: number | string): number {
    return typeof seed === 'number' ? seed >>> 0 : deriveSeed(seed, 'event');
  }

  private control(op: ControlOp): void {
    this.emitter.emit('control', op);
  }

  private buildSchedule(): ScheduledCue[] {
    const out: ScheduledCue[] = [];
    const prep = this.definition.preparation;
    for (const cue of prep.cues)
      out.push({
        cue,
        stageId: 'preparation',
        time: cue.at - prep.duration,
        lateBy: 0,
        seed: deriveSeed(this._seed, cue.id),
      });
    this.definition.stages.forEach((s, i) => {
      const w = this.windows[i] as StageWindow;
      for (const cue of s.cues)
        out.push({
          cue,
          stageId: s.id,
          time: w.start + cue.at,
          lateBy: 0,
          seed: deriveSeed(this._seed, cue.id),
        });
    });
    // Stable sort keeps authoring order for simultaneous cues.
    return out.sort((a, b) => a.time - b.time);
  }

  private windowAt(t: number): StageWindow | undefined {
    if (t < 0) return undefined;
    return this.windows.find((w) => t >= w.start && t < w.end) ?? this.windows[this.windows.length - 1];
  }

  private setState(to: LifecycleState, cause: ChangeCause): void {
    const from = this._state;
    this._state = to;
    if (to === 'AFTERMATH' && cause === 'update') this.enterAftermathTimers();
    this.emitter.emit('state', { from, to, time: this._time, cause });
  }

  private commit(cause: ChangeCause): void {
    this._time = 0;
    this.setState('TRIGGERED', cause);
    this.setState(this.windows[0]?.state ?? 'AFTERMATH', cause);
    if (this._state === 'AFTERMATH') this.enterAftermathTimers();
  }

  /** Fires scheduled cues with lo <= time < hi. */
  private fireRange(lo: number, hi: number): void {
    for (const s of this.schedule) {
      if (s.time < lo) continue;
      if (s.time >= hi) break;
      this.emitter.emit('cue', { ...s, lateBy: this._time - s.time });
    }
  }

  private nextBoundary(t: number): number {
    if (this._state === 'PREPARING') return 0;
    if (this._state === 'RESTORATION')
      return (this.restorationStartTime ?? t) + this.definition.restoration.transitionSeconds;
    const w = this.windowAt(t);
    return w ? w.end : Infinity;
  }

  private advanceTo(target: number): void {
    // Walk boundary to boundary so state is correct at every cue and each
    // transition happens at its exact authored time, whatever the frame step.
    let guard = 0;
    while (this._time < target && guard++ < 64) {
      const boundary = this.nextBoundary(this._time);
      const seg = Math.min(target, boundary);
      if (this._state !== 'RESTORATION') {
        const lo = this._time;
        this._time = seg;
        this.fireRange(lo, seg);
      } else this._time = seg;
      if (seg < boundary) break;
      this.crossBoundary();
      if (this._state === 'INTACT') break;
    }
  }

  private crossBoundary(): void {
    if (this._state === 'PREPARING') {
      this.commit('update');
      return;
    }
    if (this._state === 'RESTORATION') {
      this.clearRestoration();
      this._time = -this.definition.preparation.duration;
      this.setState('INTACT', 'update');
      return;
    }
    const next = this.windowAt(this._time);
    if (next && next.state !== this._state) this.setState(next.state, 'update');
  }

  private enterAftermathTimers(): void {
    this.aftermathEnteredAtMs = this.clock.now();
    const r = this.definition.restoration;
    this.restoreAtMs =
      r.mode === 'real-world-duration' ? this.aftermathEnteredAtMs + r.hours * 3_600_000 : null;
  }

  private clearRestoration(): void {
    this.aftermathEnteredAtMs = null;
    this.restoreAtMs = null;
    this.restorationStartTime = null;
  }

  private checkRestoration(): void {
    if (this._state !== 'AFTERMATH' || this.followsAuthority) return;
    const r = this.definition.restoration;
    const due =
      (r.mode === 'real-world-duration' &&
        this.restoreAtMs !== null &&
        this.clock.now() >= this.restoreAtMs) ||
      (r.mode === 'event-duration' && this._time >= this.committedDuration + r.seconds);
    if (due) this.beginRestoration('update');
  }

  private beginRestoration(cause: ChangeCause): void {
    this.restorationStartTime = this._time;
    this.setState('RESTORATION', cause);
    if (this.definition.restoration.transitionSeconds === 0) {
      this.clearRestoration();
      this._time = -this.definition.preparation.duration;
      this.setState('INTACT', cause);
    }
  }
}
