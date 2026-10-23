// Frame-time instrumentation and the budget governor.
//
// The governor watches measured frame time and degrades *secondary* work
// (particles, debris bodies, fluid update rate) when the frame runs over
// budget, and restores it when there is headroom. Primary work (the event
// director, authored structural motion, gameplay collision) is never scaled:
// the event stays correct even when the spectacle thins out.

export type CostCategory = 'director' | 'physics' | 'fluid' | 'particles' | 'render' | 'audio' | 'other';

export class FrameProfiler {
  private readonly ema = new Map<CostCategory, number>();
  private readonly current = new Map<CostCategory, number>();
  private frameStart = 0;
  frameMsEma = 0;
  frames = 0;

  constructor(
    private readonly now: () => number = () => performance.now(),
    private readonly smoothing = 0.1,
  ) {}

  beginFrame(): void {
    this.frameStart = this.now();
    this.current.clear();
  }

  /** Times `fn` under `category` and returns its result. */
  measure<T>(category: CostCategory, fn: () => T): T {
    const t0 = this.now();
    try {
      return fn();
    } finally {
      this.add(category, this.now() - t0);
    }
  }

  add(category: CostCategory, ms: number): void {
    this.current.set(category, (this.current.get(category) ?? 0) + ms);
  }

  /** Ends the frame. `frameMs` overrides the measured duration (e.g. rAF delta). */
  endFrame(frameMs?: number): number {
    const ms = frameMs ?? this.now() - this.frameStart;
    const a = this.frames === 0 ? 1 : this.smoothing;
    this.frameMsEma += (ms - this.frameMsEma) * a;
    for (const [k, v] of this.current) {
      const prev = this.ema.get(k);
      this.ema.set(k, prev === undefined ? v : prev + (v - prev) * this.smoothing);
    }
    this.frames++;
    return ms;
  }

  costs(): Record<string, number> {
    return Object.fromEntries(this.ema);
  }
}

export interface DegradationLevel {
  level: number;
  particleScale: number;
  debrisScale: number;
  /** Run the near-field fluid every Nth scheduled step. */
  fluidStepDivisor: number;
}

export const DEGRADATION_LEVELS: readonly DegradationLevel[] = [
  { level: 0, particleScale: 1, debrisScale: 1, fluidStepDivisor: 1 },
  { level: 1, particleScale: 0.6, debrisScale: 0.7, fluidStepDivisor: 1 },
  { level: 2, particleScale: 0.35, debrisScale: 0.45, fluidStepDivisor: 2 },
  { level: 3, particleScale: 0.2, debrisScale: 0.25, fluidStepDivisor: 3 },
];

export interface GovernorOptions {
  targetFrameMs: number;
  /** Over-budget ratio that counts as a slow frame. */
  overRatio?: number;
  /** Under-budget ratio that counts as a fast frame. */
  underRatio?: number;
  /** Consecutive slow frames before degrading one level. */
  degradeAfter?: number;
  /** Consecutive fast frames before restoring one level. */
  restoreAfter?: number;
}

export class BudgetGovernor {
  private slow = 0;
  private fast = 0;
  private levelIndex = 0;
  targetFrameMs: number;
  private readonly overRatio: number;
  private readonly underRatio: number;
  private readonly degradeAfter: number;
  private readonly restoreAfter: number;

  constructor(opts: GovernorOptions) {
    this.targetFrameMs = opts.targetFrameMs;
    this.overRatio = opts.overRatio ?? 1.15;
    this.underRatio = opts.underRatio ?? 0.7;
    this.degradeAfter = opts.degradeAfter ?? 30;
    this.restoreAfter = opts.restoreAfter ?? 120;
  }

  get current(): DegradationLevel {
    return DEGRADATION_LEVELS[this.levelIndex] as DegradationLevel;
  }

  /** Feed one frame's duration; returns the (possibly changed) level. */
  observe(frameMs: number): DegradationLevel {
    if (frameMs > this.targetFrameMs * this.overRatio) {
      this.slow++;
      this.fast = 0;
    } else if (frameMs < this.targetFrameMs * this.underRatio) {
      this.fast++;
      this.slow = 0;
    } else {
      this.slow = 0;
      this.fast = 0;
    }
    if (this.slow >= this.degradeAfter && this.levelIndex < DEGRADATION_LEVELS.length - 1) {
      this.levelIndex++;
      this.slow = 0;
    } else if (this.fast >= this.restoreAfter && this.levelIndex > 0) {
      this.levelIndex--;
      this.fast = 0;
    }
    return this.current;
  }

  reset(): void {
    this.levelIndex = 0;
    this.slow = 0;
    this.fast = 0;
  }
}

/**
 * A hard cap on simultaneous items (debris bodies, particles). `request` grants
 * as many as fit; callers degrade gracefully (e.g. particles instead of bodies).
 */
export class CountBudget {
  private used = 0;
  constructor(public limit: number) {}

  get inUse(): number {
    return this.used;
  }
  get available(): number {
    return Math.max(0, this.limit - this.used);
  }
  request(n: number): number {
    const granted = Math.min(n, this.available);
    this.used += granted;
    return granted;
  }
  release(n: number): void {
    this.used = Math.max(0, this.used - n);
  }
  reset(): void {
    this.used = 0;
  }
}
