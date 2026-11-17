/** Wall-clock source in milliseconds. Injected so restoration timers are testable. */
export interface WallClock {
  now(): number;
}

export const systemClock: WallClock = { now: () => Date.now() };

export class ManualClock implements WallClock {
  constructor(private ms = 0) {}
  now(): number {
    return this.ms;
  }
  set(ms: number): void {
    this.ms = ms;
  }
  advance(ms: number): void {
    this.ms += ms;
  }
}
