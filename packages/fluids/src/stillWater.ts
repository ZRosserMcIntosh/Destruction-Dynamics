import type { Vec3 } from '@destruction-dynamics/core';
import type { FluidAdapter, WaterImpactEvent, WaterSample } from './types';

/**
 * The simplest FluidAdapter: a flat, still surface. Useful for tests, for
 * integrations without water, and as a reference for adapter authors.
 * Impacts and disturbances are accepted and ignored.
 */
export class StillWater implements FluidAdapter {
  readonly name = 'still-water';
  private _time = 0;
  constructor(readonly seaLevel = 0) {}
  get time(): number {
    return this._time;
  }
  step(dt: number): void {
    this._time += Math.max(0, dt);
  }
  heightAt(): number {
    return this.seaLevel;
  }
  velocityAt(): Vec3 {
    return { x: 0, y: 0, z: 0 };
  }
  sample(): WaterSample {
    return { height: this.seaLevel, normal: { x: 0, y: 1, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, foam: 0 };
  }
  applyImpact(_e: WaterImpactEvent): void {}
  applyDisturbance(_p: Vec3, _r: number, _s: number): void {}
  reset(): void {
    this._time = 0;
  }
}
