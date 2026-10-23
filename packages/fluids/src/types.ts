import type { Vec3 } from '@destruction-dynamics/core';

/** Ground-plane shape of an impact, in the scene frame (x east, z south). */
export type ImpactFootprint =
  | { kind: 'circle'; radius: number }
  /** A box centred on the impact position; `yaw` rotates its long (x) axis about +y. */
  | { kind: 'box'; halfLength: number; halfWidth: number; yaw: number };

/**
 * An artistic simulation input describing something large entering the water.
 * It is not a structural or hydrodynamic impact calculation.
 */
export interface WaterImpactEvent {
  position: Vec3;
  footprint: ImpactFootprint;
  /** 0..~1.5; scales every response of the visual profile. */
  intensity: number;
  /** Seconds over which the body pushes into the water. */
  duration: number;
  /** Dominant horizontal direction of travel (need not be normalised). */
  direction: Vec3;
  /** Name of a registered visual profile (see impactProfiles). */
  visualProfile: string;
}

export interface WaterSample {
  height: number;
  normal: Vec3;
  /** Horizontal surface-current approximation, m/s. */
  velocity: Vec3;
  foam: number;
}

/** What destruction, gameplay and rendering need from any water implementation. */
export interface FluidAdapter {
  readonly name: string;
  readonly seaLevel: number;
  readonly time: number;
  step(dt: number): void;
  heightAt(x: number, z: number): number;
  /** Horizontal surface current at (x, z), m/s (cheaper than sample()). */
  velocityAt(x: number, z: number): Vec3;
  sample(x: number, z: number): WaterSample;
  applyImpact(e: WaterImpactEvent): void;
  /** A short local push (debris splash, moving object), lasting one step. */
  applyDisturbance(position: Vec3, radius: number, strength: number): void;
  reset(): void;
}

/** Bathymetry: positive water depth below sea level, metres (<= 0 is land). */
export type DepthFunction = (x: number, z: number) => number;
