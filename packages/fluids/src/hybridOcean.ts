// The default FluidAdapter: far-field Gerstner swell everywhere, a near-field
// shallow-water solver around the action, and authored surges for landmark
// impacts. Rendering reads `composite` (near-field height + surges, and foam)
// as a texture and adds the same Gerstner waves in its vertex shader.

import { type Vec3, normalize } from '@destruction-dynamics/core';
import { type AuthoredSurge, surgeHeight, surgeVelocity } from './authoredWaves';
import { FarFieldOcean, type GerstnerWave } from './farField';
import { impactProfile } from './profiles';
import { NearFieldSolver, type NearFieldOptions } from './shallowWater';
import type { DepthFunction, FluidAdapter, WaterImpactEvent, WaterSample } from './types';

interface ActiveImpact {
  event: WaterImpactEvent;
  start: number;
}

export interface HybridOceanOptions {
  waves: readonly GerstnerWave[];
  nearField: Omit<NearFieldOptions, 'depth'>;
  depth: DepthFunction;
  seaLevel?: number;
}

export class HybridOcean implements FluidAdapter {
  readonly name = 'hybrid-ocean';
  readonly seaLevel: number;
  readonly far: FarFieldOcean;
  near: NearFieldSolver;
  /** RG pairs per near-field cell: height (solver + surges), foam. Updated by step(). */
  composite: Float32Array;
  private readonly depthFn: DepthFunction;
  private readonly impacts: ActiveImpact[] = [];
  readonly surges: AuthoredSurge[] = [];
  private _time = 0;
  /** Run the solver only every Nth step (budget governor). */
  stepDivisor = 1;
  private stepCounter = 0;
  private pendingDt = 0;

  constructor(private readonly opts: HybridOceanOptions) {
    this.seaLevel = opts.seaLevel ?? 0;
    this.far = new FarFieldOcean(opts.waves);
    this.depthFn = opts.depth;
    this.near = new NearFieldSolver({ ...opts.nearField, depth: opts.depth });
    this.composite = new Float32Array(this.near.n * this.near.n * 2);
  }

  get time(): number {
    return this._time;
  }

  /** Rebuilds the near-field grid at a new resolution (quality change). */
  resize(size: number): void {
    this.near = new NearFieldSolver({ ...this.opts.nearField, size, depth: this.depthFn });
    this.composite = new Float32Array(this.near.n * this.near.n * 2);
    this.stepCounter = 0;
    this.pendingDt = 0;
  }

  /**
   * `age` (s) re-creates an impact that began in the past, e.g. after a seek or
   * a late join: its surge is placed where it would be now. Near-field ripples
   * from before the seek are not reconstructed.
   */
  applyImpact(event: WaterImpactEvent, age = 0): void {
    const start = this._time - Math.max(0, age);
    if (age < event.duration) this.impacts.push({ event, start });
    const p = impactProfile(event.visualProfile);
    if (p.surgeAmplitude <= 0) return;
    const dir = normalize({ x: event.direction.x, y: 0, z: event.direction.z });
    let ax = event.position.x,
      az = event.position.z,
      bx = ax,
      bz = az;
    if (event.footprint.kind === 'box') {
      const c = Math.cos(event.footprint.yaw);
      const s = Math.sin(event.footprint.yaw);
      const l = Math.max(0, event.footprint.halfLength - event.footprint.halfWidth);
      ax -= c * l;
      az -= s * l;
      bx += c * l;
      bz += s * l;
    }
    this.surges.push({
      ax,
      az,
      bx,
      bz,
      startTime: start,
      speed: p.surgeSpeed,
      amplitude: p.surgeAmplitude * event.intensity,
      width: p.surgeWidth,
      spreadDistance: 120,
      lifetime: p.surgeLifetime,
      dirX: dir.x,
      dirZ: dir.z,
      directionality: 0.5,
    });
  }

  applyDisturbance(position: Vec3, radius: number, strength: number): void {
    if (this.near.contains(position.x, position.z))
      this.near.addPressureDisc(position.x, position.z, radius, strength);
  }

  step(dt: number): void {
    if (dt <= 0) return;
    this._time += dt;
    this.pendingDt += dt;
    // Impact forcing for this step (kept even when the solver step is skipped).
    for (let k = this.impacts.length - 1; k >= 0; k--) {
      const a = this.impacts[k]!;
      const age = this._time - a.start;
      const e = a.event;
      if (age > e.duration) {
        this.impacts.splice(k, 1);
        continue;
      }
      const p = impactProfile(e.visualProfile);
      const env = Math.sin((Math.PI * Math.min(age, e.duration)) / e.duration);
      const head = p.pressureHead * e.intensity * env;
      if (e.footprint.kind === 'box')
        this.near.addPressureBox(
          e.position.x,
          e.position.z,
          e.footprint.halfLength,
          e.footprint.halfWidth,
          e.footprint.yaw,
          head,
        );
      else this.near.addPressureDisc(e.position.x, e.position.z, e.footprint.radius, head);
      if (p.push > 0) {
        const dir = normalize({ x: e.direction.x, y: 0, z: e.direction.z });
        const r = e.footprint.kind === 'box' ? e.footprint.halfLength : e.footprint.radius;
        this.near.addCurrent(
          e.position.x,
          e.position.z,
          r * 1.2,
          dir.x * p.push * env,
          dir.z * p.push * env,
          dt,
        );
      }
    }
    for (let k = this.surges.length - 1; k >= 0; k--)
      if (this._time - this.surges[k]!.startTime > this.surges[k]!.lifetime * 5) this.surges.splice(k, 1);

    if (++this.stepCounter % Math.max(1, this.stepDivisor) === 0) {
      this.near.step(this.pendingDt);
      this.pendingDt = 0;
    }
    this.updateComposite();
  }

  private updateComposite(): void {
    const { near, composite, surges } = this;
    const n = near.n;
    const hasSurges = surges.length > 0;
    const band = Math.max(2, n * 0.1);
    for (let j = 0; j < n; j++) {
      const z = near.originZ + (j + 0.5) * near.dx;
      for (let i = 0; i < n; i++) {
        const cell = j * n + i;
        let h = near.eta[cell]!;
        let foam = near.foam[cell]!;
        if (hasSurges && near.depth[cell]! > 0) {
          const x = near.originX + (i + 0.5) * near.dx;
          // Surges fade out across the sponge band so nothing is clipped at the domain edge.
          const e = Math.min(1, Math.min(i, j, n - 1 - i, n - 1 - j) / band);
          const fade = e * e * (3 - 2 * e);
          let s = 0;
          for (const surge of surges) s += surgeHeight(surge, x, z, this._time);
          s *= fade;
          h += s;
          // Only the breaking front of a surge foams, not the whole raised water.
          foam = Math.min(1, foam + Math.max(0, s - 2.5) * 0.12);
        }
        composite[cell * 2] = h;
        composite[cell * 2 + 1] = foam;
      }
    }
  }

  /** Near-field + surge height relative to the far-field swell (from the composite grid). */
  private nearHeight(x: number, z: number): number {
    const near = this.near;
    const n = near.n;
    const { fx, fz } = near.cellCoords(x, z);
    // Outside the near-field domain only the far-field swell exists (matches rendering).
    if (fx < 0 || fz < 0 || fx > n - 1 || fz > n - 1) return 0;
    const i = Math.min(n - 2, Math.floor(fx));
    const j = Math.min(n - 2, Math.floor(fz));
    const tx = fx - i;
    const tz = fz - j;
    const c = this.composite;
    const k = j * n + i;
    const a = c[k * 2]! * (1 - tx) + c[(k + 1) * 2]! * tx;
    const b = c[(k + n) * 2]! * (1 - tx) + c[(k + n + 1) * 2]! * tx;
    return a * (1 - tz) + b * tz;
  }

  heightAt(x: number, z: number): number {
    return this.seaLevel + this.far.heightAt(x, z, this._time) + this.nearHeight(x, z);
  }

  velocityAt(x: number, z: number): Vec3 {
    const fv = this.far.velocityAt(x, z, this._time);
    const nv = this.near.velocityAt(x, z);
    let sx = 0,
      sz = 0;
    const depth = Math.max(1, this.depthFn(x, z));
    for (const s of this.near.contains(x, z) ? this.surges : []) {
      const v = surgeVelocity(s, x, z, this._time, depth);
      sx += v.x;
      sz += v.z;
    }
    return { x: fv.x + nv.x + sx, y: 0, z: fv.z + nv.z + sz };
  }

  sample(x: number, z: number): WaterSample {
    const e = 1.0;
    const h = this.heightAt(x, z);
    const hx = this.heightAt(x + e, z) - this.heightAt(x - e, z);
    const hz = this.heightAt(x, z + e) - this.heightAt(x, z - e);
    return {
      height: h,
      normal: normalize({ x: -hx, y: 2 * e, z: -hz }),
      velocity: this.velocityAt(x, z),
      foam: this.near.sampleField(this.near.foam, x, z),
    };
  }

  /** Largest |near-field + surge| height in the composite grid (instrumentation). */
  maxDisturbance(): number {
    let m = 0;
    for (let k = 0; k < this.composite.length; k += 2) m = Math.max(m, Math.abs(this.composite[k]!));
    return m;
  }

  get activeImpacts(): number {
    return this.impacts.length;
  }

  reset(): void {
    this.near.reset();
    this.composite.fill(0);
    this.impacts.length = 0;
    this.surges.length = 0;
    this._time = 0;
    this.stepCounter = 0;
    this.pendingDt = 0;
  }
}
