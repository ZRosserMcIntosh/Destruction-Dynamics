// Gameplay collision for the tower through its lifecycle: static boxes while
// intact, kinematic boxes that follow the authored motion while it falls (so
// debris and players are swept by it), and static boxes at the wreck poses
// afterwards. The mode is derived from event state and time, so it is correct
// after a seek, a load or a late join.

import { type Pose, quatRotate, add, vec3 } from '@destruction-dynamics/core';
import type { BodyHandle, PhysicsAdapter } from '@destruction-dynamics/physics';
import type { CollapseTimeline, TowerCollapse } from './tower';
import type { CollisionMode } from './types';

export function towerCollisionModeAt(
  state: string,
  t: number,
  timeline: CollapseTimeline,
  /** When the structure stops moving (simulated layers); defaults to shortly after impact. */
  settledAt?: number,
): CollisionMode {
  if (state === 'INTACT' || state === 'PREPARING' || state === 'RESTORATION') return 'intact';
  if (t < timeline.failureStart + 0.5 * (timeline.tipStart - timeline.failureStart)) return 'intact';
  if (t >= (settledAt ?? timeline.impact + 2)) return 'wreck';
  return 'kinematic';
}

export class TowerCollision {
  private handles: BodyHandle[] = [];
  private _mode: CollisionMode | null = null;
  /** Number of mode changes (instrumentation and tests). */
  transitions = 0;

  constructor(
    private readonly physics: PhysicsAdapter,
    private readonly tower: TowerCollapse,
    private readonly group?: number,
  ) {}

  get mode(): CollisionMode | null {
    return this._mode;
  }

  get bodies(): readonly BodyHandle[] {
    return this.handles;
  }

  private centrePose(k: number, p: Pose): Pose {
    const half = this.tower.sectionHalfExtents(k);
    return {
      position: add(p.position, quatRotate(p.quaternion, vec3(0, half.y, 0))),
      quaternion: p.quaternion,
    };
  }

  /** Applies `mode` (rebuilding bodies on change) and moves kinematic bodies to `poses`. */
  update(mode: CollisionMode, poses: readonly Pose[]): void {
    if (mode !== this._mode) {
      this.clear();
      this._mode = mode;
      this.transitions++;
      const kind = mode === 'kinematic' ? 'kinematic' : 'static';
      const source = mode === 'wreck' ? this.tower.wreckPoses() : poses;
      this.handles = source.map((p, k) =>
        this.physics.createBody({
          kind: k === 0 ? 'static' : kind,
          shape: { type: 'box', halfExtents: this.tower.sectionHalfExtents(k) },
          pose: this.centrePose(k, p),
          friction: 0.8,
          ...(this.group !== undefined ? { group: this.group } : {}),
        }),
      );
      return;
    }
    if (mode === 'kinematic')
      poses.forEach((p, k) => {
        const h = this.handles[k];
        if (k > 0 && h !== undefined) this.physics.setKinematicTarget(h, this.centrePose(k, p));
      });
  }

  clear(): void {
    for (const h of this.handles) this.physics.removeBody(h);
    this.handles = [];
    this._mode = null;
  }
}
