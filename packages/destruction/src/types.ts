// Reusable destruction interfaces. Three scales share one vocabulary:
//   micro — glass, signs, small fragments: particles and pooled short-lived bodies
//   meso  — walls, facade sections, slabs: authored fracture plus selective rigid bodies
//   macro — landmark structures: authored motion with bounded runtime physics

import type { Pose, Vec3 } from '@destruction-dynamics/core';

export type DestructionScale = 'micro' | 'meso' | 'macro';

export type DamageState = 'intact' | 'damaged' | 'failing' | 'collapsed' | 'wreck' | 'restoring';

/** Which collision representation gameplay should see. */
export type CollisionMode = 'intact' | 'kinematic' | 'wreck';

/** Reference to pre-fractured geometry, streamed on demand. */
export interface FractureAssetRef {
  uri: string;
  pieces: number;
  /** Approximate bytes, for streaming budgets. */
  bytes?: number;
}

export interface DebrisSpawn {
  kind: 'panel' | 'chunk' | 'prop';
  pose: Pose;
  halfExtents: Vec3;
  velocity: Vec3;
  angularVelocity: Vec3;
  /** Seconds before the body is retired (it may be retired earlier for budget). */
  lifetime: number;
  density: number;
}

/** A destructible entity at any scale. */
export interface DestructibleEntity {
  readonly id: string;
  readonly scale: DestructionScale;
  readonly damage: DamageState;
  readonly collision: CollisionMode;
  /** Optional pre-fractured asset used once the entity breaks. */
  readonly fracture?: FractureAssetRef;
  /** Evaluate continuous state at an event time (null = intact). */
  evaluate(time: number | null, restorationProgress: number | null): void;
  /** Return to the intact state immediately. */
  restore(): void;
}
