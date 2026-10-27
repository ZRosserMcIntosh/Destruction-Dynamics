// A crash-test car driven into breakable structures. The car is a kinematic
// body moved by a one-dimensional crash model:
//
// - Against anything still standing (fixed pieces), the front crumples: crush
//   depth grows with the closing speed and resists with a crush force that
//   rises quickly to a plateau, then stiffens sharply at the cabin. The
//   structure feels that force plus the momentum flux of the front being
//   brought to rest, μv² (Riera's model for deformable impactors, Nucl. Eng.
//   Des. 8, 1968), as impulse at three points across the bumper.
// - When that frees pieces, the car shares its momentum with them
//   (perfectly inelastic), then shoves them as a kinematic body; momentum the
//   solver gives to loose debris it touches is taken back off the car.
// - Two seated dummies ride on belts (with slack) and airbags that fire above
//   a deceleration threshold; their torso travel and head swing are what the
//   view animates. All numbers are a fictional, simplified model.

import RAPIER from '@dimforge/rapier3d-compat';
import { type FractureWorld, GROUP_IMPACTOR, pieceRadius } from './fractureWorld';

export interface CrashCarSpec {
  mass: number;
  length: number;
  width: number;
  height: number;
  clearance: number;
  /** Crush force plateau of the front structure (N). */
  crushForce: number;
  /** Crush depth over which the force builds to its plateau (m). */
  crushRise: number;
  /** Crushable depth before the cabin (m). */
  crushable: number;
  /** Stiffness once the cabin is reached (N/m). */
  cabinStiffness: number;
  /** Braking deceleration after the hit (m/s²). */
  brake: number;
  /** Mass per metre of the crushing front, for the μv² term (kg/m). */
  frontMassPerLength: number;
}

export const CRASH_CAR: CrashCarSpec = {
  mass: 1400,
  length: 4.4,
  width: 1.8,
  height: 1.45,
  clearance: 0.16,
  crushForce: 300_000,
  crushRise: 0.05,
  crushable: 0.9,
  cabinStiffness: 3_000_000,
  brake: 7,
  frontMassPerLength: 300,
};

export interface Occupant {
  /** Forward travel of the torso relative to the seat (m). */
  travel: number;
  speed: number;
  /** Head pitch on the neck (rad, forward positive). */
  head: number;
  headRate: number;
  /** Peak chest deceleration (g). */
  peakChest: number;
  airbag: number;
}

const G = 9.81;

export class CrashCar {
  readonly spec: CrashCarSpec;
  readonly body: RAPIER.RigidBody;
  private readonly collider: RAPIER.Collider;
  /** x of the rear bumper; the car drives towards +x. */
  rear: number;
  readonly z: number;
  speed: number;
  crush = 0;
  /** Current and peak deceleration (g). */
  decel = 0;
  peakDecel = 0;
  impactTime: number | null = null;
  airbagTime: number | null = null;
  readonly occupants: Occupant[] = [0, 1].map(() => ({
    travel: 0,
    speed: 0,
    head: 0,
    headRate: 0,
    peakChest: 0,
    airbag: 0,
  }));
  private touching: { rb: RAPIER.RigidBody; v0: number }[] = [];
  private lastSpeed: number;

  constructor(
    private readonly fw: FractureWorld,
    start: { rear: number; z?: number; speed: number },
    spec: CrashCarSpec = CRASH_CAR,
  ) {
    this.spec = spec;
    this.rear = start.rear;
    this.z = start.z ?? 0;
    this.speed = start.speed;
    this.lastSpeed = start.speed;
    const s = spec;
    this.body = fw.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        this.rear + s.length / 2,
        s.clearance + (s.height - s.clearance) / 2,
        this.z,
      ),
    );
    this.collider = fw.world.createCollider(
      RAPIER.ColliderDesc.cuboid(s.length / 2, (s.height - s.clearance) / 2, s.width / 2)
        .setFriction(0.5)
        .setCollisionGroups(GROUP_IMPACTOR)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS),
      this.body,
    );
  }

  /** Sets the car moving (towed up to speed off camera). */
  launch(speed: number): void {
    this.speed = speed;
    this.lastSpeed = speed;
  }

  get front(): number {
    return this.rear + this.spec.length - this.crush;
  }

  /** Crush force at a crush depth (N). */
  crushForceAt(d: number): number {
    const s = this.spec;
    const f = s.crushForce * (1 - Math.exp(-d / s.crushRise));
    return d > s.crushable ? f + s.cabinStiffness * (d - s.crushable) : f;
  }

  /** Call before FractureWorld.step(). */
  preStep(dt: number): void {
    const s = this.spec;
    const fw = this.fw;
    // Loose debris in contact: remember its speed to charge the car for the push.
    this.touching = [];
    fw.world.contactPairsWith(this.collider, (other) => {
      const rb = other.parent();
      if (rb && rb.isDynamic()) this.touching.push({ rb, v0: rb.linvel().x });
    });
    const travel = this.speed * dt;
    const yLo = s.clearance + 0.1,
      yHi = s.height * 0.75;
    const hit = fw.fixedPiecesInBox(
      { x: this.front - 0.02, y: yLo, z: this.z - s.width / 2 },
      { x: this.front + travel + 0.02, y: yHi, z: this.z + s.width / 2 },
    );
    if (hit.length && this.speed > 0) {
      let wall = Infinity;
      for (const id of hit) wall = Math.min(wall, fw.fixedPieceBounds(id).min.x);
      const gap = Math.max(0, wall - this.front);
      const dCrush = Math.max(0, travel - gap);
      this.crush += dCrush;
      this.impactTime ??= fw.time;
      const F = this.crushForceAt(this.crush);
      this.speed = Math.max(0, this.speed - (F / s.mass) * dt);
      this.rear += travel;
      // Every piece in the front layer the bumper touches takes its share,
      // on its own joints.
      const closing = dCrush / dt;
      const J = (F + s.frontMassPerLength * closing * closing) * dt;
      const front = hit.filter((id) => fw.fixedPieceBounds(id).min.x < wall + 0.06);
      const hits = front.map((id) => {
        const q = fw.graph.pieces[id]!.pose.position;
        return {
          piece: id,
          point: { x: wall, y: q.y, z: q.z },
          impulse: J / front.length,
          radius: pieceRadius(fw.graph.pieces[id]!),
        };
      });
      const r = fw.strike(hits);
      // Only what is in front of the bumper rides along with the car.
      const ahead = r.freed.filter((rb) => {
        const t = rb.translation();
        return Math.abs(t.z - this.z) < s.width / 2 + 0.1 && t.y < s.height && t.x < this.front + 0.6;
      });
      if (ahead.length) this.shareWith(ahead);
    } else {
      this.rear += travel;
      if (this.impactTime !== null) this.speed = Math.max(0, this.speed - s.brake * dt);
    }
    this.body.setNextKinematicTranslation({
      x: this.rear + (s.length - this.crush) / 2,
      y: s.clearance + (s.height - s.clearance) / 2,
      z: this.z,
    });
    this.collider.setHalfExtents({
      x: (s.length - this.crush) / 2,
      y: (s.height - s.clearance) / 2,
      z: s.width / 2,
    });
  }

  /** Call after FractureWorld.step(). */
  postStep(dt: number): void {
    // Charge the car for the momentum it gave loose debris.
    let dp = 0;
    for (const { rb, v0 } of this.touching) {
      // Bodies can be rebuilt (and removed) by fractures during the step.
      if (!this.fw.world.getRigidBody(rb.handle) || this.fw.world.getRigidBody(rb.handle) !== rb) continue;
      const gain = rb.linvel().x - v0;
      if (gain > 0) dp += rb.mass() * gain;
    }
    if (dp > 0) this.speed = Math.max(0, this.speed - dp / this.spec.mass);
    const a = (this.speed - this.lastSpeed) / dt; // ≤ 0 while crashing
    this.lastSpeed = this.speed;
    this.decel = -a / G;
    this.peakDecel = Math.max(this.peakDecel, this.decel);
    if (this.airbagTime === null && this.decel > 8) this.airbagTime = this.fw.time;
    this.stepOccupants(a, dt);
  }

  private shareWith(freed: RAPIER.RigidBody[]): void {
    let m = this.spec.mass;
    let p = this.spec.mass * this.speed;
    for (const rb of freed) {
      m += rb.mass();
      p += rb.mass() * rb.linvel().x;
    }
    this.speed = Math.max(0, p / m);
    for (const rb of freed) {
      const v = rb.linvel();
      rb.setLinvel({ x: this.speed, y: v.y, z: v.z }, true);
    }
  }

  private stepOccupants(a: number, dt: number): void {
    const bag = this.airbagTime !== null && this.fw.time - this.airbagTime > 0.03;
    for (const o of this.occupants) {
      if (bag) o.airbag = Math.min(1, o.airbag + dt / 0.03);
      // Belt: slack, then a stiff, damped strap (per kg of occupant).
      const slack = 0.03;
      let f = 0;
      if (o.travel > slack) f += 900 * (o.travel - slack) + 25 * Math.max(0, o.speed);
      if (o.airbag > 0.9 && o.travel > 0.12) f += 1500 * (o.travel - 0.12);
      if (o.travel < 0) f += 2000 * o.travel + 20 * o.speed; // seat back
      const rel = -a - f; // car decelerates (a < 0): the torso carries on forward
      o.speed += rel * dt;
      o.travel += o.speed * dt;
      const chest = Math.abs(a + rel) / G;
      o.peakChest = Math.max(o.peakChest, chest);
      // Head on the neck, driven by the torso's deceleration.
      const torsoA = a + rel;
      const hRel =
        -torsoA / 0.25 -
        60 * o.head -
        6 * o.headRate -
        (o.airbag > 0.9 && o.head > 0.5 ? 400 * (o.head - 0.5) : 0);
      o.headRate += hRel * dt;
      o.head = Math.max(-0.4, Math.min(1.1, o.head + o.headRate * dt));
    }
  }

  dispose(): void {
    this.fw.world.removeRigidBody(this.body);
  }
}
