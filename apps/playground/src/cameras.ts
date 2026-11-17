// Camera modes: free orbit, the director's cinematic shots, street level,
// offshore on a boat, underwater swimming, and following debris.

import { SeededRandom } from '@destruction-dynamics/core';
import * as THREE from 'three';
import { pushOutOfObstacle } from '@destruction-dynamics/destruction';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { DeepEndSimulation } from './simulation';

export const CAMERA_MODES = ['free', 'cinematic', 'street', 'offshore', 'underwater', 'follow'] as const;
export type CameraMode = (typeof CAMERA_MODES)[number];

interface Shot {
  pos: THREE.Vector3;
  target: THREE.Vector3;
}

type ShotFn = (sim: DeepEndSimulation, age: number) => Shot;

const v = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z);

const towerTop = (sim: DeepEndSimulation) => {
  const k = sim.tower.sections.length - 1;
  const c = sim.tower.sectionCenter(k, sim.poses[k]!);
  return v(c.x, c.y, c.z);
};

const SHOTS: Record<string, ShotFn> = {
  'street-establishing': (_s, a) => ({ pos: v(30, 3.4, 135 - a * 1.5), target: v(0, 160, 0) }),
  'street-low': (_s, a) => ({ pos: v(42 + a * 0.6, 2.2, 78), target: v(0, 130, 0) }),
  'offshore-wide': () => ({ pos: v(720, 30, 560), target: v(110, 110, 0) }),
  'beach-tracking': (s) => ({ pos: v(160, 7, 280), target: towerTop(s) }),
  'impact-wide': (_s, a) => ({ pos: v(600 - a * 4, 48, 470 - a * 3), target: v(190, 6, 0) }),
  'offshore-aftermath': (_s, a) => ({ pos: v(520 - a * 1.5, 20, 320 - a), target: v(200, -2, 0) }),
};

/** Middle of the cheese wedge's top face (apex + ⅔ radius along its axis). */
const cheeseTop = (sim: DeepEndSimulation) => {
  const o = sim.variant.obstacles[0]!;
  const w = sim.variant.cheeseWedge!;
  const q = new THREE.Quaternion(o.quaternion.x, o.quaternion.y, o.quaternion.z, o.quaternion.w);
  return v(o.position.x, o.position.y, o.position.z).add(
    v((2 * w.radius) / 3, w.height, 0).applyQuaternion(q),
  );
};

/** Shots that replace the defaults when the scenario has a cheese wedge to show off. */
const CHEESE_SHOTS: Record<string, ShotFn> = {
  // Tower and wedge together from offshore and above.
  'street-establishing': (s, a) => ({
    pos: v(430, 140 - a, 300),
    target: v(90, 70, 0).lerp(cheeseTop(s), 0.4),
  }),
  'offshore-wide': (s) => ({ pos: v(520, 90, 380), target: v(80, 80, 0).lerp(cheeseTop(s), 0.5) }),
  // A slow, high orbit over the wedge: its triangle, rind and holes.
  'offshore-aftermath': (s, a) => {
    const c = cheeseTop(s);
    const ang = 0.6 + a * 0.05;
    return { pos: v(c.x + 210 * Math.cos(ang), c.y + 110, c.z + 210 * Math.sin(ang)), target: c };
  },
};

export class CameraRig {
  mode: CameraMode = 'cinematic';
  readonly controls: OrbitControls;
  private shot = 'street-establishing';
  private shotStart = 0;
  private shakeIntensity = 0;
  private shakeUntil = 0;
  private time = 0;
  private readonly noise = new SeededRandom(3);
  private readonly keys = new Set<string>();
  private yaw = 0;
  private pitch = 0;
  private dragging = false;
  private lastPointer = { x: 0, y: 0 };
  private readonly smoothTarget = new THREE.Vector3(0, 140, 0);

  constructor(
    private readonly camera: THREE.PerspectiveCamera,
    private readonly dom: HTMLElement,
    private readonly sim: DeepEndSimulation,
  ) {
    this.controls = new OrbitControls(camera, dom);
    this.controls.target.set(60, 80, 0);
    this.controls.maxDistance = 6000;
    this.controls.enableDamping = true;
    camera.position.set(520, 160, 520);
    window.addEventListener('keydown', (e) => this.keys.add(e.key.toLowerCase()));
    window.addEventListener('keyup', (e) => this.keys.delete(e.key.toLowerCase()));
    dom.addEventListener('pointerdown', (e) => {
      this.dragging = true;
      this.lastPointer = { x: e.clientX, y: e.clientY };
    });
    window.addEventListener('pointerup', () => (this.dragging = false));
    window.addEventListener('pointermove', (e) => {
      if (!this.dragging || this.mode !== 'underwater') return;
      this.yaw -= (e.clientX - this.lastPointer.x) * 0.004;
      this.pitch = THREE.MathUtils.clamp(this.pitch - (e.clientY - this.lastPointer.y) * 0.004, -1.4, 1.4);
      this.lastPointer = { x: e.clientX, y: e.clientY };
    });
    this.setMode('cinematic');
  }

  setMode(mode: CameraMode): void {
    this.mode = mode;
    this.controls.enabled = mode === 'free';
    if (mode === 'free') {
      this.controls.target.copy(this.smoothTarget);
    }
    if (mode === 'underwater') {
      // Start beside the expected wreck site, a few metres below the surface.
      this.camera.position.set(170, -9, 70);
      this.yaw = Math.atan2(-(200 - 170), -(0 - 70));
      this.pitch = -0.12;
    }
  }

  /** Director camera cue. */
  cue(type: string, params: Record<string, unknown>): void {
    if (type === 'shot') {
      const name = String(params.shot ?? '');
      if (SHOTS[name]) {
        this.shot = name;
        this.shotStart = this.time;
      }
    } else if (type === 'shake') {
      this.shakeIntensity = Number(params.intensity ?? 0.3);
      this.shakeUntil = this.time + Number(params.duration ?? 2);
    }
  }

  resetShots(): void {
    this.shot = 'street-establishing';
    this.shotStart = this.time;
    this.shakeUntil = 0;
  }

  private applyShake(dt: number): void {
    if (this.time >= this.shakeUntil || this.mode === 'free' || this.mode === 'underwater') return;
    const k = this.shakeIntensity * Math.min(1, (this.shakeUntil - this.time) / 1.5) * 0.012;
    this.camera.rotation.x += (this.noise.next() - 0.5) * k;
    this.camera.rotation.y += (this.noise.next() - 0.5) * k;
    void dt;
  }

  update(dt: number): void {
    this.time += dt;
    const sim = this.sim;
    const c = this.camera;
    switch (this.mode) {
      case 'free':
        this.controls.update();
        break;
      case 'cinematic': {
        const shot = (sim.variant.cheeseWedge && CHEESE_SHOTS[this.shot]) || SHOTS[this.shot]!;
        const s = shot(sim, this.time - this.shotStart);
        c.position.copy(s.pos);
        this.smoothTarget.lerp(s.target, 1 - Math.exp(-dt * 4));
        if (this.time - this.shotStart < 0.05) this.smoothTarget.copy(s.target);
        c.lookAt(this.smoothTarget);
        break;
      }
      case 'street': {
        c.position.set(34, 3.2, 92);
        const mid = sim.tower.sectionCenter(2, sim.poses[2]!);
        this.smoothTarget.lerp(v(mid.x, mid.y + 60, mid.z), 1 - Math.exp(-dt * 2));
        c.lookAt(this.smoothTarget);
        break;
      }
      case 'offshore': {
        const x = 480,
          z = 260;
        const s = sim.ocean.sample(x, z);
        c.position.set(x, s.height + 3.5, z);
        c.lookAt(80, 90, 0);
        c.rotation.z += s.normal.x * 0.3;
        c.rotation.x += s.normal.z * 0.3;
        break;
      }
      case 'underwater':
        this.swim(dt);
        break;
      case 'follow': {
        const b = sim.debris.active[sim.debris.active.length - 1];
        const p = b ? v(b.pose.position.x, b.pose.position.y, b.pose.position.z) : towerTop(sim);
        this.smoothTarget.lerp(p, 1 - Math.exp(-dt * 3));
        const desired = this.smoothTarget.clone().add(v(40, 25, 60));
        c.position.lerp(desired, 1 - Math.exp(-dt * 2));
        c.lookAt(this.smoothTarget);
        break;
      }
    }
    this.applyShake(dt);
  }

  private swim(dt: number): void {
    const c = this.camera;
    const fwd = v(
      -Math.sin(this.yaw) * Math.cos(this.pitch),
      Math.sin(this.pitch),
      -Math.cos(this.yaw) * Math.cos(this.pitch),
    );
    const right = v(Math.cos(this.yaw), 0, -Math.sin(this.yaw));
    const speed = this.keys.has('shift') ? 14 : 5;
    const move = v(0, 0, 0);
    if (this.keys.has('w') || this.keys.has('arrowup')) move.add(fwd);
    if (this.keys.has('s') || this.keys.has('arrowdown')) move.sub(fwd);
    if (this.keys.has('d') || this.keys.has('arrowright')) move.add(right);
    if (this.keys.has('a') || this.keys.has('arrowleft')) move.sub(right);
    if (this.keys.has('e') || this.keys.has(' ')) move.y += 1;
    if (this.keys.has('q') || this.keys.has('c')) move.y -= 1;
    if (move.lengthSq() > 0) c.position.addScaledVector(move.normalize(), speed * dt);
    // Collide with the seabed and with wreck sections (push out of their boxes).
    const floor = this.sim.ground(c.position.x, c.position.z) + 1.2;
    if (c.position.y < floor) c.position.y = floor;
    const sim = this.sim;
    const boxes = sim.poses.slice(1).map((pose, i) => ({
      centre: sim.tower.sectionCenter(i + 1, pose),
      half: sim.tower.sectionHalfExtents(i + 1),
      pose,
    }));
    for (const o of sim.variant.obstacles) {
      const out = pushOutOfObstacle(o, c.position, 1);
      c.position.set(out.x, out.y, out.z);
    }
    boxes.forEach(({ centre, half, pose }) => {
      const q = new THREE.Quaternion(
        pose.quaternion.x,
        pose.quaternion.y,
        pose.quaternion.z,
        pose.quaternion.w,
      );
      const local = c.position
        .clone()
        .sub(v(centre.x, centre.y, centre.z))
        .applyQuaternion(q.clone().invert());
      const pad = 1;
      const px = half.x + pad - Math.abs(local.x);
      const py = half.y + pad - Math.abs(local.y);
      const pz = half.z + pad - Math.abs(local.z);
      if (px > 0 && py > 0 && pz > 0) {
        if (px <= py && px <= pz) local.x = Math.sign(local.x) * (half.x + pad);
        else if (py <= pz) local.y = Math.sign(local.y) * (half.y + pad);
        else local.z = Math.sign(local.z) * (half.z + pad);
        c.position.copy(local.applyQuaternion(q).add(v(centre.x, centre.y, centre.z)));
      }
    });
    c.rotation.set(0, 0, 0);
    c.lookAt(c.position.clone().add(fwd));
  }

  get shotName(): string {
    return this.shot;
  }
}
