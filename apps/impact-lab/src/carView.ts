// A procedural crash-test car (no external assets): an extruded side profile
// sampled densely along its length so the front can crumple, a glasshouse,
// wheels, yellow-and-black crash markers, two seated dummies and airbags.
// Car frame: x from the rear bumper (0) forward, y up, z across (centred).

import type { CrashCar, CrashCarSpec } from '@destruction-dynamics/destruction';
import * as THREE from 'three';

/** Side profile, rear bumper to nose (x, y). */
const PROFILE: [number, number][] = [
  [0.12, 0.31],
  [4.26, 0.31],
  [4.38, 0.36],
  [4.44, 0.47],
  [4.44, 0.58],
  [4.38, 0.68],
  [4.22, 0.78],
  [3.9, 0.85],
  [3.35, 0.93],
  [1.1, 0.97],
  [0.55, 0.95],
  [0.2, 0.9],
  [0.05, 0.8],
  [0.0, 0.62],
  [0.03, 0.42],
];
const GLASSHOUSE: [number, number][] = [
  [0.98, 0.96],
  [3.32, 0.92],
  [3.0, 1.12],
  [2.62, 1.36],
  [2.3, 1.41],
  [1.5, 1.41],
  [1.22, 1.3],
];
/** The crumple zone starts this far back from the nose. */
const ZONE = 1.7;

function resample(points: [number, number][], step: number): THREE.Vector2[] {
  const out: THREE.Vector2[] = [];
  for (let i = 0; i < points.length; i++) {
    const a = points[i]!;
    const b = points[(i + 1) % points.length]!;
    const n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / step));
    for (let k = 0; k < n; k++)
      out.push(new THREE.Vector2(a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n));
  }
  return out;
}

function extrude(
  points: [number, number][],
  width: number,
  step: number,
  bevel: number,
): THREE.BufferGeometry {
  const shape = new THREE.Shape(resample(points, step));
  const g = new THREE.ExtrudeGeometry(shape, {
    depth: width - 2 * bevel,
    bevelEnabled: bevel > 0,
    bevelThickness: bevel,
    bevelSize: bevel * 0.8,
    bevelSegments: 3,
    curveSegments: 4,
  });
  g.translate(0, 0, -(width - 2 * bevel) / 2);
  return g;
}

function plateTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 60;
  const g = c.getContext('2d')!;
  g.fillStyle = '#f4f4ef';
  g.fillRect(0, 0, 256, 60);
  g.fillStyle = '#1d3f9a';
  g.fillRect(0, 0, 26, 60);
  g.fillStyle = '#111';
  g.font = 'bold 40px system-ui, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText('DD·CRASH', 140, 32);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function markerTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d')!;
  for (let q = 0; q < 4; q++) {
    g.fillStyle = q % 2 ? '#111' : '#f5c518';
    g.beginPath();
    g.moveTo(32, 32);
    g.arc(32, 32, 30, (q * Math.PI) / 2, ((q + 1) * Math.PI) / 2);
    g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

interface Deformable {
  mesh: THREE.Mesh;
  rest: Float32Array;
}

export class CarView {
  readonly group = new THREE.Group();
  private readonly deform: Deformable[] = [];
  private readonly wheels: THREE.Mesh[] = [];
  private readonly glass: THREE.MeshStandardMaterial;
  private readonly dummies: { torso: THREE.Group; head: THREE.Group; bag: THREE.Mesh }[] = [];
  private lastCrush = -1;
  private readonly spec: CrashCarSpec;

  constructor(spec: CrashCarSpec) {
    this.spec = spec;
    const paint = new THREE.MeshPhysicalMaterial({
      color: 0x8fb0cc,
      roughness: 0.3,
      metalness: 0.45,
      clearcoat: 0.8,
      clearcoatRoughness: 0.15,
    });
    const body = new THREE.Mesh(extrude(PROFILE, spec.width, 0.04, 0.07), paint);
    body.castShadow = true;
    body.receiveShadow = true;
    this.addDeformable(body);
    // Black stripe along the sill and a yellow nose band (test-car livery).
    const trim = new THREE.MeshStandardMaterial({ color: 0x1b1d20, roughness: 0.6 });
    const sill = new THREE.Mesh(
      extrude(
        [
          [0.3, 0.3],
          [4.1, 0.3],
          [4.1, 0.42],
          [0.3, 0.42],
        ],
        spec.width + 0.01,
        0.05,
        0,
      ),
      trim,
    );
    this.addDeformable(sill);
    this.glass = new THREE.MeshStandardMaterial({
      color: 0x2a3a46,
      roughness: 0.05,
      metalness: 0.2,
      transparent: true,
      opacity: 0.38,
      envMapIntensity: 2,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    const gh = new THREE.Mesh(extrude(GLASSHOUSE, spec.width - 0.16, 0.04, 0.03), this.glass);
    this.addDeformable(gh);
    const roof = new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.05, spec.width - 0.14), paint);
    roof.position.set(1.9, 1.42, 0);
    roof.castShadow = true;
    this.group.add(roof);
    // Wheels.
    const tyre = new THREE.MeshStandardMaterial({ color: 0x151617, roughness: 0.85 });
    const rim = new THREE.MeshStandardMaterial({ color: 0xb8bcc2, roughness: 0.3, metalness: 0.9 });
    for (const x of [0.85, 3.55])
      for (const z of [-1, 1]) {
        const w = new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.32, 0.22, 24), tyre);
        w.rotation.x = Math.PI / 2;
        const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.23, 6), rim);
        w.add(hub);
        w.position.set(x, 0.32, z * (spec.width / 2 - 0.12));
        w.castShadow = true;
        this.wheels.push(w);
        this.group.add(w);
      }
    // Crash-test markers on both sides and the roof.
    const marker = new THREE.MeshBasicMaterial({ map: markerTexture(), transparent: true });
    for (const side of [-1, 1]) {
      for (const x of [1.3, 2.9]) {
        const m = new THREE.Mesh(new THREE.CircleGeometry(0.11, 24), marker);
        m.position.set(x, 0.72, side * (spec.width / 2 + 0.005));
        if (side < 0) m.rotation.y = Math.PI;
        this.group.add(m);
      }
    }
    const top = new THREE.Mesh(new THREE.CircleGeometry(0.16, 24), marker);
    top.rotation.x = -Math.PI / 2;
    top.position.set(1.9, 1.45, 0);
    this.group.add(top);
    // Headlights.
    const lamp = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      emissive: 0xfff4d0,
      emissiveIntensity: 0.6,
      roughness: 0.1,
    });
    const lamps = new THREE.Mesh(
      new THREE.BufferGeometry().copy(
        new THREE.BoxGeometry(0.06, 0.1, 0.32).translate(4.33, 0.72, spec.width / 2 - 0.3),
      ),
      lamp,
    );
    this.addDeformable(lamps);
    const lamps2 = new THREE.Mesh(
      new THREE.BoxGeometry(0.06, 0.1, 0.32).translate(4.33, 0.72, -(spec.width / 2 - 0.3)),
      lamp,
    );
    this.addDeformable(lamps2);
    // Bumpers, tail lights and number plates.
    const bumperMat = new THREE.MeshStandardMaterial({ color: 0x202326, roughness: 0.7 });
    this.addDeformable(
      new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.2, spec.width + 0.02).translate(4.42, 0.44, 0), bumperMat),
    );
    const rearBumper = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.2, spec.width + 0.02), bumperMat);
    rearBumper.position.set(0.0, 0.44, 0);
    this.group.add(rearBumper);
    const tail = new THREE.MeshStandardMaterial({
      color: 0x8a0d0d,
      emissive: 0xff2a1a,
      emissiveIntensity: 0.5,
      roughness: 0.2,
    });
    for (const z of [-1, 1]) {
      const t = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.12, 0.34), tail);
      t.position.set(0.02, 0.8, z * (spec.width / 2 - 0.25));
      this.group.add(t);
    }
    const plate = new THREE.MeshStandardMaterial({ map: plateTexture(), roughness: 0.5 });
    const rearPlate = new THREE.Mesh(new THREE.PlaneGeometry(0.52, 0.12), plate);
    rearPlate.position.set(-0.052, 0.62, 0);
    rearPlate.rotation.y = -Math.PI / 2;
    this.group.add(rearPlate);
    this.addDeformable(
      new THREE.Mesh(
        new THREE.PlaneGeometry(0.52, 0.12).rotateY(Math.PI / 2).translate(4.475, 0.44, 0),
        plate,
      ),
    );
    // Dummies and airbags.
    for (const z of [0.4, -0.4]) this.dummies.push(this.makeDummy(z));
    this.group.name = 'crash-car';
  }

  private addDeformable(mesh: THREE.Mesh): void {
    const pos = mesh.geometry.getAttribute('position') as THREE.BufferAttribute;
    this.deform.push({ mesh, rest: Float32Array.from(pos.array as Float32Array) });
    this.group.add(mesh);
  }

  private makeDummy(z: number) {
    const yellow = new THREE.MeshStandardMaterial({ color: 0xf2c51b, roughness: 0.55 });
    const black = new THREE.MeshStandardMaterial({ color: 0x1b1c1f, roughness: 0.6 });
    const root = new THREE.Group();
    root.position.set(1.85, 0.5, z);
    // Pelvis and legs (fixed to the seat).
    const pelvis = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.14, 0.32), yellow);
    root.add(pelvis);
    for (const s of [-0.09, 0.09]) {
      const thigh = new THREE.Mesh(new THREE.CapsuleGeometry(0.06, 0.32, 4, 8), yellow);
      thigh.rotation.z = Math.PI / 2;
      thigh.position.set(0.24, 0.02, s);
      root.add(thigh);
      const shin = new THREE.Mesh(new THREE.CapsuleGeometry(0.05, 0.3, 4, 8), black);
      shin.position.set(0.46, -0.16, s);
      root.add(shin);
    }
    // Torso pivots at the hips, the head at the neck.
    const torso = new THREE.Group();
    const chest = new THREE.Mesh(new THREE.CapsuleGeometry(0.13, 0.28, 4, 10), yellow);
    chest.position.y = 0.3;
    torso.add(chest);
    for (const s of [-0.19, 0.19]) {
      const arm = new THREE.Mesh(new THREE.CapsuleGeometry(0.045, 0.3, 4, 8), yellow);
      arm.position.set(0.12, 0.3, s);
      arm.rotation.z = -1.0;
      torso.add(arm);
    }
    const head = new THREE.Group();
    head.position.y = 0.55;
    const skull = new THREE.Mesh(new THREE.SphereGeometry(0.1, 16, 12), yellow);
    skull.position.y = 0.1;
    head.add(skull);
    const mark = new THREE.Mesh(new THREE.CircleGeometry(0.035, 16), black);
    mark.position.set(0, 0.1, z > 0 ? 0.101 : -0.101);
    if (z < 0) mark.rotation.y = Math.PI;
    head.add(mark);
    torso.add(head);
    root.add(torso);
    root.traverse((o) => (o.castShadow = true));
    const bag = new THREE.Mesh(
      new THREE.SphereGeometry(0.3, 20, 14),
      new THREE.MeshStandardMaterial({ color: 0xf4f4f2, roughness: 0.8, transparent: true, opacity: 0.92 }),
    );
    bag.position.set(2.45, 0.98, z);
    bag.scale.setScalar(0.001);
    this.group.add(root, bag);
    return { torso, head, bag };
  }

  /** Bends the front back by the crush depth: shortened, wrinkled, hood buckled. */
  private applyCrush(crush: number): void {
    const L = this.spec.length;
    const xs = L - ZONE;
    const k = Math.max(0.05, (ZONE - crush) / ZONE);
    const amount = Math.min(1, crush / ZONE);
    for (const d of this.deform) {
      const pos = d.mesh.geometry.getAttribute('position') as THREE.BufferAttribute;
      const a = pos.array as Float32Array;
      const r = d.rest;
      for (let i = 0; i < a.length; i += 3) {
        const x0 = r[i]!,
          y0 = r[i + 1]!,
          z0 = r[i + 2]!;
        if (x0 <= xs) {
          a[i] = x0;
          a[i + 1] = y0;
          a[i + 2] = z0;
          continue;
        }
        const t = (x0 - xs) / ZONE;
        a[i] = xs + (x0 - xs) * k;
        const wrinkle =
          Math.sin(x0 * 29 + z0 * 6) * 0.045 * amount * t + Math.sin(x0 * 61 + y0 * 13) * 0.02 * amount;
        const buckle = y0 > 0.78 ? 0.32 * amount * Math.sin(Math.PI * Math.min(1, t * 1.2)) : 0;
        a[i + 1] = Math.max(0.12, y0 + wrinkle + buckle);
        a[i + 2] = z0 * (1 + 0.1 * amount * t) + Math.sin(x0 * 17 + y0 * 9) * 0.03 * amount;
      }
      pos.needsUpdate = true;
      d.mesh.geometry.computeVertexNormals();
    }
  }

  update(car: CrashCar, rear: number, crush: number, z = car.z): void {
    this.group.position.set(rear, 0, z);
    if (Math.abs(crush - this.lastCrush) > 0.002) {
      this.applyCrush(crush);
      this.lastCrush = crush;
      // Front wheels get shoved back once the crush reaches them.
      const shove = Math.max(0, crush - 0.55);
      this.wheels.forEach((w, i) => {
        if (i >= 2) w.position.x = 3.55 - shove * 0.9;
      });
      // The windscreen crazes when the front collapses into the bulkhead.
      const craze = THREE.MathUtils.smoothstep(crush, 0.35, 0.8);
      this.glass.color.setRGB(0.16 + 0.4 * craze, 0.23 + 0.38 * craze, 0.27 + 0.35 * craze);
      this.glass.opacity = 0.38 + 0.25 * craze;
    }
    const spin = rear / 0.32;
    for (const w of this.wheels) w.rotation.y = -spin;
    car.occupants.forEach((o, i) => {
      const d = this.dummies[i]!;
      d.torso.rotation.z = -Math.atan2(o.travel, 0.45);
      d.torso.position.x = Math.min(0.12, Math.max(0, o.travel * 0.3));
      d.head.rotation.z = -o.head;
      const s = o.airbag;
      d.bag.scale.set(0.6 * s + 0.001, s + 0.001, s + 0.001);
    });
  }

  dispose(): void {
    this.group.traverse((o) => {
      if (o instanceof THREE.Mesh) {
        o.geometry.dispose();
        (o.material as THREE.Material).dispose();
      }
    });
  }
}
