// Gallery props: the air cannon, the steel target frame, and the projectiles
// (tennis ball, golf ball, ball bearing, bowling ball, frozen chicken). The
// watermelon is breakable and drawn by the piece renderer.

import * as THREE from 'three';
import { CANNON_X, CHICKEN_POINTS, type ProjectileId, TARGET_CENTRE, type TargetSpec } from './lab';
import { ConvexGeometry } from 'three/addons/geometries/ConvexGeometry.js';

const steel = () => new THREE.MeshStandardMaterial({ color: 0x5d646b, roughness: 0.45, metalness: 0.8 });

export function createCannon(): THREE.Group {
  const g = new THREE.Group();
  const s = steel();
  const tube = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.17, 1.6, 28, 1, true), s);
  tube.rotation.z = -Math.PI / 2;
  tube.position.set(CANNON_X - 0.1, TARGET_CENTRE.y, 0);
  const tank = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.24, 0.7, 6, 18),
    new THREE.MeshStandardMaterial({ color: 0xc23b2c, roughness: 0.35, metalness: 0.3 }),
  );
  tank.rotation.z = Math.PI / 2;
  tank.position.set(CANNON_X - 1.25, TARGET_CENTRE.y - 0.05, 0);
  const muzzle = new THREE.Mesh(new THREE.TorusGeometry(0.16, 0.03, 8, 28), s);
  muzzle.rotation.y = Math.PI / 2;
  muzzle.position.set(CANNON_X + 0.7, TARGET_CENTRE.y, 0);
  g.add(tube, tank, muzzle);
  for (const [x, z] of [
    [-1.6, -0.5],
    [-1.6, 0.5],
    [-0.2, 0],
  ] as const) {
    const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, TARGET_CENTRE.y + 0.1, 8), s);
    leg.position.set(CANNON_X + x, (TARGET_CENTRE.y - 0.05) / 2, z);
    g.add(leg);
  }
  g.traverse((o) => (o.castShadow = true));
  g.name = 'cannon';
  return g;
}

/** A steel frame around a target (plus legs down to the base beam). */
export function createFrame(t: TargetSpec): THREE.Group {
  const g = new THREE.Group();
  const s = steel();
  const c = TARGET_CENTRE;
  const w = t.size.width,
    h = t.size.height,
    d = Math.max(0.08, t.size.thickness + 0.04);
  const bar = 0.06;
  const add = (sx: number, sy: number, sz: number, x: number, y: number, z: number) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), s);
    m.position.set(x, y, z);
    m.castShadow = true;
    m.receiveShadow = true;
    g.add(m);
  };
  add(d, bar, w + 2 * bar, c.x, c.y + h / 2 + bar / 2, c.z);
  add(d, bar, w + 2 * bar, c.x, c.y - h / 2 - bar / 2, c.z);
  add(d, h, bar, c.x, c.y, c.z - w / 2 - bar / 2);
  add(d, h, bar, c.x, c.y, c.z + w / 2 + bar / 2);
  for (const z of [-1, 1]) add(0.08, c.y - h / 2, 0.08, c.x, (c.y - h / 2) / 2, c.z + z * (w / 2 + bar / 2));
  add(0.6, 0.1, 2, 0, 0.05, 0);
  g.name = 'frame';
  return g;
}

export function createProjectileMesh(id: ProjectileId): THREE.Object3D {
  switch (id) {
    case 'tennis': {
      const m = new THREE.Mesh(
        new THREE.SphereGeometry(0.0335, 24, 16),
        new THREE.MeshStandardMaterial({ color: 0xd4e83a, roughness: 0.95 }),
      );
      const seam = new THREE.Mesh(
        new THREE.TorusGeometry(0.0336, 0.0025, 6, 40),
        new THREE.MeshStandardMaterial({ color: 0xf6f6ee, roughness: 0.9 }),
      );
      seam.rotation.x = 0.6;
      m.add(seam);
      return m;
    }
    case 'golf':
      return new THREE.Mesh(
        new THREE.IcosahedronGeometry(0.0214, 3),
        new THREE.MeshStandardMaterial({ color: 0xf7f7f4, roughness: 0.4, flatShading: true }),
      );
    case 'bearing':
      return new THREE.Mesh(
        new THREE.SphereGeometry(0.025, 32, 20),
        new THREE.MeshStandardMaterial({ color: 0xe8ebee, roughness: 0.08, metalness: 1 }),
      );
    case 'bowling': {
      const m = new THREE.Mesh(
        new THREE.SphereGeometry(0.109, 40, 28),
        new THREE.MeshPhysicalMaterial({
          color: 0x1e3f9c,
          roughness: 0.25,
          clearcoat: 1,
          clearcoatRoughness: 0.05,
        }),
      );
      const hole = new THREE.MeshStandardMaterial({ color: 0x050608, roughness: 1 });
      for (const [a, b] of [
        [0, 0],
        [0.35, 0.25],
        [0.35, -0.25],
      ] as const) {
        const h = new THREE.Mesh(new THREE.CircleGeometry(0.012, 12), hole);
        const n = new THREE.Vector3(Math.cos(a) * Math.cos(b), Math.sin(a), Math.cos(a) * Math.sin(b));
        h.position.copy(n.clone().multiplyScalar(0.1095));
        h.lookAt(n.multiplyScalar(1));
        m.add(h);
      }
      return m;
    }
    case 'chicken': {
      const skin = new THREE.MeshPhysicalMaterial({
        color: 0xf0d2b8,
        roughness: 0.55,
        clearcoat: 0.6,
        clearcoatRoughness: 0.5,
      });
      const body = new THREE.Mesh(
        new ConvexGeometry(CHICKEN_POINTS.map((p) => new THREE.Vector3(p.x, p.y, p.z))),
        skin,
      );
      for (const s of [-1, 1]) {
        const leg = new THREE.Mesh(new THREE.CapsuleGeometry(0.022, 0.07, 4, 8), skin);
        leg.position.set(-0.1, 0.03, s * 0.045);
        leg.rotation.z = -1.2;
        const knob = new THREE.Mesh(
          new THREE.SphereGeometry(0.014, 8, 6),
          new THREE.MeshStandardMaterial({ color: 0xf8efe4 }),
        );
        knob.position.y = 0.055;
        leg.add(knob);
        body.add(leg);
      }
      return body;
    }
    default:
      return new THREE.Group();
  }
}
