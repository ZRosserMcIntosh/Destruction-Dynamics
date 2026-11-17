// Floating props (boats, buoys, cargo crates), instanced per type.

import type { Pose } from '@destruction-dynamics/core';
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

export type PropKind = 'boat' | 'buoy' | 'crate';

export interface PropInstance {
  kind: PropKind;
  pose: Pose;
}

function boatGeometry(): THREE.BufferGeometry {
  const hull = new THREE.BoxGeometry(7, 1.6, 2.6);
  const cabin = new THREE.BoxGeometry(2.6, 1.4, 2);
  cabin.translate(-0.8, 1.5, 0);
  return mergeGeometries([hull, cabin])!;
}

export const PROP_HALF_EXTENTS: Record<PropKind, { x: number; y: number; z: number }> = {
  boat: { x: 3.5, y: 0.8, z: 1.3 },
  buoy: { x: 0.7, y: 1.2, z: 0.7 },
  crate: { x: 1, y: 1, z: 1 },
};

export class PropsView {
  readonly group = new THREE.Group();
  private readonly meshes: Record<PropKind, THREE.InstancedMesh>;

  constructor(capacity: number) {
    this.meshes = {
      boat: new THREE.InstancedMesh(
        boatGeometry(),
        new THREE.MeshStandardMaterial({ color: 0xf2f0ea, roughness: 0.5 }),
        capacity,
      ),
      buoy: new THREE.InstancedMesh(
        new THREE.CylinderGeometry(0.7, 0.7, 2.4, 12),
        new THREE.MeshStandardMaterial({ color: 0xd9452b, roughness: 0.6 }),
        capacity,
      ),
      crate: new THREE.InstancedMesh(
        new THREE.BoxGeometry(2, 2, 2),
        new THREE.MeshStandardMaterial({ color: 0x9a6b3c, roughness: 0.85 }),
        capacity,
      ),
    };
    for (const m of Object.values(this.meshes)) {
      m.count = 0;
      m.castShadow = true;
      m.frustumCulled = false;
      this.group.add(m);
    }
    this.group.name = 'props';
  }

  update(items: readonly PropInstance[]): void {
    const m = new THREE.Matrix4();
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const one = new THREE.Vector3(1, 1, 1);
    for (const mesh of Object.values(this.meshes)) mesh.count = 0;
    for (const it of items) {
      const mesh = this.meshes[it.kind];
      if (mesh.count >= mesh.instanceMatrix.count) continue;
      p.set(it.pose.position.x, it.pose.position.y, it.pose.position.z);
      q.set(it.pose.quaternion.x, it.pose.quaternion.y, it.pose.quaternion.z, it.pose.quaternion.w);
      mesh.setMatrixAt(mesh.count++, m.compose(p, q, one));
    }
    for (const mesh of Object.values(this.meshes)) mesh.instanceMatrix.needsUpdate = true;
  }
}
