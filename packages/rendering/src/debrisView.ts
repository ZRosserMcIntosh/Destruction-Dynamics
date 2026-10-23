// Instanced rendering of physics debris (one draw call per debris material).

import type { Pose, Vec3 } from '@destruction-dynamics/core';
import * as THREE from 'three';

const _m = new THREE.Matrix4();
const _p = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _s = new THREE.Vector3();

export interface DebrisInstance {
  kind: string;
  pose: Pose;
  halfExtents: Vec3;
}

export class DebrisView {
  readonly group = new THREE.Group();
  private meshes = new Map<string, THREE.InstancedMesh>();
  private capacity: number;
  private readonly materials: Record<string, THREE.Material> = {
    panel: new THREE.MeshStandardMaterial({ color: 0x7f9bb0, roughness: 0.2, metalness: 0.35 }),
    chunk: new THREE.MeshStandardMaterial({ color: 0xa39d92, roughness: 0.9 }),
    prop: new THREE.MeshStandardMaterial({ color: 0xc46a2b, roughness: 0.6 }),
  };

  constructor(capacity: number) {
    this.capacity = capacity;
    this.build();
    this.group.name = 'debris';
  }

  private build(): void {
    for (const m of this.meshes.values()) {
      this.group.remove(m);
      m.dispose();
    }
    this.meshes.clear();
    const geo = new THREE.BoxGeometry(2, 2, 2);
    for (const [kind, mat] of Object.entries(this.materials)) {
      const mesh = new THREE.InstancedMesh(geo, mat, Math.max(1, this.capacity));
      mesh.count = 0;
      mesh.castShadow = true;
      mesh.frustumCulled = false;
      this.meshes.set(kind, mesh);
      this.group.add(mesh);
    }
  }

  setCapacity(n: number): void {
    if (n === this.capacity) return;
    this.capacity = n;
    this.build();
  }

  update(items: Iterable<DebrisInstance>): void {
    for (const m of this.meshes.values()) m.count = 0;
    for (const it of items) {
      const mesh = this.meshes.get(it.kind) ?? this.meshes.get('chunk')!;
      if (mesh.count >= this.capacity) continue;
      _p.set(it.pose.position.x, it.pose.position.y, it.pose.position.z);
      _q.set(it.pose.quaternion.x, it.pose.quaternion.y, it.pose.quaternion.z, it.pose.quaternion.w);
      _s.set(it.halfExtents.x, it.halfExtents.y, it.halfExtents.z);
      mesh.setMatrixAt(mesh.count++, _m.compose(_p, _q, _s));
    }
    for (const m of this.meshes.values()) m.instanceMatrix.needsUpdate = true;
  }
}
