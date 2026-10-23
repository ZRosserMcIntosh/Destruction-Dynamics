// Immediate-mode wireframe boxes for physics, collision and water debugging.

import type { Pose, Vec3 } from '@destruction-dynamics/core';
import * as THREE from 'three';

const CORNERS = [
  [-1, -1, -1],
  [1, -1, -1],
  [1, 1, -1],
  [-1, 1, -1],
  [-1, -1, 1],
  [1, -1, 1],
  [1, 1, 1],
  [-1, 1, 1],
] as const;
const EDGES = [0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7];

const _v = new THREE.Vector3();
const _q = new THREE.Quaternion();

export class DebugDraw {
  readonly lines: THREE.LineSegments;
  private readonly pos: Float32Array;
  private readonly col: Float32Array;
  private n = 0;
  private readonly maxBoxes: number;

  constructor(maxBoxes = 4096) {
    this.maxBoxes = maxBoxes;
    this.pos = new Float32Array(maxBoxes * 24 * 3);
    this.col = new Float32Array(maxBoxes * 24 * 3);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(this.col, 3).setUsage(THREE.DynamicDrawUsage));
    this.lines = new THREE.LineSegments(
      g,
      new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, transparent: true }),
    );
    this.lines.frustumCulled = false;
    this.lines.renderOrder = 999;
    this.lines.name = 'debug-draw';
  }

  begin(): void {
    this.n = 0;
  }

  box(pose: Pose, half: Vec3, color: THREE.ColorRepresentation): void {
    if (this.n >= this.maxBoxes) return;
    const c = new THREE.Color(color);
    _q.set(pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w);
    const base = this.n * 24 * 3;
    EDGES.forEach((e, k) => {
      const cr = CORNERS[e]!;
      _v.set(cr[0] * half.x, cr[1] * half.y, cr[2] * half.z).applyQuaternion(_q);
      this.pos[base + k * 3] = _v.x + pose.position.x;
      this.pos[base + k * 3 + 1] = _v.y + pose.position.y;
      this.pos[base + k * 3 + 2] = _v.z + pose.position.z;
      this.col[base + k * 3] = c.r;
      this.col[base + k * 3 + 1] = c.g;
      this.col[base + k * 3 + 2] = c.b;
    });
    this.n++;
  }

  end(): void {
    const g = this.lines.geometry;
    g.setDrawRange(0, this.n * 24);
    g.attributes.position!.needsUpdate = true;
    g.attributes.color!.needsUpdate = true;
  }

  get boxes(): number {
    return this.n;
  }
}
