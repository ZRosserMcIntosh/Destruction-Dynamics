// The hero tower: per-section groups whose transforms follow the authored
// collapse, with instanced floor slabs and facade panels. Panels detach by
// hiding their instance; the debris system spawns the physical piece.

import type { Pose } from '@destruction-dynamics/core';
import {
  type FacadePlan,
  type TowerSection,
  type TowerSpec,
  facadePanelGeometry,
} from '@destruction-dynamics/destruction';
import * as THREE from 'three';

const _m = new THREE.Matrix4();
const _q = new THREE.Quaternion();
const _s = new THREE.Vector3();
const ZERO = new THREE.Matrix4().makeScale(0, 0, 0);

export interface PanelGeometry {
  /** Section-local centre. */
  center: THREE.Vector3;
  /** Half extents in section-local axes. */
  half: THREE.Vector3;
}

export class TowerView {
  readonly group = new THREE.Group();
  readonly sectionGroups: THREE.Group[] = [];
  private readonly panelMeshes: THREE.InstancedMesh[] = [];
  /** Per panel: owning section, instance index and section-local geometry. */
  private readonly panelSection: Uint16Array;
  private readonly panelInstance: Uint32Array;
  readonly panelGeometry: PanelGeometry[] = [];
  private readonly attached: Uint8Array;
  private readonly materials: THREE.Material[] = [];

  constructor(
    readonly spec: TowerSpec,
    readonly sections: readonly TowerSection[],
    readonly plan: FacadePlan,
  ) {
    const concrete = new THREE.MeshStandardMaterial({ color: 0xbdb7ab, roughness: 0.85 });
    const slab = new THREE.MeshStandardMaterial({ color: 0xe6e1d6, roughness: 0.7 });
    const glass = new THREE.MeshStandardMaterial({ color: 0x6f8fa6, roughness: 0.12, metalness: 0.35 });
    this.materials.push(concrete, slab, glass);
    const fh = spec.floorHeight;

    const counts = new Array<number>(sections.length).fill(0);
    for (const p of plan.panels) counts[p.section]!++;
    this.panelSection = new Uint16Array(plan.panels.length);
    this.panelInstance = new Uint32Array(plan.panels.length);
    this.attached = new Uint8Array(plan.panels.length).fill(1);

    const panelGeo = new THREE.BoxGeometry(1, 1, 1);
    for (const sec of sections) {
      const g = new THREE.Group();
      g.name = `tower-section-${sec.index}`;
      const core = new THREE.Mesh(
        new THREE.BoxGeometry(spec.width - 1.6, sec.height - 0.1, spec.depth - 1.6),
        concrete,
      );
      core.position.y = sec.height / 2;
      core.castShadow = true;
      core.receiveShadow = true;
      const slabs = new THREE.InstancedMesh(
        new THREE.BoxGeometry(spec.width + 0.4, 0.45, spec.depth + 0.4),
        slab,
        sec.floorCount,
      );
      for (let f = 0; f < sec.floorCount; f++)
        slabs.setMatrixAt(f, _m.makeTranslation(0, (f + 1) * fh - 0.23, 0));
      slabs.castShadow = true;
      const panels = new THREE.InstancedMesh(panelGeo, glass, Math.max(1, counts[sec.index]!));
      panels.castShadow = true;
      g.add(core, slabs, panels);
      if (sec.index === sections.length - 1) {
        const crown = new THREE.Mesh(new THREE.BoxGeometry(spec.width * 0.6, 8, spec.depth * 0.6), concrete);
        crown.position.y = sec.height + 4;
        crown.castShadow = true;
        g.add(crown);
      }
      this.sectionGroups.push(g);
      this.panelMeshes.push(panels);
      this.group.add(g);
    }

    const next = new Array<number>(sections.length).fill(0);
    const color = new THREE.Color();
    facadePanelGeometry(spec, sections, plan).forEach((g, i) => {
      const p = plan.panels[i]!;
      this.panelGeometry.push({
        center: new THREE.Vector3(g.center.x, g.center.y, g.center.z),
        half: new THREE.Vector3(g.halfExtents.x, g.halfExtents.y, g.halfExtents.z),
      });
      const k = next[p.section]!++;
      this.panelSection[i] = p.section;
      this.panelInstance[i] = k;
      const mesh = this.panelMeshes[p.section]!;
      mesh.setMatrixAt(k, this.panelMatrix(i));
      const lit = ((p.floor * 31 + p.column * 17 + p.face.length * 7) % 11) / 11;
      color.setRGB(0.75 + lit * 0.25, 0.85 + lit * 0.1, 0.95);
      mesh.setColorAt(k, color);
    });
    this.group.name = 'tower';
  }

  private panelMatrix(i: number): THREE.Matrix4 {
    const g = this.panelGeometry[i]!;
    return _m.compose(g.center, _q.identity(), _s.copy(g.half).multiplyScalar(2));
  }

  setSectionPoses(poses: readonly Pose[]): void {
    poses.forEach((p, k) => {
      const g = this.sectionGroups[k];
      if (!g) return;
      g.position.set(p.position.x, p.position.y, p.position.z);
      g.quaternion.set(p.quaternion.x, p.quaternion.y, p.quaternion.z, p.quaternion.w);
    });
  }

  isAttached(i: number): boolean {
    return this.attached[i] === 1;
  }

  setAttached(i: number, on: boolean): void {
    if ((this.attached[i] === 1) === on) return;
    this.attached[i] = on ? 1 : 0;
    const mesh = this.panelMeshes[this.panelSection[i]!]!;
    mesh.setMatrixAt(this.panelInstance[i]!, on ? this.panelMatrix(i) : ZERO);
    mesh.instanceMatrix.needsUpdate = true;
  }

  /** Re-derives every panel from the plan at event time t (seek, restoration). */
  syncToTime(t: number | null): number {
    let detached = 0;
    for (let i = 0; i < this.plan.panels.length; i++) {
      const on = t === null || t < this.plan.detachTime[i]!;
      this.setAttached(i, on);
      if (!on) detached++;
    }
    return detached;
  }

  /** World pose of a panel's centre given its section pose. */
  panelWorld(i: number, out: { position: THREE.Vector3; quaternion: THREE.Quaternion }): void {
    const g = this.sectionGroups[this.panelSection[i]!]!;
    g.updateMatrixWorld();
    out.position.copy(this.panelGeometry[i]!.center).applyMatrix4(g.matrixWorld);
    g.getWorldQuaternion(out.quaternion);
  }

  get panelCount(): number {
    return this.plan.panels.length;
  }

  dispose(): void {
    this.group.traverse((o) => {
      if (o instanceof THREE.Mesh) o.geometry.dispose();
    });
    for (const m of this.materials) m.dispose();
  }
}
