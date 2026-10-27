// Draws the pieces of a FractureWorld: authored boxes as one InstancedMesh
// per material (mortar edges, per-unit colour, darkening as joints take
// damage), and run-time shards as one BatchedMesh per material.

import type { FractureMaterial, FractureWorld, Piece } from '@destruction-dynamics/destruction';
import * as THREE from 'three';
import { ConvexGeometry } from 'three/addons/geometries/ConvexGeometry.js';
import type { ImpactLab } from './lab';

const MAX_SHARDS = 2500;
const MAX_SHARD_VERTS = 2500 * 90;

const EYES_GLSL = /* glsl */ `
vec3 eyeHash(vec3 p) {
  p = vec3(dot(p, vec3(127.1, 311.7, 74.7)), dot(p, vec3(269.5, 183.3, 246.1)), dot(p, vec3(113.5, 271.9, 124.6)));
  return fract(sin(p) * 43758.5453123);
}
float cheeseEyes(vec3 p) {
  vec3 cell = floor(p), f = fract(p);
  float d = 1e9;
  for (int x = -1; x <= 1; x++)
    for (int y = -1; y <= 1; y++)
      for (int z = -1; z <= 1; z++) {
        vec3 o = vec3(float(x), float(y), float(z));
        vec3 h = eyeHash(cell + o);
        float r = h.x < 0.3 ? 0.0 : 0.2 + 0.24 * h.z;
        d = min(d, length(o + 0.2 + 0.6 * h - f) - r);
      }
  return d;
}`;

/** Patches a standard material: mortar edges from box UVs, cheese eyes, straw. */
function patch(material: THREE.MeshStandardMaterial, look: FractureMaterial['look'], edges: boolean): void {
  material.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vObj;\nvarying vec2 vBoxUv;')
      .replace(
        '#include <begin_vertex>',
        `#include <begin_vertex>
        vBoxUv = ${edges ? 'uv' : 'vec2(0.5)'};
        vec4 wp = vec4(transformed, 1.0);
        #ifdef USE_INSTANCING
        wp = instanceMatrix * wp;
        #endif
        #ifdef USE_BATCHING
        wp = batchingMatrix * wp;
        #endif
        vObj = (modelMatrix * wp).xyz;`,
      );
    let frag = `#include <color_fragment>\n`;
    if (edges && (look === 'brick' || look === 'block' || look === 'glass'))
      frag += `
        float e = min(min(vBoxUv.x, 1.0 - vBoxUv.x), min(vBoxUv.y, 1.0 - vBoxUv.y));
        float mortar = 1.0 - smoothstep(0.0, ${look === 'glass' ? '0.06' : '0.035'}, e);
        diffuseColor.rgb = mix(diffuseColor.rgb, vec3(${look === 'glass' ? '0.82, 0.84, 0.82' : '0.62, 0.6, 0.56'}), mortar * 0.85);
        ${look === 'glass' ? 'diffuseColor.a = mix(diffuseColor.a, 0.95, mortar);' : ''}`;
    if (look === 'brick' || look === 'block') {
      frag += `
        float grain = fract(sin(dot(floor(vObj * 160.0), vec3(12.9898, 78.233, 37.719))) * 43758.5453);
        diffuseColor.rgb *= 0.9 + 0.2 * grain;`;
    }
    if (look === 'hay')
      frag += `
        float straw = sin(vObj.x * 90.0 + sin(vObj.y * 37.0) * 2.0) * 0.5 + 0.5;
        float straw2 = fract(sin(dot(floor(vObj * 120.0), vec3(12.9898, 78.233, 37.719))) * 43758.5453);
        diffuseColor.rgb *= 0.78 + 0.22 * straw + 0.12 * straw2;`;
    if (look === 'cheese')
      frag += `
        float eye = cheeseEyes(vObj / 0.32);
        float inEye = 1.0 - smoothstep(-0.02, 0.02, eye);
        float depth = clamp(-eye * 2.5, 0.0, 1.0);
        vec3 cavity = mix(vec3(0.82, 0.55, 0.14), vec3(0.32, 0.17, 0.03), depth);
        diffuseColor.rgb = mix(diffuseColor.rgb, cavity, inEye);`;
    if (look === 'board')
      frag += `
        float ply = sin(vObj.y * 140.0 + sin(vObj.z * 9.0) * 3.0) * 0.5 + 0.5;
        diffuseColor.rgb *= 0.93 + 0.07 * ply;`;
    shader.fragmentShader = shader.fragmentShader
      .replace(
        '#include <common>',
        `#include <common>\nvarying vec3 vObj;\nvarying vec2 vBoxUv;\n${look === 'cheese' ? EYES_GLSL : ''}`,
      )
      .replace('#include <color_fragment>', frag);
  };
  material.customProgramCacheKey = () => `${look}-${edges}`;
}

function makeMaterial(
  mat: FractureMaterial,
  edges: boolean,
  vertexColors = false,
): THREE.MeshStandardMaterial {
  const look = mat.look;
  const glassy = look === 'glass' || look === 'ice';
  const m = new THREE.MeshStandardMaterial({
    color: vertexColors ? 0xffffff : new THREE.Color(mat.color),
    roughness: glassy ? 0.06 : look === 'cheese' ? 0.55 : look === 'melon' ? 0.45 : 0.88,
    metalness: 0,
    transparent: glassy,
    opacity: look === 'ice' ? 0.62 : glassy ? (mat.id === 'glassBlock' ? 0.55 : 0.28) : 1,
    depthWrite: !glassy,
    envMapIntensity: glassy ? 2.2 : 0.6,
    vertexColors,
    side: glassy ? THREE.DoubleSide : THREE.FrontSide,
  });
  patch(m, look, edges);
  return m;
}

interface BoxSet {
  mesh: THREE.InstancedMesh;
  index: Map<number, number>;
  base: THREE.Color[];
}

interface ShardSet {
  mesh: THREE.BatchedMesh;
  index: Map<number, number>;
}

const tmpM = new THREE.Matrix4();
const tmpP = new THREE.Vector3();
const tmpQ = new THREE.Quaternion();
const tmpS = new THREE.Vector3();
const ZERO = new THREE.Matrix4().makeScale(0, 0, 0);
const tmpC = new THREE.Color();

export class PieceRenderer {
  readonly group = new THREE.Group();
  private boxes = new Map<string, BoxSet>();
  private shards = new Map<string, ShardSet>();
  private alive: boolean[] = [];
  private placed: boolean[] = [];
  private frame = 0;
  /** Melon centres by piece id, for rind/flesh vertex colours. */
  private melonOffset = new Map<number, THREE.Vector3>();

  constructor() {
    this.group.name = 'pieces';
  }

  /** Builds instanced meshes for the world's authored box pieces. */
  build(fw: FractureWorld): void {
    this.dispose();
    const byMat = new Map<string, Piece[]>();
    for (const p of fw.graph.pieces) {
      if (p.shape.kind !== 'box') continue;
      const list = byMat.get(p.material.id) ?? [];
      list.push(p);
      byMat.set(p.material.id, list);
    }
    for (const [id, list] of byMat) {
      const mat = list[0]!.material;
      const mesh = new THREE.InstancedMesh(
        new THREE.BoxGeometry(1, 1, 1),
        makeMaterial(mat, true),
        list.length,
      );
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      const index = new Map<number, number>();
      const base: THREE.Color[] = [];
      list.forEach((p, i) => {
        index.set(p.id, i);
        // Per-unit colour variation, seeded by the piece id.
        const h = Math.sin(p.id * 12.9898) * 43758.5453;
        const r = h - Math.floor(h);
        const c = new THREE.Color(0xffffff).multiplyScalar(0.86 + 0.24 * r);
        if (mat.look === 'brick') c.offsetHSL((r - 0.5) * 0.025, 0, 0);
        base.push(c);
        mesh.setColorAt(i, c);
      });
      this.boxes.set(id, { mesh, index, base });
      this.group.add(mesh);
    }
  }

  private shardSet(mat: FractureMaterial): ShardSet {
    let s = this.shards.get(mat.id);
    if (!s) {
      const melon = mat.look === 'melon';
      const mesh = new THREE.BatchedMesh(
        MAX_SHARDS,
        MAX_SHARD_VERTS,
        undefined,
        makeMaterial(mat, false, melon),
      );
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      mesh.perObjectFrustumCulled = false;
      mesh.sortObjects = false;
      s = { mesh, index: new Map() };
      this.shards.set(mat.id, s);
      this.group.add(mesh);
    }
    return s;
  }

  /** Records where a melon's pieces sit relative to its centre (for colours). */
  registerMelon(fw: FractureWorld, pieces: number[], centre: { x: number; y: number; z: number }): void {
    for (const id of pieces) {
      const p = fw.graph.pieces[id]!.pose.position;
      this.melonOffset.set(id, new THREE.Vector3(p.x - centre.x, p.y - centre.y, p.z - centre.z));
    }
  }

  private addShard(p: Piece): void {
    if (p.shape.kind !== 'convex') return;
    const set = this.shardSet(p.material);
    if (set.index.size >= MAX_SHARDS) return;
    const pts = p.shape.points.map((q) => new THREE.Vector3(q.x, q.y, q.z));
    let geom: THREE.BufferGeometry;
    try {
      geom = new ConvexGeometry(pts);
    } catch {
      return;
    }
    if (p.material.look === 'melon') {
      const off = this.melonOffset.get(p.id) ?? new THREE.Vector3();
      const pos = geom.getAttribute('position');
      const col = new Float32Array(pos.count * 3);
      const v = new THREE.Vector3();
      for (let i = 0; i < pos.count; i++) {
        v.fromBufferAttribute(pos, i).add(off);
        const r = v.length();
        const rind = THREE.MathUtils.smoothstep(r, 0.085, 0.1);
        const stripe = Math.sin(Math.atan2(v.z, v.x) * 9) > 0.2 ? 1 : 0.7;
        const flesh = [0.86, 0.16, 0.2];
        const green = [0.12 * stripe, 0.42 * stripe, 0.14 * stripe];
        const pale = r > 0.08 && r < 0.095 ? 1 : 0;
        for (let k = 0; k < 3; k++)
          col[i * 3 + k] = pale ? [0.9, 0.95, 0.75][k]! : flesh[k]! * (1 - rind) + green[k]! * rind;
      }
      geom.setAttribute('color', new THREE.BufferAttribute(col, 3));
    }
    try {
      const gid = set.mesh.addGeometry(geom);
      const iid = set.mesh.addInstance(gid);
      set.index.set(p.id, iid);
      if (p.material.look !== 'melon') {
        const h = Math.sin(p.id * 78.233) * 43758.5453;
        set.mesh.setColorAt(iid, tmpC.setRGB(1, 1, 1).multiplyScalar(0.92 + 0.16 * (h - Math.floor(h))));
      }
    } catch {
      // Batch full: the shard simply is not drawn.
    }
    geom.dispose();
  }

  update(lab: ImpactLab, alpha: number): void {
    const fw = lab.fw;
    const pieces = fw.graph.pieces;
    const tint = this.frame++ % 12 === 0;
    for (let id = 0; id < pieces.length; id++) {
      const p = pieces[id]!;
      const wasAlive = this.alive[id];
      if (!p.alive) {
        if (wasAlive !== false) {
          this.hide(p);
          this.alive[id] = false;
        }
        continue;
      }
      if (wasAlive === undefined && p.shape.kind === 'convex') this.addShard(p);
      this.alive[id] = true;
      const loose = fw.isLoose(id);
      if (!loose && this.placed[id] && !tint) continue;
      const pose = loose ? lab.piecePose(id, alpha) : p.pose;
      tmpP.set(pose.position.x, pose.position.y, pose.position.z);
      tmpQ.set(pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w);
      if (p.shape.kind === 'box') {
        const set = this.boxes.get(p.material.id);
        const i = set?.index.get(id);
        if (!set || i === undefined) continue;
        const h = p.shape.half;
        tmpS.set(2 * h.x, 2 * h.y, 2 * h.z);
        set.mesh.setMatrixAt(i, tmpM.compose(tmpP, tmpQ, tmpS));
        set.mesh.instanceMatrix.needsUpdate = true;
        if (tint && Number.isFinite(p.material.bondStrength)) {
          // Darken with the worst joint damage: cracks in the mortar.
          let worst = 0;
          for (const bi of p.bonds) {
            const b = fw.graph.bonds[bi]!;
            worst = Math.max(worst, b.broken ? 1 : b.damage / b.capacity);
          }
          tmpC.copy(set.base[i]!).multiplyScalar(1 - 0.38 * Math.min(1, worst));
          set.mesh.setColorAt(i, tmpC);
          if (set.mesh.instanceColor) set.mesh.instanceColor.needsUpdate = true;
        }
      } else {
        const set = this.shards.get(p.material.id);
        const i = set?.index.get(id);
        if (!set || i === undefined) continue;
        tmpS.set(1, 1, 1);
        set.mesh.setMatrixAt(i, tmpM.compose(tmpP, tmpQ, tmpS));
      }
      this.placed[id] = true;
    }
  }

  private hide(p: Piece): void {
    if (p.shape.kind === 'box') {
      const set = this.boxes.get(p.material.id);
      const i = set?.index.get(p.id);
      if (set && i !== undefined) {
        set.mesh.setMatrixAt(i, ZERO);
        set.mesh.instanceMatrix.needsUpdate = true;
      }
    } else {
      const set = this.shards.get(p.material.id);
      const i = set?.index.get(p.id);
      if (set && i !== undefined) set.mesh.setVisibleAt(i, false);
    }
  }

  dispose(): void {
    for (const s of this.boxes.values()) {
      s.mesh.geometry.dispose();
      (s.mesh.material as THREE.Material).dispose();
      s.mesh.dispose();
    }
    for (const s of this.shards.values()) {
      (s.mesh.material as THREE.Material).dispose();
      s.mesh.dispose();
    }
    this.group.clear();
    this.boxes.clear();
    this.shards.clear();
    this.alive = [];
    this.placed = [];
    this.melonOffset.clear();
  }

  get shardCount(): number {
    let n = 0;
    for (const s of this.shards.values()) n += s.index.size;
    return n;
  }
}
