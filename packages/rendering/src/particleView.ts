// Draws a ParticleSim as two point clouds: alpha-blended (spray, mist, dust,
// foam) and additive (glass glints, sparks, bubbles). Soft round sprites sized
// in metres with perspective.

import * as THREE from 'three';
import { KIND_PARAMS, PARTICLE_KINDS, type ParticleSim } from './particleSim';

const VERT = /* glsl */ `
attribute vec4 aColor;
attribute float aSize;
uniform float uScale;
varying vec4 vColor;
#include <fog_pars_vertex>
void main() {
  vColor = aColor;
  vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
  gl_PointSize = clamp(aSize * uScale / max(-mvPosition.z, 0.1), 1.0, 256.0);
  gl_Position = projectionMatrix * mvPosition;
  #include <fog_vertex>
}`;

const FRAG = /* glsl */ `
varying vec4 vColor;
#include <fog_pars_fragment>
void main() {
  vec2 c = gl_PointCoord - 0.5;
  float r = length(c) * 2.0;
  float a = vColor.a * smoothstep(1.0, 0.25, r);
  if (a < 0.003) discard;
  gl_FragColor = vec4(vColor.rgb, a);
  #include <fog_fragment>
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

class Cloud {
  readonly points: THREE.Points;
  readonly position: THREE.BufferAttribute;
  readonly color: THREE.BufferAttribute;
  readonly size: THREE.BufferAttribute;

  constructor(capacity: number, additive: boolean, uniforms: Record<string, THREE.IUniform>) {
    const g = new THREE.BufferGeometry();
    this.position = new THREE.BufferAttribute(new Float32Array(capacity * 3), 3).setUsage(
      THREE.DynamicDrawUsage,
    );
    this.color = new THREE.BufferAttribute(new Float32Array(capacity * 4), 4).setUsage(
      THREE.DynamicDrawUsage,
    );
    this.size = new THREE.BufferAttribute(new Float32Array(capacity), 1).setUsage(THREE.DynamicDrawUsage);
    g.setAttribute('position', this.position);
    g.setAttribute('aColor', this.color);
    g.setAttribute('aSize', this.size);
    g.setDrawRange(0, 0);
    const m = new THREE.ShaderMaterial({
      uniforms: THREE.UniformsUtils.merge([THREE.UniformsLib.fog, uniforms]),
      vertexShader: VERT,
      fragmentShader: FRAG,
      transparent: true,
      depthWrite: false,
      fog: true,
      blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending,
    });
    m.uniforms.uScale = uniforms.uScale!;
    this.points = new THREE.Points(g, m);
    this.points.frustumCulled = false;
    this.points.renderOrder = additive ? 11 : 10;
  }
}

export class ParticleView {
  readonly group = new THREE.Group();
  private readonly uniforms = { uScale: { value: 500 } };
  private normal: Cloud;
  private additive: Cloud;
  readonly capacity: number;

  constructor(capacity: number) {
    this.capacity = capacity;
    this.normal = new Cloud(capacity, false, this.uniforms);
    this.additive = new Cloud(capacity, true, this.uniforms);
    this.group.add(this.normal.points, this.additive.points);
    this.group.name = 'particles';
  }

  /** Pixels per metre at unit distance for the current viewport. */
  setViewport(heightPx: number, fovDeg: number): void {
    this.uniforms.uScale.value = heightPx / (2 * Math.tan((fovDeg * Math.PI) / 360));
  }

  update(sim: ParticleSim): void {
    if (sim.capacity !== this.capacity) throw new Error('ParticleView capacity must match the simulation');
    const params = PARTICLE_KINDS.map((k) => KIND_PARAMS[k]);
    let nN = 0,
      nA = 0;
    const N = this.normal,
      A = this.additive;
    const np = N.position.array as Float32Array,
      nc = N.color.array as Float32Array,
      ns = N.size.array as Float32Array;
    const ap = A.position.array as Float32Array,
      ac = A.color.array as Float32Array,
      as = A.size.array as Float32Array;
    for (let i = 0; i < sim.count; i++) {
      const p = params[sim.kind[i]!]!;
      const life = sim.life[i]!;
      const t = sim.age[i]! / life;
      const fade = Math.min(1, sim.age[i]! * 8) * (1 - t * t);
      const twinkle = p.additive && p.gravity > 0 ? 0.5 + 0.5 * Math.sin(sim.age[i]! * 40 + i) : 1;
      const alpha = p.alpha * fade * twinkle;
      if (p.additive) {
        ap[nA * 3] = sim.px[i]!;
        ap[nA * 3 + 1] = sim.py[i]!;
        ap[nA * 3 + 2] = sim.pz[i]!;
        ac[nA * 4] = p.color[0];
        ac[nA * 4 + 1] = p.color[1];
        ac[nA * 4 + 2] = p.color[2];
        ac[nA * 4 + 3] = alpha;
        as[nA++] = sim.size[i]!;
      } else {
        np[nN * 3] = sim.px[i]!;
        np[nN * 3 + 1] = sim.py[i]!;
        np[nN * 3 + 2] = sim.pz[i]!;
        nc[nN * 4] = p.color[0];
        nc[nN * 4 + 1] = p.color[1];
        nc[nN * 4 + 2] = p.color[2];
        nc[nN * 4 + 3] = alpha;
        ns[nN++] = sim.size[i]!;
      }
    }
    for (const [cloud, n] of [
      [N, nN],
      [A, nA],
    ] as const) {
      cloud.points.geometry.setDrawRange(0, n);
      for (const attr of [cloud.position, cloud.color, cloud.size]) {
        attr.clearUpdateRanges();
        attr.addUpdateRange(0, n * attr.itemSize);
        attr.needsUpdate = true;
      }
    }
  }
}
