// Underwater presentation: a screen-space tint with drifting light shafts and
// a vignette, plus suspended sediment that wraps around the camera entirely in
// the vertex shader (no CPU cost per particle).

import { SeededRandom } from '@destruction-dynamics/core';
import * as THREE from 'three';
import { NOISE_GLSL } from './shaderChunks';

export class UnderwaterView {
  readonly overlay: THREE.Mesh;
  readonly sediment: THREE.Points;
  private readonly uniforms = {
    uTime: { value: 0 },
    uDepth: { value: 0 },
    uCam: { value: new THREE.Vector3() },
    uSettle: { value: 0 },
  };

  constructor(sedimentCount: number) {
    this.overlay = new THREE.Mesh(
      new THREE.PlaneGeometry(2, 2),
      new THREE.ShaderMaterial({
        uniforms: this.uniforms,
        vertexShader: /* glsl */ `
          varying vec2 vUv;
          void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }`,
        fragmentShader: /* glsl */ `
          ${NOISE_GLSL}
          uniform float uTime;
          uniform float uDepth;
          varying vec2 vUv;
          void main() {
            float depthFade = clamp(uDepth / 40.0, 0.0, 1.0);
            vec3 top = mix(vec3(0.15, 0.55, 0.6), vec3(0.04, 0.2, 0.27), depthFade);
            vec3 bottom = vec3(0.01, 0.07, 0.1);
            vec3 c = mix(bottom, top, vUv.y);
            float shafts = ddFbm(vec2(vUv.x * 7.0 + vUv.y * 1.5 + uTime * 0.05, uTime * 0.1));
            shafts = pow(shafts, 3.0) * smoothstep(0.25, 1.0, vUv.y) * (1.0 - depthFade);
            float vig = smoothstep(0.95, 0.35, length(vUv - 0.5));
            float a = 0.28 + 0.35 * (1.0 - vig);
            gl_FragColor = vec4(c + shafts * 0.5, a);
          }`,
        transparent: true,
        depthTest: false,
        depthWrite: false,
      }),
    );
    this.overlay.frustumCulled = false;
    this.overlay.renderOrder = 1000;
    this.overlay.visible = false;
    this.overlay.name = 'underwater-overlay';

    const r = new SeededRandom(5);
    const base = new Float32Array(sedimentCount * 3);
    for (let i = 0; i < base.length; i++) base[i] = r.range(0, 40);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(base, 3));
    this.sediment = new THREE.Points(
      g,
      new THREE.ShaderMaterial({
        uniforms: this.uniforms,
        vertexShader: /* glsl */ `
          uniform vec3 uCam;
          uniform float uTime;
          varying float vA;
          void main() {
            vec3 drift = vec3(sin(uTime * 0.1 + position.y) * 0.6, -uTime * 0.05, cos(uTime * 0.13 + position.x) * 0.6);
            vec3 p = uCam + mod(position + drift - uCam, 40.0) - 20.0;
            vec4 mv = modelViewMatrix * vec4(p, 1.0);
            vA = smoothstep(20.0, 4.0, -mv.z);
            gl_PointSize = clamp(60.0 / max(-mv.z, 0.5), 1.0, 6.0);
            gl_Position = projectionMatrix * mv;
          }`,
        fragmentShader: /* glsl */ `
          uniform float uSettle;
          varying float vA;
          void main() {
            float r = length(gl_PointCoord - 0.5) * 2.0;
            gl_FragColor = vec4(vec3(0.75, 0.8, 0.7), vA * (0.25 + 0.5 * uSettle) * smoothstep(1.0, 0.0, r));
          }`,
        transparent: true,
        depthWrite: false,
      }),
    );
    this.sediment.frustumCulled = false;
    this.sediment.visible = false;
    this.sediment.name = 'sediment';
  }

  /** `turbidity` 0..1: how much material is suspended (high right after the impact). */
  update(camera: THREE.Camera, time: number, depthBelowSurface: number, turbidity: number): void {
    const under = depthBelowSurface > 0;
    this.overlay.visible = under;
    this.sediment.visible = under;
    this.uniforms.uTime.value = time;
    this.uniforms.uDepth.value = Math.max(0, depthBelowSurface);
    this.uniforms.uCam.value.copy(camera.position);
    this.uniforms.uSettle.value = turbidity;
  }

  setSedimentCount(n: number): void {
    const g = this.sediment.geometry;
    const r = new SeededRandom(5);
    const base = new Float32Array(n * 3);
    for (let i = 0; i < base.length; i++) base[i] = r.range(0, 40);
    g.setAttribute('position', new THREE.BufferAttribute(base, 3));
  }
}
