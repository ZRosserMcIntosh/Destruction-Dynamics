// Ocean rendering: a camera-following geometric grid for the far field and a
// uniform patch over the near-field solver domain. Both apply the same
// Gerstner waves on the GPU; the patch adds the solver/surge heightfield and
// foam from a float texture updated each frame.

import type { HybridOcean } from '@destruction-dynamics/fluids';
import { MAX_GERSTNER_WAVES } from '@destruction-dynamics/fluids';
import * as THREE from 'three';
import type { AtmosphereUniforms } from './environment';
import { FOG_GLSL, NOISE_GLSL, SKY_GLSL } from './shaderChunks';

const VERT = /* glsl */ `
uniform float uTime;
uniform vec4 uWaveA[${MAX_GERSTNER_WAVES}];
uniform vec4 uWaveB[${MAX_GERSTNER_WAVES}];
uniform int uWaveCount;
uniform sampler2D uNear;
uniform vec4 uNearRect;
uniform vec2 uWaveFade;
varying vec3 vWorld;
varying vec3 vNormal;
varying float vFoam;
varying float vNearHeight;
void main() {
  vec3 p = (modelMatrix * vec4(position, 1.0)).xyz;
  vec2 x0 = p.xz;
  float fade = 1.0 - smoothstep(uWaveFade.x, uWaveFade.y, length(x0 - cameraPosition.xz));
  vec3 disp = vec3(0.0);
  vec3 n = vec3(0.0, 1.0, 0.0);
  for (int i = 0; i < ${MAX_GERSTNER_WAVES}; i++) {
    if (i >= uWaveCount) break;
    vec4 a = uWaveA[i];
    vec4 b = uWaveB[i];
    float amp = a.z * fade;
    float th = b.x * dot(a.xy, x0) - b.y * uTime + a.w;
    float c = cos(th), s = sin(th);
    disp.x += b.z * amp * a.x * c;
    disp.z += b.z * amp * a.y * c;
    disp.y += amp * s;
    n.x -= a.x * b.x * amp * c;
    n.z -= a.y * b.x * amp * c;
    n.y -= b.z * b.x * amp * s;
  }
  vec2 uv = (x0 - uNearRect.xy) / uNearRect.z;
  float h = 0.0;
  vFoam = 0.0;
  if (uv.x > 0.0 && uv.y > 0.0 && uv.x < 1.0 && uv.y < 1.0) {
    vec2 hf = texture2D(uNear, uv).rg;
    h = hf.r;
    vFoam = hf.g;
    float t = uNearRect.w;
    float cell = uNearRect.z * t;
    float hx = texture2D(uNear, uv + vec2(t, 0.0)).r - texture2D(uNear, uv - vec2(t, 0.0)).r;
    float hz = texture2D(uNear, uv + vec2(0.0, t)).r - texture2D(uNear, uv - vec2(0.0, t)).r;
    n.x -= hx / (2.0 * cell);
    n.z -= hz / (2.0 * cell);
  }
#ifdef FAR_MESH
  // Under the near-field patch the far mesh follows the same heightfield a
  // little lower, so any crack between the two shows water, not sky.
  float m = uNearRect.w * 1.5;
  if (uv.x > m && uv.y > m && uv.x < 1.0 - m && uv.y < 1.0 - m) disp.y -= 1.5;
#endif
  p += disp;
  p.y += h;
  vNearHeight = h;
  vWorld = p;
  vNormal = normalize(n);
  gl_Position = projectionMatrix * viewMatrix * vec4(p, 1.0);
}`;

const FRAG = /* glsl */ `
${NOISE_GLSL}
${SKY_GLSL}
${FOG_GLSL}
uniform float uTime;
uniform vec3 uDeep;
uniform vec3 uShallow;
uniform float uDebug;
varying vec3 vWorld;
varying vec3 vNormal;
varying float vFoam;
varying float vNearHeight;
void main() {
  vec3 N = normalize(vNormal);
  vec3 V = normalize(cameraPosition - vWorld);
  float dist = length(cameraPosition - vWorld);
  vec3 col;
  if (gl_FrontFacing) {
    float ndv = max(dot(N, V), 0.0);
    float fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);
    vec3 R = reflect(-V, N);
    R.y = abs(R.y);
    vec3 refl = ddSky(R);
    float crest = clamp(0.35 + vWorld.y * 0.05 + (1.0 - N.y) * 1.5, 0.0, 1.0);
    vec3 body = mix(uDeep, uShallow, crest);
    // Light scattered through thin crests facing the sun.
    body += uShallow * 0.6 * pow(max(dot(-V, uSunDir), 0.0), 4.0) * clamp(vWorld.y * 0.15, 0.0, 1.0);
    col = mix(body, refl, fres);
    vec3 H = normalize(uSunDir + V);
    float nh = max(dot(N, H), 0.0);
    col += uSunColor * (pow(nh, 900.0) * 6.0 + pow(nh, 80.0) * 0.12);
    float pattern = ddFbm(vWorld.xz * 0.12 + vec2(uTime * 0.03, uTime * 0.02));
    float steep = smoothstep(0.35, 0.7, 1.0 - N.y);
    float foam = clamp(vFoam * 1.4 + steep * 0.6, 0.0, 1.0);
    foam = smoothstep(0.25, 0.75, foam * (0.55 + 0.9 * pattern));
    col = mix(col, vec3(0.93, 0.96, 0.98), foam);
  } else {
    // Seen from below: Snell's window (critical angle ≈ 48.6°) inside total internal reflection.
    float cosv = clamp(dot(V, -N), 0.0, 1.0);
    float window = smoothstep(0.6, 0.72, cosv);
    vec3 above = ddSky(refract(-V, -N, 1.33)) * 0.85 + vec3(0.05, 0.15, 0.15);
    col = mix(uFogColor * 1.4, above, window);
    col = mix(col, vec3(0.9), clamp(vFoam, 0.0, 1.0) * 0.5);
  }
  col = mix(col, uFogColor, ddFog(dist));
  if (uDebug > 0.5) {
    float h = vNearHeight;
    vec3 heat = h > 0.0 ? mix(vec3(0.1, 0.5, 0.2), vec3(1.0, 0.2, 0.1), clamp(h / 6.0, 0.0, 1.0))
                        : mix(vec3(0.1, 0.5, 0.2), vec3(0.1, 0.2, 1.0), clamp(-h / 6.0, 0.0, 1.0));
    col = mix(col, heat, 0.75);
  }
  gl_FragColor = vec4(col, 1.0);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

/** Grid coordinates densest at the centre: u_i grows geometrically to `extent`. */
function geometricAxis(rings: number, first: number, extent: number): number[] {
  // Solve first * ((1+g)^rings - 1) / g = extent for g by bisection.
  let lo = 1e-6,
    hi = 1;
  for (let it = 0; it < 80; it++) {
    const g = (lo + hi) / 2;
    if ((first * (Math.pow(1 + g, rings) - 1)) / g > extent) hi = g;
    else lo = g;
  }
  const g = (lo + hi) / 2;
  const half: number[] = [];
  for (let i = 0; i <= rings; i++) half.push((first * (Math.pow(1 + g, i) - 1)) / g);
  return [
    ...half
      .slice(1)
      .reverse()
      .map((v) => -v),
    ...half,
  ];
}

export function createGeometricGrid(rings: number, first: number, extent: number): THREE.BufferGeometry {
  const axis = geometricAxis(rings, first, extent);
  const n = axis.length;
  const pos = new Float32Array(n * n * 3);
  for (let j = 0; j < n; j++)
    for (let i = 0; i < n; i++) {
      const k = (j * n + i) * 3;
      pos[k] = axis[i]!;
      pos[k + 2] = axis[j]!;
    }
  const idx: number[] = [];
  for (let j = 0; j < n - 1; j++)
    for (let i = 0; i < n - 1; i++) {
      const a = j * n + i;
      idx.push(a, a + n, a + 1, a + 1, a + n, a + n + 1);
    }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.boundingSphere = new THREE.Sphere(new THREE.Vector3(), extent * 1.5);
  return g;
}

export interface OceanViewOptions {
  ocean: HybridOcean;
  atmosphere: AtmosphereUniforms;
  gridResolution: number;
  floatLinear: boolean;
  farExtent?: number;
}

export class OceanView {
  readonly group = new THREE.Group();
  private readonly uniforms: Record<string, THREE.IUniform>;
  private readonly farMaterial: THREE.ShaderMaterial;
  private readonly nearMaterial: THREE.ShaderMaterial;
  private far: THREE.Mesh;
  private near: THREE.Mesh;
  private texture: THREE.DataTexture;
  private nearSize = 0;
  private gridRings: number;

  constructor(private readonly opts: OceanViewOptions) {
    const waves = opts.ocean.far.waves;
    const consts = opts.ocean.far.constants;
    const waveA = Array.from({ length: MAX_GERSTNER_WAVES }, (_, i) => {
      const w = waves[i];
      return w ? new THREE.Vector4(w.dirX, w.dirZ, w.amplitude, w.phase) : new THREE.Vector4();
    });
    const waveB = Array.from({ length: MAX_GERSTNER_WAVES }, (_, i) => {
      const c = consts[i];
      return c ? new THREE.Vector4(c.k, c.omega, c.q, 0) : new THREE.Vector4();
    });
    this.texture = this.makeTexture();
    this.uniforms = {
      ...opts.atmosphere,
      uTime: { value: 0 },
      uWaveA: { value: waveA },
      uWaveB: { value: waveB },
      uWaveCount: { value: waves.length },
      uNear: { value: this.texture },
      uNearRect: { value: this.nearRect() },
      uWaveFade: { value: new THREE.Vector2(1500, 5000) },
      uDeep: { value: new THREE.Color(0x06303f) },
      uShallow: { value: new THREE.Color(0x1f8c8a) },
      uDebug: { value: 0 },
    };
    const common = {
      uniforms: this.uniforms,
      vertexShader: VERT,
      fragmentShader: FRAG,
      side: THREE.DoubleSide,
    };
    this.farMaterial = new THREE.ShaderMaterial({ ...common, defines: { FAR_MESH: 1 } });
    this.nearMaterial = new THREE.ShaderMaterial(common);
    this.gridRings = opts.gridResolution;
    this.far = new THREE.Mesh(
      createGeometricGrid(this.gridRings, 1.5, opts.farExtent ?? 9000),
      this.farMaterial,
    );
    this.far.frustumCulled = false;
    this.far.name = 'ocean-far';
    this.near = this.makeNearPatch();
    this.group.add(this.far, this.near);
    this.group.name = 'ocean';
  }

  private nearRect(): THREE.Vector4 {
    const nf = this.opts.ocean.near;
    return new THREE.Vector4(nf.originX, nf.originZ, nf.extent, 1 / nf.n);
  }

  private makeTexture(): THREE.DataTexture {
    const o = this.opts.ocean;
    this.nearSize = o.near.n;
    const t = new THREE.DataTexture(o.composite, o.near.n, o.near.n, THREE.RGFormat, THREE.FloatType);
    const filter = this.opts.floatLinear ? THREE.LinearFilter : THREE.NearestFilter;
    t.magFilter = filter;
    t.minFilter = filter;
    t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
    t.needsUpdate = true;
    return t;
  }

  private makeNearPatch(): THREE.Mesh {
    const nf = this.opts.ocean.near;
    const segs = Math.min(nf.n, 384);
    const g = new THREE.PlaneGeometry(nf.extent, nf.extent, segs, segs);
    g.rotateX(-Math.PI / 2);
    const m = new THREE.Mesh(g, this.nearMaterial);
    m.position.set(nf.centerX, 0, nf.centerZ);
    m.frustumCulled = false;
    m.name = 'ocean-near';
    return m;
  }

  get gridResolution(): number {
    return this.gridRings;
  }

  /** Vertices drawn by both ocean meshes (instrumentation). */
  get vertexCount(): number {
    return this.far.geometry.attributes.position!.count + this.near.geometry.attributes.position!.count;
  }

  setGridResolution(rings: number): void {
    if (rings === this.gridRings) return;
    this.gridRings = rings;
    this.far.geometry.dispose();
    this.far.geometry = createGeometricGrid(rings, 1.5, this.opts.farExtent ?? 9000);
  }

  setDebug(on: boolean): void {
    this.uniforms.uDebug!.value = on ? 1 : 0;
    this.nearMaterial.wireframe = on;
  }

  /** Call after the fluid near-field has been resized. */
  syncNearField(): void {
    this.texture.dispose();
    this.texture = this.makeTexture();
    this.uniforms.uNear!.value = this.texture;
    this.uniforms.uNearRect!.value = this.nearRect();
    this.group.remove(this.near);
    this.near.geometry.dispose();
    this.near = this.makeNearPatch();
    this.group.add(this.near);
  }

  update(camera: THREE.Camera): void {
    const o = this.opts.ocean;
    if (o.near.n !== this.nearSize || this.texture.image.data !== o.composite) this.syncNearField();
    this.texture.needsUpdate = true;
    this.uniforms.uTime!.value = o.time;
    // Follow the camera in coarse steps so the far grid does not swim.
    const step = 24;
    this.far.position.set(
      Math.round(camera.position.x / step) * step,
      0,
      Math.round(camera.position.z / step) * step,
    );
  }

  dispose(): void {
    this.texture.dispose();
    this.far.geometry.dispose();
    this.near.geometry.dispose();
    this.farMaterial.dispose();
    this.nearMaterial.dispose();
  }
}
