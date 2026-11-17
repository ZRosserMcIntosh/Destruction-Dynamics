// Sky, sun and atmosphere state shared by every material that shades itself.

import * as THREE from 'three';
import { SKY_GLSL } from './shaderChunks';

export interface AtmosphereUniforms {
  uSkyTop: THREE.IUniform<THREE.Color>;
  uSkyHorizon: THREE.IUniform<THREE.Color>;
  uSunDir: THREE.IUniform<THREE.Vector3>;
  uSunColor: THREE.IUniform<THREE.Color>;
  uFogColor: THREE.IUniform<THREE.Color>;
  uFogNear: THREE.IUniform<number>;
  uFogFar: THREE.IUniform<number>;
  uFogDensity: THREE.IUniform<number>;
}

export const ABOVE_WATER_FOG = { color: new THREE.Color(0xb9cbd8), near: 1800, far: 9000 };
export const UNDERWATER_FOG = { color: new THREE.Color(0x0b3a46), density: 0.022 };

export function createAtmosphere(sunDirection: THREE.Vector3): AtmosphereUniforms {
  return {
    uSkyTop: { value: new THREE.Color(0x3d78c2) },
    uSkyHorizon: { value: new THREE.Color(0xc9dbe6) },
    uSunDir: { value: sunDirection.clone().normalize() },
    uSunColor: { value: new THREE.Color(1.0, 0.92, 0.78) },
    uFogColor: { value: ABOVE_WATER_FOG.color.clone() },
    uFogNear: { value: ABOVE_WATER_FOG.near },
    uFogFar: { value: ABOVE_WATER_FOG.far },
    uFogDensity: { value: 0 },
  };
}

/** Applies above/below-water fog to both our shaders and Three's built-in fog. */
export function setUnderwaterAtmosphere(
  a: AtmosphereUniforms,
  scene: THREE.Scene,
  underwater: boolean,
): void {
  if (underwater) {
    a.uFogColor.value.copy(UNDERWATER_FOG.color);
    a.uFogDensity.value = UNDERWATER_FOG.density;
    scene.fog = new THREE.FogExp2(UNDERWATER_FOG.color.getHex(), UNDERWATER_FOG.density);
    scene.background = UNDERWATER_FOG.color.clone();
  } else {
    a.uFogColor.value.copy(ABOVE_WATER_FOG.color);
    a.uFogDensity.value = 0;
    scene.fog = new THREE.Fog(ABOVE_WATER_FOG.color.getHex(), ABOVE_WATER_FOG.near, ABOVE_WATER_FOG.far);
    scene.background = null;
  }
}

export function createSkyDome(a: AtmosphereUniforms): THREE.Mesh {
  const material = new THREE.ShaderMaterial({
    uniforms: { ...a },
    vertexShader: /* glsl */ `
      varying vec3 vDir;
      void main() {
        vDir = normalize((modelMatrix * vec4(position, 1.0)).xyz - cameraPosition);
        vec4 p = projectionMatrix * viewMatrix * vec4((modelMatrix * vec4(position, 1.0)).xyz, 1.0);
        gl_Position = p.xyww;
      }`,
    fragmentShader: /* glsl */ `
      ${SKY_GLSL}
      varying vec3 vDir;
      void main() {
        vec3 d = normalize(vDir);
        vec3 c = ddSky(vec3(d.x, max(d.y, 0.0), d.z));
        gl_FragColor = vec4(c, 1.0);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`,
    side: THREE.BackSide,
    depthWrite: false,
  });
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 16), material);
  mesh.scale.setScalar(20000);
  mesh.frustumCulled = false;
  mesh.renderOrder = -10;
  mesh.name = 'sky';
  return mesh;
}
