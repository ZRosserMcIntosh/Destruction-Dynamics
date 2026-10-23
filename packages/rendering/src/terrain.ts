// Ground: city block, promenade, beach and seabed as one heightfield mesh,
// plus animated caustics on everything below the water line.

import * as THREE from 'three';
import { NOISE_GLSL } from './shaderChunks';

export interface TerrainOptions {
  groundHeight: (x: number, z: number) => number;
  minX: number;
  maxX: number;
  minZ: number;
  maxZ: number;
  segmentsX: number;
  segmentsZ: number;
  /** Colour by x band, west to east. */
  shoreX: number;
  roadMinX: number;
  roadMaxX: number;
}

export interface CausticsUniforms {
  uTime: THREE.IUniform<number>;
  uCaustics: THREE.IUniform<number>;
  uWaterLevel: THREE.IUniform<number>;
}

/** Adds depth-faded animated caustics below the water line to a standard material. */
export function addCaustics(material: THREE.MeshStandardMaterial, u: CausticsUniforms): void {
  material.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, u);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vDdWorld;')
      .replace(
        '#include <worldpos_vertex>',
        '#include <worldpos_vertex>\nvDdWorld = (modelMatrix * vec4(transformed, 1.0)).xyz;',
      );
    shader.fragmentShader = shader.fragmentShader
      .replace(
        '#include <common>',
        `#include <common>\nvarying vec3 vDdWorld;\nuniform float uTime;\nuniform float uCaustics;\nuniform float uWaterLevel;\n${NOISE_GLSL}`,
      )
      .replace(
        '#include <emissivemap_fragment>',
        `#include <emissivemap_fragment>
        float ddDepth = uWaterLevel - vDdWorld.y;
        if (ddDepth > 0.0) {
          diffuseColor.rgb *= mix(vec3(1.0), vec3(0.45, 0.75, 0.8), clamp(ddDepth / 25.0, 0.0, 1.0));
          totalEmissiveRadiance += uCaustics * vec3(0.55, 0.8, 0.85) * ddCaustic(vDdWorld.xz, uTime) * exp(-ddDepth / 14.0) * 0.35;
        }`,
      );
  };
}

export function createTerrain(o: TerrainOptions, caustics: CausticsUniforms): THREE.Mesh {
  const g = new THREE.PlaneGeometry(o.maxX - o.minX, o.maxZ - o.minZ, o.segmentsX, o.segmentsZ);
  g.rotateX(-Math.PI / 2);
  g.translate((o.minX + o.maxX) / 2, 0, (o.minZ + o.maxZ) / 2);
  const pos = g.attributes.position as THREE.BufferAttribute;
  const colors = new Float32Array(pos.count * 3);
  const c = new THREE.Color();
  const city = new THREE.Color(0x6f7468);
  const sand = new THREE.Color(0xe2cf9e);
  const wet = new THREE.Color(0xb59f72);
  const seabed = new THREE.Color(0xc2ab7c);
  const deep = new THREE.Color(0x5c6f6a);
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const z = pos.getZ(i);
    const y = o.groundHeight(x, z);
    pos.setY(i, y);
    if (x < o.roadMinX) c.copy(city);
    else if (x < o.roadMaxX) c.set(0x3d3f42);
    else if (x < o.shoreX - 12) c.copy(sand);
    else if (x < o.shoreX + 4) c.copy(wet);
    else c.copy(seabed).lerp(deep, Math.min(1, -y / 40));
    const n = Math.sin(x * 0.37) * Math.sin(z * 0.29) * 0.04;
    colors[i * 3] = c.r + n;
    colors[i * 3 + 1] = c.g + n;
    colors[i * 3 + 2] = c.b + n;
  }
  g.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  g.computeVertexNormals();
  const m = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.95, metalness: 0 });
  addCaustics(m, caustics);
  const mesh = new THREE.Mesh(g, m);
  mesh.receiveShadow = true;
  mesh.name = 'terrain';
  return mesh;
}
