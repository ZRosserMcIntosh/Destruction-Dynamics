// Background city: instanced towers along the beachfront with procedural
// windows shaded from world position (no textures, no assets).

import { SeededRandom } from '@destruction-dynamics/core';
import * as THREE from 'three';

export interface CityOptions {
  seed: number;
  density: number;
  /** Keep this rectangle (x/z) free for the hero tower. */
  exclude: { minX: number; maxX: number; minZ: number; maxZ: number };
  groundLevel: number;
}

function windowMaterial(): THREE.MeshStandardMaterial {
  const m = new THREE.MeshStandardMaterial({ color: 0xd8d4cc, roughness: 0.75, metalness: 0.1 });
  m.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vCityWorld;\nvarying vec3 vCityNormal;')
      .replace(
        '#include <worldpos_vertex>',
        `#include <worldpos_vertex>
        vec4 cw = vec4(transformed, 1.0);
        #ifdef USE_INSTANCING
          cw = instanceMatrix * cw;
        #endif
        vCityWorld = (modelMatrix * cw).xyz;
        vCityNormal = normal;`,
      );
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vCityWorld;\nvarying vec3 vCityNormal;')
      .replace(
        '#include <color_fragment>',
        `#include <color_fragment>
        if (abs(vCityNormal.y) < 0.5) {
          float u = abs(vCityNormal.x) > 0.5 ? vCityWorld.z : vCityWorld.x;
          float fy = fract(vCityWorld.y / 3.2);
          float fx = fract(u / 3.0);
          float win = step(0.18, fy) * step(fy, 0.85) * step(0.12, fx) * step(fx, 0.88);
          diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.16, 0.22, 0.28), win * 0.85);
        }`,
      );
  };
  return m;
}

export function createCity(o: CityOptions): THREE.InstancedMesh {
  const r = new SeededRandom(o.seed);
  const boxes: { x: number; z: number; w: number; d: number; h: number }[] = [];
  // Beachfront row (west of the road), then two rows behind it.
  for (const row of [
    { x: -10, spacing: 46, hMin: 70, hMax: 230 },
    { x: -80, spacing: 42, hMin: 40, hMax: 170 },
    { x: -150, spacing: 38, hMin: 25, hMax: 110 },
    { x: -230, spacing: 36, hMin: 15, hMax: 70 },
  ]) {
    for (let z = -1500; z <= 1500; z += row.spacing * r.range(0.85, 1.2)) {
      if (!r.chance(o.density)) continue;
      const w = r.range(18, 30);
      const d = r.range(18, 30);
      const x = row.x + r.range(-6, 6);
      if (
        x + w / 2 > o.exclude.minX &&
        x - w / 2 < o.exclude.maxX &&
        z + d / 2 > o.exclude.minZ &&
        z - d / 2 < o.exclude.maxZ
      )
        continue;
      boxes.push({ x, z, w, d, h: r.range(row.hMin, row.hMax) });
    }
  }
  const geo = new THREE.BoxGeometry(1, 1, 1);
  geo.translate(0, 0.5, 0);
  const mesh = new THREE.InstancedMesh(geo, windowMaterial(), boxes.length);
  const m = new THREE.Matrix4();
  const tint = new THREE.Color();
  boxes.forEach((b, i) => {
    m.makeScale(b.w, b.h, b.d).setPosition(b.x, o.groundLevel, b.z);
    mesh.setMatrixAt(i, m);
    tint.setHSL(0.08 + r.range(-0.03, 0.05), r.range(0.05, 0.2), r.range(0.62, 0.85));
    mesh.setColorAt(i, tint);
  });
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.name = 'city';
  return mesh;
}

/** Promenade road and kerb along the beach. */
export function createRoad(minX: number, maxX: number, level: number, length = 3200): THREE.Group {
  const g = new THREE.Group();
  const road = new THREE.Mesh(
    new THREE.BoxGeometry(maxX - minX, 0.3, length),
    new THREE.MeshStandardMaterial({ color: 0x2f3134, roughness: 0.9 }),
  );
  road.position.set((minX + maxX) / 2, level + 0.15, 0);
  road.receiveShadow = true;
  const line = new THREE.Mesh(
    new THREE.BoxGeometry(0.25, 0.05, length),
    new THREE.MeshBasicMaterial({ color: 0xe8d36a }),
  );
  line.position.set((minX + maxX) / 2, level + 0.32, 0);
  g.add(road, line);
  g.name = 'road';
  return g;
}
