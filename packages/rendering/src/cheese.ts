// A wedge of Swiss cheese cut from a wheel. The holes are 3D cellular noise in
// the wedge's own coordinates, so every cut face shows round eyes that line up
// like a real cut. The curved back carries a waxy rind. No textures or assets.

import type { Pose } from '@destruction-dynamics/core';
import * as THREE from 'three';

export interface CheeseWedgeParams {
  radius: number;
  /** Opening angle, radians. */
  angle: number;
  height: number;
}

const CHEESE_GLSL = /* glsl */ `
varying vec3 vCheese;
uniform float uRadius;
vec3 cheeseHash(vec3 p) {
  p = vec3(dot(p, vec3(127.1, 311.7, 74.7)), dot(p, vec3(269.5, 183.3, 246.1)), dot(p, vec3(113.5, 271.9, 124.6)));
  return fract(sin(p) * 43758.5453123);
}
// Signed distance to the nearest eye (negative inside), in cell units.
float cheeseEyes(vec3 p) {
  vec3 cell = floor(p), f = fract(p);
  float d = 1e9;
  for (int x = -1; x <= 1; x++)
    for (int y = -1; y <= 1; y++)
      for (int z = -1; z <= 1; z++) {
        vec3 o = vec3(float(x), float(y), float(z));
        vec3 h = cheeseHash(cell + o);
        // Fewer, larger eyes read better at city scale.
        float radius = h.x < 0.3 ? 0.0 : 0.22 + 0.24 * h.z;
        d = min(d, length(o + 0.2 + 0.6 * h - f) - radius);
      }
  return d;
}`;

export function createCheeseWedge(p: CheeseWedgeParams, pose: Pose): THREE.Mesh {
  // Sector in the shape's 2D plane: x forward, y = −z (so extruding and turning
  // the extrusion axis to +y gives a wedge opening along +x, from y = 0 up).
  const s = new THREE.Shape();
  s.moveTo(0, 0);
  s.absarc(0, 0, p.radius, -p.angle / 2, p.angle / 2, false);
  s.lineTo(0, 0);
  const g = new THREE.ExtrudeGeometry(s, { depth: p.height, bevelEnabled: false, curveSegments: 48 });
  g.rotateX(-Math.PI / 2);
  g.computeVertexNormals();

  const material = new THREE.MeshStandardMaterial({ color: 0xf6cd57, roughness: 0.6, metalness: 0 });
  material.onBeforeCompile = (shader) => {
    shader.uniforms.uRadius = { value: p.radius };
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vCheese;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvCheese = position;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>\n${CHEESE_GLSL}`)
      .replace(
        '#include <color_fragment>',
        `#include <color_fragment>
        float fromApex = length(vCheese.xz);
        float rind = smoothstep(uRadius - 1.6, uRadius - 0.6, fromApex);
        float eye = cheeseEyes(vCheese / 13.0);
        float inEye = (1.0 - smoothstep(-0.015, 0.015, eye)) * (1.0 - rind);
        // Darker toward the back of each eye, a pale lip around it.
        float depth = clamp(-eye * 2.5, 0.0, 1.0);
        vec3 cavity = mix(vec3(0.8, 0.52, 0.12), vec3(0.3, 0.16, 0.03), depth);
        float lip = smoothstep(0.05, 0.0, abs(eye)) * (1.0 - inEye) * (1.0 - rind);
        diffuseColor.rgb = mix(diffuseColor.rgb, cavity, inEye) + lip * 0.08;
        diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.8, 0.5, 0.12), rind);`,
      )
      .replace(
        '#include <roughnessmap_fragment>',
        `#include <roughnessmap_fragment>
        roughnessFactor = mix(roughnessFactor, 0.35, rind);`,
      );
  };
  const mesh = new THREE.Mesh(g, material);
  mesh.position.set(pose.position.x, pose.position.y, pose.position.z);
  mesh.quaternion.set(pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.name = 'cheese';
  return mesh;
}
