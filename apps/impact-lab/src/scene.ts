// The test facility: a concrete pad with lane markings, sky, sun with
// shadows, and a reflection environment for paint, steel and glass.

import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

export interface Stage {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  sun: THREE.DirectionalLight;
}

export function createStage(container: HTMLElement): Stage {
  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xaec8dc);
  scene.fog = new THREE.Fog(0xaec8dc, 60, 260);
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.55;

  const camera = new THREE.PerspectiveCamera(42, container.clientWidth / container.clientHeight, 0.05, 900);
  camera.position.set(-6, 2.2, 10);

  scene.add(new THREE.HemisphereLight(0xdfeeff, 0x6b6257, 1.1));
  const sun = new THREE.DirectionalLight(0xfff1dc, 2.6);
  sun.position.set(-14, 22, 12);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  const sc = sun.shadow.camera;
  sc.left = -18;
  sc.right = 18;
  sc.top = 14;
  sc.bottom = -14;
  sc.near = 1;
  sc.far = 80;
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.02;
  scene.add(sun, sun.target);

  scene.add(createGround());
  scene.add(createSkyDome());
  return { renderer, scene, camera, sun };
}

function concreteTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d')!;
  g.fillStyle = '#8e9296';
  g.fillRect(0, 0, 512, 512);
  // Speckle and stains.
  let seed = 7;
  const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 9000; i++) {
    const v = 120 + rnd() * 50;
    g.fillStyle = `rgba(${v},${v + 2},${v + 4},${0.15 + rnd() * 0.2})`;
    g.fillRect(rnd() * 512, rnd() * 512, 1 + rnd() * 2, 1 + rnd() * 2);
  }
  for (let i = 0; i < 18; i++) {
    g.fillStyle = `rgba(60,62,66,${0.04 + rnd() * 0.05})`;
    g.beginPath();
    g.ellipse(rnd() * 512, rnd() * 512, 20 + rnd() * 70, 10 + rnd() * 40, rnd() * 3, 0, 7);
    g.fill();
  }
  // Expansion joint along two edges (the texture tiles every 5 m).
  g.fillStyle = 'rgba(40,42,45,0.8)';
  g.fillRect(0, 0, 512, 3);
  g.fillRect(0, 0, 3, 512);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

function createGround(): THREE.Group {
  const group = new THREE.Group();
  const tex = concreteTexture();
  tex.repeat.set(80, 80);
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(400, 400),
    new THREE.MeshStandardMaterial({ map: tex, roughness: 0.92, metalness: 0 }),
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  group.add(ground);
  // Lane markings along the run-up (x from −40 to the wall).
  const paint = new THREE.MeshStandardMaterial({ color: 0xf2d24a, roughness: 0.7 });
  const white = new THREE.MeshStandardMaterial({ color: 0xf4f4f0, roughness: 0.7 });
  for (const z of [-1.6, 1.6]) {
    const line = new THREE.Mesh(new THREE.PlaneGeometry(44, 0.12), paint);
    line.rotation.x = -Math.PI / 2;
    line.position.set(-22, 0.004, z);
    line.receiveShadow = true;
    group.add(line);
  }
  for (let x = -40; x <= -2; x += 2) {
    const tick = new THREE.Mesh(new THREE.PlaneGeometry(0.08, x % 10 === 0 ? 0.9 : 0.4), white);
    tick.rotation.x = -Math.PI / 2;
    tick.position.set(x, 0.005, 2.1);
    group.add(tick);
  }
  return group;
}

function createSkyDome(): THREE.Mesh {
  const g = new THREE.SphereGeometry(600, 32, 16);
  const m = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    depthWrite: false,
    fog: false,
    vertexShader: /* glsl */ `
      varying vec3 vDir;
      void main() {
        vDir = normalize(position);
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }`,
    fragmentShader: /* glsl */ `
      varying vec3 vDir;
      void main() {
        float h = clamp(vDir.y, -0.1, 1.0);
        vec3 zenith = vec3(0.32, 0.55, 0.82);
        vec3 horizon = vec3(0.74, 0.83, 0.9);
        vec3 c = mix(horizon, zenith, pow(max(h, 0.0), 0.6));
        gl_FragColor = vec4(c, 1.0);
      }`,
  });
  const sky = new THREE.Mesh(g, m);
  sky.name = 'sky';
  return sky;
}

export function resizeStage(s: Stage, container: HTMLElement): void {
  const w = container.clientWidth,
    h = container.clientHeight;
  s.renderer.setSize(w, h);
  s.camera.aspect = w / h;
  s.camera.updateProjectionMatrix();
}
