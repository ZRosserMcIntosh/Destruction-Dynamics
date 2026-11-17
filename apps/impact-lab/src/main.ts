// Impact Lab: UI, cameras, automatic slow motion, effects and the frame loop.

import './style.css';
import { ProceduralAudioEngine, type SoundType } from '@destruction-dynamics/audio';
import { SeededRandom } from '@destruction-dynamics/core';
import { CRASH_CAR, type FractureEvent } from '@destruction-dynamics/destruction';
import { ParticleSim, ParticleView, type ParticleKind } from '@destruction-dynamics/rendering';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CarView } from './carView';
import {
  CANNON_X,
  ImpactLab,
  type LabSetup,
  type LiveProjectile,
  PROJECTILES,
  type ProjectileId,
  STEP,
  TARGETS,
  TARGET_CENTRE,
  type TargetId,
  WALLS,
  type WallId,
} from './lab';
import { PieceRenderer } from './pieces';
import { createCannon, createFrame, createProjectileMesh } from './props';
import { createStage, resizeStage } from './scene';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const viewport = $<HTMLDivElement>('viewport');

// --- Initial setup from the URL (?mode=gallery&target=glass, ?wall=cheese&speed=120) ---
const params = new URLSearchParams(location.search);
const isWall = (v: string | null): v is WallId => !!v && v in WALLS;
const isTarget = (v: string | null): v is TargetId => !!v && v in TARGETS;
const isProjectile = (v: string | null): v is ProjectileId => !!v && v in PROJECTILES;
let crashSetup: Extract<LabSetup, { mode: 'crash' }> = {
  mode: 'crash',
  wall: isWall(params.get('wall')) ? (params.get('wall') as WallId) : 'brick',
  speedKmh: Number(params.get('speed')) || 80,
};
let gallerySetup: Extract<LabSetup, { mode: 'gallery' }> = {
  mode: 'gallery',
  target: isTarget(params.get('target')) ? (params.get('target') as TargetId) : 'glass',
};
const startMode = params.get('mode') === 'gallery' ? 'gallery' : 'crash';

// --- Scene ---
const stage = createStage(viewport);
const { scene, camera, renderer } = stage;
const pieces = new PieceRenderer();
scene.add(pieces.group);
const carView = new CarView(CRASH_CAR);
scene.add(carView.group);
const cannon = createCannon();
scene.add(cannon);
let frame: THREE.Group | null = null;
const particles = new ParticleSim(6000);
const particleView = new ParticleView(6000);
particleView.setViewport(viewport.clientHeight, camera.fov);
scene.add(particleView.group);
const projectileMeshes = new Map<LiveProjectile, THREE.Object3D>();
const audio = new ProceduralAudioEngine(0.7);
let audioStarted = false;
const rng = new SeededRandom(11);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 1, 0);
controls.enabled = false;

const lab = await ImpactLab.create(startMode === 'crash' ? crashSetup : gallerySetup);

function rebuildView(): void {
  pieces.build(lab.fw);
  const crash = lab.setup.mode === 'crash';
  carView.group.visible = crash;
  cannon.visible = !crash;
  if (frame) {
    scene.remove(frame);
    frame.traverse((o) => o instanceof THREE.Mesh && o.geometry.dispose());
    frame = null;
  }
  if (!crash) {
    frame = createFrame(TARGETS[(lab.setup as { target: TargetId }).target]);
    scene.add(frame);
  } else {
    // Concrete sill under the wall.
    frame = new THREE.Group();
    const sill = new THREE.Mesh(
      new THREE.BoxGeometry(0.6, 0.1, 6.4),
      new THREE.MeshStandardMaterial({ color: 0x9a9ea2, roughness: 0.9 }),
    );
    sill.position.set(0, 0.05, 0);
    sill.castShadow = sill.receiveShadow = true;
    frame.add(sill);
    scene.add(frame);
  }
  for (const m of projectileMeshes.values()) scene.remove(m);
  projectileMeshes.clear();
  particles.clear();
  slowScale = 1;
  impactSeen = false;
  lastHitTime = -Infinity;
  cutCamera = true;
}
rebuildView();

// --- UI ---
const tabCrash = $<HTMLButtonElement>('tab-crash');
const tabGallery = $<HTMLButtonElement>('tab-gallery');
const crashControls = $<HTMLElement>('crash-controls');
const galleryControls = $<HTMLElement>('gallery-controls');
const wallSel = $<HTMLSelectElement>('wall');
const speed = $<HTMLInputElement>('speed');
const speedLabel = $<HTMLSpanElement>('speed-label');
const projSel = $<HTMLSelectElement>('projectile');
const targetSel = $<HTMLSelectElement>('target');
const launch = $<HTMLInputElement>('launch');
const launchLabel = $<HTMLSpanElement>('launch-label');
const slowSel = $<HTMLSelectElement>('slowmo');
const camSel = $<HTMLSelectElement>('camera');
const sound = $<HTMLInputElement>('sound');
const readout = $<HTMLDivElement>('readout');
const statsEl = $<HTMLPreElement>('stats');
const hud = $<HTMLDivElement>('hud');

wallSel.value = crashSetup.wall;
speed.value = String(crashSetup.speedKmh);
targetSel.value = gallerySetup.target;
if (isProjectile(params.get('projectile'))) projSel.value = params.get('projectile')!;
if (params.get('launch')) launch.value = params.get('launch')!;
const syncLabels = () => {
  speedLabel.textContent = `${speed.value} km/h`;
  launchLabel.textContent = `${launch.value} m/s`;
};
syncLabels();

let busy = false;
async function withLab(fn: () => Promise<void>): Promise<void> {
  if (busy) return;
  busy = true;
  try {
    await fn();
  } finally {
    busy = false;
  }
}

function showMode(mode: 'crash' | 'gallery'): void {
  tabCrash.classList.toggle('active', mode === 'crash');
  tabGallery.classList.toggle('active', mode === 'gallery');
  tabCrash.setAttribute('aria-selected', String(mode === 'crash'));
  tabGallery.setAttribute('aria-selected', String(mode === 'gallery'));
  crashControls.hidden = mode !== 'crash';
  galleryControls.hidden = mode !== 'gallery';
}
showMode(lab.setup.mode);

async function switchTo(setup: LabSetup): Promise<void> {
  await lab.reset(setup);
  showMode(setup.mode);
  rebuildView();
}

function startAudio(): void {
  if (audioStarted || !sound.checked) return;
  audioStarted = true;
  void audio.start();
}

tabCrash.onclick = () => withLab(() => switchTo(crashSetup));
tabGallery.onclick = () => withLab(() => switchTo(gallerySetup));
wallSel.onchange = () =>
  withLab(async () => {
    crashSetup = { ...crashSetup, wall: wallSel.value as WallId };
    await switchTo(crashSetup);
  });
speed.oninput = () => {
  syncLabels();
  crashSetup = { ...crashSetup, speedKmh: Number(speed.value) };
};
launch.oninput = syncLabels;
targetSel.onchange = () =>
  withLab(async () => {
    gallerySetup = { mode: 'gallery', target: targetSel.value as TargetId };
    await switchTo(gallerySetup);
  });
$<HTMLButtonElement>('btn-crash').onclick = () =>
  withLab(async () => {
    startAudio();
    crashSetup = { ...crashSetup, speedKmh: Number(speed.value), wall: wallSel.value as WallId };
    if (
      lab.startedAt !== null ||
      lab.setup.mode !== 'crash' ||
      (lab.setup as typeof crashSetup).wall !== crashSetup.wall
    )
      await switchTo(crashSetup);
    else lab.setup = crashSetup;
    lab.launch();
  });
$<HTMLButtonElement>('btn-reset').onclick = () => withLab(() => switchTo(crashSetup));
const replay = () =>
  withLab(async () => {
    startAudio();
    await lab.replay();
    rebuildView();
  });
$<HTMLButtonElement>('btn-replay').onclick = replay;
$<HTMLButtonElement>('btn-replay-g').onclick = replay;
$<HTMLButtonElement>('btn-fire').onclick = () => {
  startAudio();
  fire(projSel.value as ProjectileId, Number(launch.value));
};
$<HTMLButtonElement>('btn-new-target').onclick = () => withLab(() => switchTo(gallerySetup));
camSel.onchange = () => {
  controls.enabled = camSel.value === 'free';
  cutCamera = true;
};
sound.onchange = () => {
  if (!sound.checked) audio.stopAll();
  else startAudio();
};

function fire(p: ProjectileId, v: number): void {
  if (lab.setup.mode !== 'gallery') return;
  lab.fire(p, v);
  // Air-cannon puff at the muzzle.
  particles.emit('dust', 26, rng, (s, r) => {
    s.x = CANNON_X + 0.75;
    s.y = TARGET_CENTRE.y;
    s.z = 0;
    s.vx = r.range(1, 4);
    s.vy = r.range(-0.6, 0.8);
    s.vz = r.range(-0.8, 0.8);
    s.size = r.range(0.15, 0.35);
    s.life = r.range(0.8, 1.6);
  });
  play('metal', 0.15, { x: CANNON_X, y: TARGET_CENTRE.y, z: 0 });
}

// --- Effects and sound ---
const lastCue = new Map<SoundType, number>();
function play(type: SoundType, gain: number, at: { x: number; y: number; z: number }): void {
  if (!sound.checked || !audioStarted) return;
  const now = performance.now();
  if (now - (lastCue.get(type) ?? 0) < 70) return;
  lastCue.set(type, now);
  audio.play({ type, gain: Math.min(1, gain), position: at, seed: Math.floor(now) });
}

const GLASSY = new Set(['glass', 'glassBlock', 'ice']);
const kindCap = (m: string) => (GLASSY.has(m) ? 40 : m === 'melon' ? 30 : m === 'hay' ? 6 : 10);
function effects(events: FractureEvent[]): void {
  for (const e of events) {
    if (e.kind === 'hit' && e.impulse < 20) continue;
    // One slow-motion moment per strike on the target, not per falling shard.
    if (
      lab.setup.mode === 'gallery' &&
      Math.abs(e.point.x - TARGET_CENTRE.x) < 0.4 &&
      e.time - lastHitTime > 1
    )
      lastHitTime = e.time;
    const n = e.kind === 'hit' ? 1 : Math.min(kindCap(e.material), 2 + e.count);
    const kind: ParticleKind = GLASSY.has(e.material) ? 'glass' : e.material === 'melon' ? 'juice' : 'dust';
    if (e.material === 'cheese') continue;
    particles.emit(kind, kind === 'juice' ? n * 3 : n, rng, (s, r) => {
      s.x = e.point.x + r.range(-0.05, 0.05);
      s.y = e.point.y + r.range(-0.05, 0.05);
      s.z = e.point.z + r.range(-0.05, 0.05);
      const sp = kind === 'dust' ? r.range(0.3, 2) : r.range(1, 6);
      s.vx = r.range(-1, 1) * sp + (kind === 'dust' ? 0.5 : 2);
      s.vy = r.range(-0.3, 1) * sp;
      s.vz = r.range(-1, 1) * sp;
      s.size =
        kind === 'dust' ? r.range(0.06, 0.2) : kind === 'juice' ? r.range(0.015, 0.04) : r.range(0.01, 0.03);
      s.life = kind === 'dust' ? r.range(1.2, 2.6) : r.range(0.6, 1.4);
    });
    const loud = Math.min(1, 0.15 + Math.log10(1 + e.impulse) * 0.25);
    play(GLASSY.has(e.material) ? 'glass' : e.material === 'melon' ? 'splash' : 'debris', loud, e.point);
  }
}

// --- Cameras and slow motion ---
let slowScale = 1;
let impactSeen = false;
let lastHitTime = -Infinity;
let cutCamera = true;
let orbitAngle = 0;
const camTarget = new THREE.Vector3(0, 1, 0);
const camPos = new THREE.Vector3();

function autoSlow(): number {
  if (lab.setup.mode === 'crash') {
    const car = lab.car!;
    if (lab.startedAt === null) return 1;
    if (car.impactTime === null) {
      const d = lab.approach();
      return d > 3 ? 1 : THREE.MathUtils.lerp(0.06, 1, Math.max(0, d) / 3);
    }
    const since = lab.time - car.impactTime;
    if (since < 0.5) return 0.06;
    if (since < 1.6) return THREE.MathUtils.lerp(0.06, 0.5, (since - 0.5) / 1.1);
    return 1;
  }
  const d = lab.approach();
  const since = lab.time - lastHitTime;
  if (since < 0.15) return 0.04;
  if (d < 1.5) return THREE.MathUtils.lerp(0.04, 1, d / 1.5);
  if (since < 0.8) return THREE.MathUtils.lerp(0.04, 1, (since - 0.15) / 0.65);
  return 1;
}

function desiredCamera(dt: number): { pos: THREE.Vector3; target: THREE.Vector3; snap?: boolean } {
  const mode = camSel.value;
  if (lab.setup.mode === 'crash') {
    const car = lab.car!;
    const front = car.front;
    switch (mode) {
      case 'side':
        return { pos: new THREE.Vector3(-1.5, 1.2, 9.5), target: new THREE.Vector3(-1.5, 1, 0) };
      case 'front':
        return { pos: new THREE.Vector3(9, 1.5, 2.6), target: new THREE.Vector3(-4, 0.9, 0) };
      case 'top':
        return { pos: new THREE.Vector3(-1.5, 14, 0.01), target: new THREE.Vector3(-1.5, 0, 0) };
      case 'onboard':
        return {
          pos: new THREE.Vector3(car.rear + 1.8, 1.25, 0.4),
          target: new THREE.Vector3(car.rear + 8, 0.9, 0.2),
          snap: true,
        };
    }
    // Cinematic.
    if (lab.startedAt === null)
      return { pos: new THREE.Vector3(-19, 2.6, 9.5), target: new THREE.Vector3(-7, 0.9, 0) };
    const since = car.impactTime === null ? -1 : lab.time - car.impactTime;
    if (car.impactTime === null && front < -6)
      return {
        pos: new THREE.Vector3(front - 2.5, 1.0, 5.4),
        target: new THREE.Vector3(front + 1.5, 0.8, 0),
        snap: true,
      };
    if (since < 4) {
      orbitAngle += dt * 0.12;
      const a = 2.15 + orbitAngle;
      return {
        pos: new THREE.Vector3(Math.cos(a) * 7.2 - 0.8, 1.5 + orbitAngle * 0.4, Math.sin(a) * 7.2),
        target: new THREE.Vector3(-0.4, 0.9, 0),
      };
    }
    return { pos: new THREE.Vector3(-9, 3.6, 9), target: new THREE.Vector3(Math.max(0, front - 2), 0.8, 0) };
  }
  // Gallery.
  switch (mode) {
    case 'side':
      return { pos: new THREE.Vector3(-3, 1.5, 5), target: new THREE.Vector3(-3, 1.4, 0) };
    case 'front':
      return { pos: new THREE.Vector3(3.2, 1.7, 1.3), target: new THREE.Vector3(-6, 1.3, 0) };
    case 'top':
      return { pos: new THREE.Vector3(-3, 9, 0.01), target: new THREE.Vector3(-3, 0, 0) };
    case 'onboard': {
      const last = lab.projectiles[lab.projectiles.length - 1];
      const rb = last?.rb ?? (last?.pieces ? lab.fw.bodyOfPiece(last.pieces[0]!) : undefined);
      if (rb) {
        const t = rb.translation();
        return {
          pos: new THREE.Vector3(t.x - 1.2, t.y + 0.25, t.z + 0.35),
          target: new THREE.Vector3(t.x + 2, t.y, t.z),
          snap: true,
        };
      }
      return { pos: new THREE.Vector3(CANNON_X - 1.5, 1.7, 0.4), target: new THREE.Vector3(0, 1.4, 0) };
    }
  }
  const near = lab.approach() < 2.5 || lab.time - lastHitTime < 1.2;
  if (near) {
    orbitAngle += dt * 0.25;
    const a = 2.35 + orbitAngle;
    return {
      pos: new THREE.Vector3(Math.cos(a) * 2.6, 1.65, Math.sin(a) * 2.6),
      target: new THREE.Vector3(-0.05, TARGET_CENTRE.y, 0),
    };
  }
  orbitAngle = 0;
  return { pos: new THREE.Vector3(-3.4, 1.9, 4.6), target: new THREE.Vector3(-2.6, 1.3, 0) };
}

function updateCamera(dt: number): void {
  if (camSel.value === 'free') {
    controls.update();
    return;
  }
  const d = desiredCamera(dt);
  const k = d.snap || cutCamera ? 1 : 1 - Math.exp(-dt * 3);
  camPos.lerp(d.pos, k);
  camTarget.lerp(d.target, k);
  if (d.snap || cutCamera) {
    camPos.copy(d.pos);
    camTarget.copy(d.target);
  }
  cutCamera = false;
  camera.position.copy(camPos);
  camera.lookAt(camTarget);
  controls.target.copy(camTarget);
}

// --- Projectile meshes ---
function syncProjectiles(alpha: number): void {
  for (const p of lab.projectiles) {
    if (projectileMeshes.has(p)) continue;
    if (p.pieces) {
      pieces.registerMelon(lab.fw, p.pieces, { x: CANNON_X + 0.6, y: TARGET_CENTRE.y, z: 0 });
      projectileMeshes.set(p, new THREE.Group());
      continue;
    }
    const m = createProjectileMesh(p.id);
    m.traverse((o) => (o.castShadow = true));
    scene.add(m);
    projectileMeshes.set(p, m);
  }
  for (const [p, m] of projectileMeshes) {
    if (!lab.projectiles.includes(p)) {
      scene.remove(m);
      projectileMeshes.delete(p);
      continue;
    }
    if (!p.rb) continue;
    const pose = lab.bodyPose(p.rb, alpha);
    m.position.set(pose.position.x, pose.position.y, pose.position.z);
    m.quaternion.set(pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w);
  }
}

// --- Readouts ---
function updateReadout(scale: number): void {
  const s = lab.stats();
  if (s.car) {
    const c = s.car;
    readout.innerHTML = `
      <div><b>${c.speedKmh.toFixed(0)}</b><span>km/h now</span></div>
      <div><b>${c.crush.toFixed(2)} m</b><span>front crush</span></div>
      <div><b>${c.peakG.toFixed(0)} g</b><span>peak car deceleration</span></div>
      <div><b>${c.chestG.toFixed(0)} g</b><span>peak dummy chest</span></div>
      <div><b>${s.loose}</b><span>pieces loose</span></div>
      <div><b>${s.jointsBroken}</b><span>joints broken</span></div>`;
    hud.innerHTML = `${WALLS[(lab.setup as typeof crashSetup).wall].label} · ${(lab.setup as typeof crashSetup).speedKmh} km/h<br><span class="big">${c.speedKmh.toFixed(0)} km/h</span>${
      c.impact
        ? `<br>${c.peakG.toFixed(0)} g peak · ${c.crush.toFixed(2)} m crush${c.airbags ? ' · airbags' : ''}`
        : ''
    }${scale < 0.5 ? `<br><span class="slow">SLOW MOTION ×${scale < 0.1 ? scale.toFixed(2) : scale.toFixed(1)}</span>` : ''}`;
  } else {
    readout.innerHTML = `
      <div><b>${s.shots}</b><span>shots fired</span></div>
      <div><b>${s.loose}</b><span>pieces loose</span></div>
      <div><b>${s.jointsBroken}</b><span>joints broken</span></div>
      <div><b>${s.bodies}</b><span>rigid bodies</span></div>`;
    const proj = PROJECTILES[projSel.value as ProjectileId];
    hud.innerHTML = `${proj.label} → ${TARGETS[(lab.setup as typeof gallerySetup).target].label}<br><span class="big">${launch.value} m/s</span>${
      scale < 0.5 ? `<br><span class="slow">SLOW MOTION ×${scale.toFixed(2)}</span>` : ''
    }`;
  }
  statsEl.textContent = [
    `step ${s.stepMs.toFixed(1)} ms (solver ${s.solverMs.toFixed(1)})`,
    `bodies ${s.bodies} · pieces ${s.pieces} · shards drawn ${pieces.shardCount}`,
    `joints ${s.jointsBroken}/${s.joints} broken`,
    `sim time ${s.time.toFixed(2)} s · scale ${scale.toFixed(2)}`,
    `particles ${particles.count}`,
  ].join('\n');
}

// --- Loop ---
let acc = 0;
let last = performance.now();
let frames = 0;
let readoutAt = 0;
function tick(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const target = slowSel.value === 'auto' ? autoSlow() : Number(slowSel.value);
  slowScale += (target - slowScale) * (1 - Math.exp(-dt * 8));
  if (target < slowScale) slowScale = target; // drop into slow motion at once
  if (!busy) {
    acc += dt * slowScale;
    let steps = 0;
    while (acc >= STEP && steps < 6) {
      lab.step();
      acc -= STEP;
      steps++;
      if (lab.car?.impactTime !== null && lab.car && !impactSeen) {
        impactSeen = true;
        const v = lab.stats().car!.speedKmh;
        play('metal', 0.4 + v / 150, { x: 0, y: 0.6, z: 0 });
        play('rumble', 0.3 + v / 200, { x: 0, y: 0.6, z: 0 });
      }
      effects(lab.takeEvents());
    }
    if (steps === 6) acc = Math.min(acc, STEP);
  }
  const alpha = Math.min(1, acc / STEP);
  syncProjectiles(alpha);
  pieces.update(lab, alpha);
  if (lab.car) {
    const c = lab.carState(alpha);
    carView.update(lab.car, c.rear, c.crush);
  }
  particles.update(dt * slowScale, { gravity: 9.81, waterHeight: () => -1e9, groundHeight: () => 0 });
  particleView.update(particles);
  updateCamera(dt);
  audio.setListener({
    position: camera.position,
    forward: camera.getWorldDirection(new THREE.Vector3()),
    up: camera.up,
    underwater: false,
  });
  renderer.render(scene, camera);
  if (now - readoutAt > 150) {
    readoutAt = now;
    updateReadout(slowScale);
  }
  frames++;
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

window.addEventListener('resize', () => {
  resizeStage(stage, viewport);
  particleView.setViewport(viewport.clientHeight, camera.fov);
});

// Test and automation hooks.
declare global {
  interface Window {
    __IMPACT?: {
      state: () => ReturnType<ImpactLab['stats']> & { frames: number; scale: number };
      crash: (wall: WallId, kmh: number) => Promise<void>;
      fire: (p: ProjectileId, v: number) => void;
      target: (t: TargetId) => Promise<void>;
      slowmo: (v: string) => void;
    };
  }
}
window.__IMPACT = {
  state: () => ({ ...lab.stats(), frames, scale: slowScale }),
  crash: async (wall, kmh) => {
    wallSel.value = wall;
    speed.value = String(kmh);
    syncLabels();
    crashSetup = { mode: 'crash', wall, speedKmh: kmh };
    await switchTo(crashSetup);
    lab.launch();
  },
  fire: (p, v) => fire(p, v),
  target: async (t) => {
    targetSel.value = t;
    gallerySetup = { mode: 'gallery', target: t };
    await switchTo(gallerySetup);
  },
  slowmo: (v) => {
    slowSel.value = v;
  },
};
