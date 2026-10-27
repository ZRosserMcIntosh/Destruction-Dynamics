// Builds and updates the Three.js scene from the simulation.

import type { QualitySettings } from '@destruction-dynamics/core';
import {
  type AtmosphereUniforms,
  DebrisView,
  DebugDraw,
  OceanView,
  ParticleView,
  PropsView,
  TowerView,
  UnderwaterView,
  createAtmosphere,
  createCheeseWedge,
  createCity,
  createRoad,
  createSkyDome,
  createTerrain,
  setUnderwaterAtmosphere,
} from '@destruction-dynamics/rendering';
import { DEEP_END_LAYOUT, deepEndGroundHeight } from '@destruction-dynamics/scenarios';
import * as THREE from 'three';
import type { DeepEndSimulation } from './simulation';

export interface DebugFlags {
  physics: boolean;
  water: boolean;
  collision: boolean;
}

export class SceneView {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly atmosphere: AtmosphereUniforms;
  readonly debug: DebugFlags = { physics: false, water: false, collision: false };
  private readonly sun: THREE.DirectionalLight;
  private readonly caustics = { uTime: { value: 0 }, uCaustics: { value: 1 }, uWaterLevel: { value: 0 } };
  private readonly ocean: OceanView;
  private tower: TowerView;
  private towerLayout = -1;
  private readonly debris: DebrisView;
  private readonly props: PropsView;
  private particles: ParticleView;
  private readonly underwater: UnderwaterView;
  private readonly debugDraw = new DebugDraw();
  private city: THREE.InstancedMesh;
  private quality: QualitySettings;
  underwaterNow = false;
  readonly floatLinear: boolean;

  constructor(
    private readonly container: HTMLElement,
    private readonly sim: DeepEndSimulation,
  ) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 0.95;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    container.appendChild(this.renderer.domElement);
    this.floatLinear = this.renderer.extensions.has('OES_texture_float_linear');
    this.quality = sim.quality;

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.5, 30000);
    const sunDir = new THREE.Vector3(-0.35, 0.62, 0.7).normalize();
    this.atmosphere = createAtmosphere(sunDir);
    setUnderwaterAtmosphere(this.atmosphere, this.scene, false);

    this.sun = new THREE.DirectionalLight(0xfff1dd, 2.6);
    this.sun.position
      .copy(sunDir)
      .multiplyScalar(900)
      .add(new THREE.Vector3(150, 0, 0));
    this.sun.target.position.set(150, 0, 0);
    const sc = this.sun.shadow.camera;
    sc.left = -420;
    sc.right = 420;
    sc.top = 420;
    sc.bottom = -420;
    sc.near = 10;
    sc.far = 2500;
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.6;
    this.scene.add(this.sun, this.sun.target, new THREE.HemisphereLight(0xcfe3ff, 0x8a7b5f, 0.9));

    this.scene.add(createSkyDome(this.atmosphere));
    const L = DEEP_END_LAYOUT;
    this.scene.add(
      createTerrain(
        {
          groundHeight: deepEndGroundHeight,
          minX: -700,
          maxX: 3000,
          minZ: -1700,
          maxZ: 1700,
          segmentsX: 300,
          segmentsZ: 120,
          shoreX: L.shoreX,
          roadMinX: L.road.minX,
          roadMaxX: L.road.maxX,
        },
        this.caustics,
      ),
    );
    this.scene.add(createRoad(L.road.minX, L.road.maxX, L.cityLevel));
    const wedge = sim.variant.cheeseWedge;
    const o = sim.variant.obstacles[0];
    if (wedge && o)
      this.scene.add(createCheeseWedge(wedge, { position: o.position, quaternion: o.quaternion }));
    this.city = this.makeCity();
    this.scene.add(this.city);

    this.ocean = new OceanView({
      ocean: sim.ocean,
      atmosphere: this.atmosphere,
      gridResolution: this.quality.oceanGridResolution,
      floatLinear: this.floatLinear,
    });
    this.scene.add(this.ocean.group);
    this.tower = this.makeTower();
    this.debris = new DebrisView(this.quality.maxDebrisBodies);
    this.props = new PropsView(64);
    this.particles = new ParticleView(sim.particles.capacity);
    this.underwater = new UnderwaterView(this.quality.underwaterParticles);
    this.scene.add(this.debris.group, this.props.group, this.particles.group, this.underwater.sediment);
    this.scene.add(this.underwater.overlay, this.debugDraw.lines);
    this.applyQuality(this.quality);
    this.resize();
  }

  private makeCity(): THREE.InstancedMesh {
    return createCity({
      seed: 11,
      density: this.quality.cityDensity,
      exclude: { minX: -40, maxX: 40, minZ: -40, maxZ: 40 },
      groundLevel: DEEP_END_LAYOUT.cityLevel,
    });
  }

  private makeTower(): TowerView {
    const t = new TowerView(this.sim.tower.spec, this.sim.tower.sections, this.sim.plan);
    this.scene.add(t.group);
    this.towerLayout = this.sim.layoutVersion;
    return t;
  }

  applyQuality(q: QualitySettings): void {
    const prev = this.quality;
    this.quality = q;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, q.maxPixelRatio) * q.renderScale);
    this.renderer.shadowMap.enabled = q.shadows;
    this.sun.castShadow = q.shadows;
    if (this.sun.shadow.mapSize.x !== q.shadowMapSize) {
      this.sun.shadow.mapSize.set(q.shadowMapSize, q.shadowMapSize);
      this.sun.shadow.map?.dispose();
      this.sun.shadow.map = null;
    }
    this.ocean.setGridResolution(q.oceanGridResolution);
    this.debris.setCapacity(q.maxDebrisBodies);
    this.caustics.uCaustics.value = q.caustics ? 1 : 0;
    if (prev.underwaterParticles !== q.underwaterParticles)
      this.underwater.setSedimentCount(q.underwaterParticles);
    if (prev.cityDensity !== q.cityDensity) {
      this.scene.remove(this.city);
      this.city.geometry.dispose();
      this.city = this.makeCity();
      this.scene.add(this.city);
    }
    // Materials must recompile when shadows toggle.
    this.scene.traverse((o) => {
      if (o instanceof THREE.Mesh && o.material instanceof THREE.Material) o.material.needsUpdate = true;
    });
    this.resize();
  }

  resize(): void {
    const w = this.container.clientWidth || window.innerWidth;
    const h = this.container.clientHeight || window.innerHeight;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.particles.setViewport(h * this.renderer.getPixelRatio(), this.camera.fov);
  }

  /** Depth of the camera below the local water surface (negative when above). */
  cameraDepth(): number {
    return this.sim.ocean.heightAt(this.camera.position.x, this.camera.position.z) - this.camera.position.y;
  }

  render(): void {
    const sim = this.sim;
    if (sim.layoutVersion !== this.towerLayout) {
      this.scene.remove(this.tower.group);
      this.tower.dispose();
      this.tower = this.makeTower();
    }
    if (sim.particles.capacity !== this.particles.capacity) {
      this.scene.remove(this.particles.group);
      this.particles = new ParticleView(sim.particles.capacity);
      this.scene.add(this.particles.group);
      this.resize();
    }
    const d = sim.director;
    this.tower.setSectionPoses(sim.poses);
    this.tower.syncToTime(sim.tower.effectiveTime(sim.towerTime, d.restorationProgress));
    this.debris.update(sim.debris.active);
    this.props.update(sim.floaters);
    this.particles.update(sim.particles);
    this.ocean.update(this.camera);
    this.caustics.uTime.value = sim.ocean.time;

    const depth = this.cameraDepth();
    const under = depth > 0;
    if (under !== this.underwaterNow) {
      this.underwaterNow = under;
      setUnderwaterAtmosphere(this.atmosphere, this.scene, under);
    }
    this.underwater.update(this.camera, sim.ocean.time, depth, sim.turbidity);
    this.drawDebug();
    this.renderer.render(this.scene, this.camera);
  }

  private drawDebug(): void {
    const dd = this.debugDraw;
    dd.begin();
    const sim = this.sim;
    if (this.debug.physics) {
      for (const b of sim.debris.active)
        dd.box(b.pose, b.halfExtents, sim.physics.isSleeping(b.handle) ? 0x5577ff : 0x33ff66);
      for (const f of sim.floaters) dd.box(f.pose, { x: 3.5, y: 0.8, z: 1.3 }, 0xffaa33);
    }
    if (this.debug.collision && sim.collision.mode) {
      const color = { intact: 0x66ccff, kinematic: 0xff5533, wreck: 0xffdd33 }[sim.collision.mode];
      const poses = sim.collision.mode === 'wreck' ? sim.tower.wreckPoses() : sim.poses;
      poses.forEach((p, k) => {
        const half = sim.tower.sectionHalfExtents(k);
        const c = sim.tower.sectionCenter(k, p);
        dd.box({ position: c, quaternion: p.quaternion }, half, color);
      });
    }
    if (this.debug.water) {
      const nf = sim.ocean.near;
      dd.box(
        { position: { x: nf.centerX, y: 0, z: nf.centerZ }, quaternion: { x: 0, y: 0, z: 0, w: 1 } },
        { x: nf.extent / 2, y: 0.5, z: nf.extent / 2 },
        0x00ffff,
      );
    }
    dd.end();
    this.ocean.setDebug(this.debug.water);
  }

  get info() {
    const r = this.renderer.info;
    return {
      calls: r.render.calls,
      triangles: r.render.triangles,
      geometries: r.memory.geometries,
      textures: r.memory.textures,
      pixelRatio: this.renderer.getPixelRatio(),
      shadows: this.renderer.shadowMap.enabled,
      shadowMapSize: this.sun.shadow.mapSize.x,
      oceanGrid: this.ocean.gridResolution,
      oceanVertices: this.ocean.vertexCount,
      debugBoxes: this.debugDraw.boxes,
      underwater: this.underwaterNow,
      floatLinear: this.floatLinear,
      panelsDetached: this.tower.panelCount - countAttached(this.tower),
    };
  }
}

function countAttached(t: TowerView): number {
  let n = 0;
  for (let i = 0; i < t.panelCount; i++) if (t.isAttached(i)) n++;
  return n;
}
