// CPU-side costs of the simulation subsystems at each quality preset's budget.
// Run with `npm run bench`. GPU cost is not measured here.

import { QUALITY_PRESETS, SeededRandom, quatIdentity, vec3 } from '@destruction-dynamics/core';
import {
  DEEP_END_TOWER,
  LayerSolver,
  TowerCollapse,
  buildFacadePlan,
  towerSections,
} from '@destruction-dynamics/destruction';
import { EventDirector } from '@destruction-dynamics/events';
import { HybridOcean, NearFieldSolver, computeBuoyancy, makeWaves } from '@destruction-dynamics/fluids';
import { RapierPhysicsAdapter } from '@destruction-dynamics/physics';
import { ParticleSim } from '@destruction-dynamics/rendering';
import {
  CHEESE_OBSTACLE,
  deepEndDepth,
  deepEndEvent,
  deepEndGroundHeight,
  deepEndTimeline,
} from '@destruction-dynamics/scenarios';
import { mkdirSync, writeFileSync } from 'node:fs';
import { afterAll, describe, it } from 'vitest';

const results: { group: string; name: string; meanMs: number; p95Ms: number; samples: number }[] = [];
let group = '';

/** Times `fn` for ~`ms` milliseconds after a warm-up; records mean and p95 per call. */
function bench(name: string, fn: () => void, ms = 400): void {
  const g = group;
  it(name, () => {
    for (let i = 0; i < 5; i++) fn();
    const samples: number[] = [];
    const end = performance.now() + ms;
    while (performance.now() < end || samples.length < 10) {
      const t0 = performance.now();
      fn();
      samples.push(performance.now() - t0);
    }
    samples.sort((a, b) => a - b);
    const mean = samples.reduce((a, b) => a + b, 0) / samples.length;
    results.push({
      group: g,
      name,
      meanMs: mean,
      p95Ms: samples[Math.floor(samples.length * 0.95)]!,
      samples: samples.length,
    });
  });
}

const suite = (name: string, body: () => void | Promise<void>) =>
  describe(name, async () => {
    group = name;
    await body();
  });

afterAll(() => {
  const lines = results.map(
    (r) =>
      `${r.group.padEnd(70)} ${r.name.padEnd(34)} ${r.meanMs.toFixed(3).padStart(9)} ms  p95 ${r.p95Ms.toFixed(3).padStart(9)} ms`,
  );
  console.info(`\n${lines.join('\n')}`);
  mkdirSync('benchmarks/results', { recursive: true });
  writeFileSync(
    'benchmarks/results/latest.json',
    JSON.stringify(
      { node: process.version, platform: process.platform, date: new Date().toISOString(), results },
      null,
      2,
    ),
  );
});

const presets = Object.values(QUALITY_PRESETS);

suite('near-field shallow water, one 1/60 s step', () => {
  for (const q of presets) {
    const s = new NearFieldSolver({
      size: q.fluidGridSize,
      extent: 840,
      centerX: 260,
      centerZ: 0,
      depth: deepEndDepth,
      maxDepth: 45,
    });
    s.addPressureBox(190, 0, 100, 19, 0, 14);
    bench(`${q.id} (${q.fluidGridSize}²)`, () => s.step(1 / 60));
  }
});

suite('hybrid ocean step with an active landmark surge (solver + composite)', () => {
  for (const q of presets) {
    const o = new HybridOcean({
      waves: makeWaves('moderate', 0.35, 7),
      depth: deepEndDepth,
      nearField: { size: q.fluidGridSize, extent: 840, centerX: 260, centerZ: 0, maxDepth: 45 },
    });
    o.applyImpact({
      position: vec3(190, 0, 0),
      footprint: { kind: 'box', halfLength: 100, halfWidth: 19, yaw: 0 },
      intensity: 1,
      duration: 1.6,
      direction: vec3(1, 0, 0),
      visualProfile: 'landmark-collapse',
    });
    bench(`${q.id} (${q.fluidGridSize}²)`, () => o.step(1 / 60));
  }
});

suite('particles update', () => {
  for (const q of presets) {
    const p = new ParticleSim(q.maxParticles);
    const env = { gravity: 9.81, waterHeight: () => 0, groundHeight: () => -50 };
    bench(`${q.id} (${q.maxParticles})`, () => {
      if (p.count < q.maxParticles * 0.9)
        p.emit('mist', q.maxParticles, new SeededRandom(1), (s, r) => {
          s.y = r.range(2, 50);
          s.life = 1e9;
        });
      p.update(1 / 60, env);
    });
  }
});

suite('rapier step, preset debris budget (bodies fall, then settle and sleep)', async () => {
  for (const q of presets) {
    const phys = new RapierPhysicsAdapter();
    await phys.init();
    phys.createBody({
      kind: 'static',
      shape: { type: 'box', halfExtents: vec3(500, 1, 500) },
      pose: { position: vec3(0, -1, 0), quaternion: quatIdentity() },
    });
    const r = new SeededRandom(2);
    for (let i = 0; i < q.maxDebrisBodies; i++)
      phys.createBody({
        kind: 'dynamic',
        shape: { type: 'box', halfExtents: vec3(r.range(0.3, 1.5), r.range(0.1, 0.8), r.range(0.3, 1.5)) },
        pose: {
          position: vec3(r.range(-80, 80), r.range(5, 200), r.range(-80, 80)),
          quaternion: quatIdentity(),
        },
      });
    bench(`${q.id} (${q.maxDebrisBodies} bodies)`, () => phys.step(1 / q.physicsHz));
  }
});

suite('buoyancy for floating props (8 samples each)', () => {
  const o = new HybridOcean({
    waves: makeWaves('moderate', 0.35, 7),
    depth: deepEndDepth,
    nearField: { size: 96, extent: 840, centerX: 260, centerZ: 0 },
  });
  for (const q of presets) {
    bench(`${q.id} (${q.floatingObjects} props)`, () => {
      for (let i = 0; i < q.floatingObjects; i++)
        computeBuoyancy(
          {
            pose: { position: vec3(200 + i * 5, 0, i * 3), quaternion: quatIdentity() },
            halfExtents: vec3(3.5, 0.8, 1.3),
            velocity: vec3(),
            mass: 8000,
          },
          o,
        );
    });
  }
});

suite('authored structure', () => {
  const tl = deepEndTimeline();
  const tower = new TowerCollapse(DEEP_END_TOWER, tl, { groundHeight: deepEndGroundHeight, seed: 1 });
  let t = 0;
  bench('tower poses at an event time', () => {
    tower.posesAt((t = (t + 0.37) % 60));
  });
  bench('facade plan build (8 columns, 2720 panels)', () => {
    buildFacadePlan(DEEP_END_TOWER, towerSections(DEEP_END_TOWER), tl, 8, 1);
  });
  const d = new EventDirector(deepEndEvent);
  bench('director update (1/60 s, looping)', () => {
    if (d.state === 'INTACT') d.trigger();
    if (d.state === 'AFTERMATH') d.reset();
    else d.update(1 / 60);
  });
});

suite('floor-layer precompute (whole collapse, one call)', () => {
  for (const [label, sections, obstacles] of [
    ['12 layers', 12, []],
    ['26 layers', 26, []],
    ['77 floors', 77, []],
    ['77 floors + cheese wedge', 77, [CHEESE_OBSTACLE]],
  ] as const) {
    const tower = new TowerCollapse({ ...DEEP_END_TOWER, sections }, deepEndTimeline(), {
      groundHeight: deepEndGroundHeight,
      seed: 1234,
      simulateLayers: 'deferred',
      obstacles: [...obstacles],
    });
    const { layers, options } = tower.layerSolverInput();
    bench(
      label,
      () => void new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run(),
      1,
    );
  }
});
