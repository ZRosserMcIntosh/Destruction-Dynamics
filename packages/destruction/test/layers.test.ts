import { add, distance, quatRotate, vec3 } from '@destruction-dynamics/core';
import { deepEndGroundHeight, deepEndTimeline } from '@destruction-dynamics/scenarios';
import { describe, expect, it } from 'vitest';
import { DEEP_END_TOWER, LayerSolver, TowerCollapse, layerPoseAt, towerSections } from '../src';

const timeline = deepEndTimeline();
const make = (sections: number, simulateLayers: boolean | 'deferred' = true, seed = 1234) =>
  new TowerCollapse({ ...DEEP_END_TOWER, sections }, timeline, {
    groundHeight: deepEndGroundHeight,
    seed,
    simulateLayers,
  });

const top = (t: TowerCollapse, time: number) => {
  const k = t.sections.length - 1;
  const p = t.posesAt(time)[k]!;
  return add(p.position, quatRotate(p.quaternion, vec3(0, t.sections[k]!.height, 0)));
};

describe('floor layers', () => {
  it('splits the tower one floor per layer at full fidelity', () => {
    const s = towerSections({ ...DEEP_END_TOWER, sections: 77 });
    expect(s).toHaveLength(78);
    expect(s.slice(1).every((x) => x.floorCount === 1)).toBe(true);
  });

  it('keeps the authored fall the same shape whatever the layer count', () => {
    const coarse = make(5, false);
    const fine = make(77, 'deferred');
    for (const f of [0.3, 0.7, 1]) {
      const t = timeline.tipStart + f * (timeline.impact - timeline.tipStart) - 1e-3;
      expect(distance(top(coarse, t), top(fine, t))).toBeLessThan(6);
    }
  });

  const tower = make(26);
  const stats = tower.layerStats!;

  it('releases layers at impact without a jump', () => {
    const a = tower.posesAt(timeline.impact - 1e-3);
    const b = tower.posesAt(timeline.impact + 1e-3);
    a.forEach((p, k) => expect(distance(p.position, b[k]!.position)).toBeLessThan(0.3));
  });

  it('breaks the slab joints and settles every layer on the ground or seabed', () => {
    expect(stats.layers).toBe(26);
    expect(stats.jointsBroken / stats.joints).toBeGreaterThan(0.5);
    for (const [k, p] of tower.wreckPoses().entries()) {
      if (k === 0) continue;
      const c = tower.sectionCenter(k, p);
      expect(Number.isFinite(c.x + c.y + c.z)).toBe(true);
      expect(c.x).toBeGreaterThan(0);
      expect(c.x).toBeLessThan(450);
      expect(Math.abs(c.z)).toBeLessThan(120);
      // Centre above the bed, and not perched absurdly high.
      const above = c.y - deepEndGroundHeight(c.x, c.z);
      expect(above).toBeGreaterThan(0.5);
      expect(above).toBeLessThan(40);
    }
  });

  it('ends at the wreck poses when SETTLING ends', () => {
    const end = tower.posesAt(timeline.settleEnd + 5);
    tower.wreckPoses().forEach((p, k) => expect(distance(p.position, end[k]!.position)).toBeLessThan(1e-3));
    expect(tower.settledAt).toBeLessThanOrEqual(timeline.settleEnd + 1e-6);
  });

  it('is deterministic for a seed', () => {
    const { layers, options } = tower.layerSolverInput();
    const run = () => new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run().data;
    expect(run()).toEqual(run());
  });

  it('attaches a result computed elsewhere (worker path)', () => {
    const deferred = make(26, 'deferred');
    expect(deferred.layersAttached).toBe(false);
    const { layers, options } = deferred.layerSolverInput();
    const r = new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run();
    deferred.attachLayers(r);
    expect(deferred.layersAttached).toBe(true);
    expect(deferred.layerVersion).toBe(1);
    expect(deferred.wreckPoses()).toEqual(tower.wreckPoses());
    expect(layerPoseAt(r, 3, 1e9).position).toEqual(tower.wreckPoses()[4]!.position);
    expect(() => make(5, 'deferred').attachLayers(r)).toThrow(/does not match/);
  });

  it('streams keyframes in chunks that rebuild the full result', () => {
    const { layers, options } = tower.layerSolverInput();
    const chunks: [number, number][] = [];
    let copy: Float32Array | null = null;
    const r = new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run({
      chunkFrames: 15,
      onFrames: (from, to, data) => {
        copy ??= new Float32Array(data.length);
        copy.set(data.subarray(from * layers.length * 7, to * layers.length * 7), from * layers.length * 7);
        chunks.push([from, to]);
      },
    });
    expect(chunks[0]![0]).toBe(0);
    chunks.slice(1).forEach(([from], i) => expect(from).toBe(chunks[i]![1]));
    expect(chunks.at(-1)![1]).toBe(r.frames);
    expect(copy).toEqual(r.data);
  });

  it('plays a partial stream, holding at the newest keyframe until more arrive', () => {
    const partial = make(26, 'deferred');
    const { layers, options } = partial.layerSolverInput();
    const r = new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run();
    partial.attachLayers(r, 30);
    expect(partial.layersComplete).toBe(false);
    const held = partial.posesAt(timeline.settleEnd);
    const atOneSecond = tower.posesAt(partial.releaseTime + 29 / r.hz);
    held.forEach((p, k) => expect(distance(p.position, atOneSecond[k]!.position)).toBeLessThan(1e-3));
    partial.extendLayers(r.frames);
    expect(partial.layersComplete).toBe(true);
    expect(partial.wreckPoses()).toEqual(tower.wreckPoses());
  });

  it('brings the pile nearly to rest by the end (no contact jitter)', () => {
    // Some layers still creep down the seabed slope, so not all of them sleep;
    // none should be jittering or sliding fast.
    const { layers, options } = tower.layerSolverInput();
    const r = new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run();
    const end = (r.frames - 1) / r.hz;
    for (let k = 0; k < r.layers; k++) {
      const a = layerPoseAt(r, k, end - 0.5).position;
      const b = layerPoseAt(r, k, end).position;
      expect(distance(a, b) / 0.5).toBeLessThan(0.5);
    }
  });

  it('simulates all 77 floors within a bounded precompute', () => {
    const full = make(77);
    const s = full.layerStats!;
    expect(s.layers).toBe(77);
    expect(s.jointsBroken).toBeGreaterThan(s.joints / 2);
    expect(s.computeMs).toBeLessThan(10_000);
    for (const [k, p] of full.wreckPoses().entries()) {
      if (k === 0) continue;
      const c = full.sectionCenter(k, p);
      expect(c.y - deepEndGroundHeight(c.x, c.z)).toBeGreaterThan(0);
      expect(Math.abs(c.z)).toBeLessThan(150);
    }
  }, 30_000);
});
