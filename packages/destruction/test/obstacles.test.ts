import { add, quatRotate, vec3 } from '@destruction-dynamics/core';
import { CHEESE_OBSTACLE, deepEndGroundHeight, deepEndTimeline } from '@destruction-dynamics/scenarios';
import { describe, expect, it } from 'vitest';
import {
  DEEP_END_TOWER,
  TowerCollapse,
  boxShape,
  shapePenetration,
  toObstacleLocal,
  wedgeShape,
} from '../src';

const timeline = deepEndTimeline();
const o = CHEESE_OBSTACLE;
/** How deep a world point is inside the cheese (≤ 0 outside). */
const depthInCheese = (p: { x: number; y: number; z: number }) =>
  shapePenetration(o.shape, toObstacleLocal(o, p)).depth;

describe('convex shapes', () => {
  it('builds a closed box and a wheel wedge', () => {
    const box = boxShape(vec3(1, 2, 3));
    expect(box.edges).toHaveLength(12);
    expect(shapePenetration(box, vec3(0.5, 0, 0)).depth).toBeCloseTo(0.5);
    expect(shapePenetration(box, vec3(2, 0, 0)).depth).toBeLessThan(0);
    const w = wedgeShape({ radius: 100, angle: Math.PI / 3, height: 10, rindSegments: 6 });
    expect(shapePenetration(w, vec3(50, 5, 0)).depth).toBeGreaterThan(4);
    expect(shapePenetration(w, vec3(50, 5, 40)).depth).toBeLessThan(0); // outside the 60° opening
    expect(shapePenetration(w, vec3(-1, 5, 0)).depth).toBeLessThan(0); // behind the apex
    expect(shapePenetration(w, vec3(101, 5, 0)).depth).toBeLessThan(0); // past the rind
  });
});

describe('indestructible obstacles', () => {
  const tower = new TowerCollapse({ ...DEEP_END_TOWER, sections: 26 }, timeline, {
    groundHeight: deepEndGroundHeight,
    seed: 1234,
    simulateLayers: true,
    obstacles: [o],
  });

  it('releases the layers when the authored fall first touches the obstacle', () => {
    expect(tower.releaseTime).toBeGreaterThan(timeline.tipStart);
    expect(tower.releaseTime).toBeLessThan(timeline.impact);
    expect(
      new TowerCollapse(DEEP_END_TOWER, timeline, { groundHeight: deepEndGroundHeight, seed: 1 }).releaseTime,
    ).toBe(timeline.impact);
  });

  it('never lets a layer sink into the cheese, during or after the collapse', () => {
    let worst = 0;
    for (let t = tower.releaseTime; t <= timeline.settleEnd; t += 0.5) {
      tower.posesAt(t).forEach((p, k) => {
        if (k === 0) return;
        const h = tower.sections[k]!.height;
        for (const x of [-1, 0, 1])
          for (const y of [0, 0.5, 1])
            for (const z of [-1, 0, 1])
              worst = Math.max(
                worst,
                depthInCheese(add(p.position, quatRotate(p.quaternion, vec3(x * 16, y * h, z * 15)))),
              );
      });
    }
    expect(worst).toBeLessThan(1.5);
  });

  it('never launches a layer above the height it was released at', () => {
    const start = Math.max(
      ...tower
        .posesAt(tower.releaseTime)
        .slice(1)
        .map((p, k) => tower.sectionCenter(k + 1, p).y),
    );
    let peak = 0;
    for (let t = tower.releaseTime; t <= timeline.settleEnd; t += 0.1)
      tower.posesAt(t).forEach((p, k) => k && (peak = Math.max(peak, tower.sectionCenter(k, p).y)));
    expect(peak).toBeLessThanOrEqual(start + 5);
  });

  it('breaks the tower apart over it and settles', () => {
    const s = tower.layerStats!;
    expect(s.jointsBroken / s.joints).toBeGreaterThan(0.5);
    for (const [k, p] of tower.wreckPoses().entries()) {
      if (k === 0) continue;
      const c = tower.sectionCenter(k, p);
      expect(Number.isFinite(c.x + c.y + c.z)).toBe(true);
      expect(c.y - deepEndGroundHeight(c.x, c.z)).toBeGreaterThan(0);
      expect(depthInCheese(c)).toBeLessThan(0);
    }
  });
});
