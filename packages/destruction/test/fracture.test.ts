import { describe, expect, it } from 'vitest';
import {
  BondGraph,
  CrashCar,
  FRACTURE_MATERIALS as M,
  FractureWorld,
  WORLD,
  buildPanel,
  buildScoredSphere,
  buildWall,
  voronoiBox,
} from '../src';

const brickUnit = { length: 0.225, height: 0.075, thickness: 0.1125 };

describe('Voronoi box split', () => {
  it('fills the box exactly with convex cells', () => {
    const half = { x: 0.5, y: 0.2, z: 0.1 };
    const seeds = Array.from({ length: 12 }, (_, i) => ({
      x: Math.sin(i * 7.1) * 0.45,
      y: Math.cos(i * 3.3) * 0.18,
      z: Math.sin(i * 1.7) * 0.09,
    }));
    const cells = voronoiBox(half, seeds);
    const total = cells.reduce((s, c) => s + c.volume, 0);
    expect(total).toBeCloseTo(8 * half.x * half.y * half.z, 6);
    for (const c of cells) {
      for (const p of c.points) {
        expect(Math.abs(p.x)).toBeLessThanOrEqual(half.x + 1e-6);
        expect(Math.abs(p.y)).toBeLessThanOrEqual(half.y + 1e-6);
      }
      // Shared faces are symmetric.
      for (const [j, patch] of c.neighbours)
        expect(cells[j]!.neighbours.get(cells.indexOf(c))?.area).toBeCloseTo(patch.area, 9);
    }
    // Each box face is covered exactly by the cells that touch it.
    const faceZ = cells.reduce((s, c) => s + (c.boxFaces.get(5)?.area ?? 0), 0);
    expect(faceZ).toBeCloseTo(4 * half.x * half.y, 6);
  });
});

describe('bond graph', () => {
  it('keeps a wall standing as one anchored island until joints break', () => {
    const g = new BondGraph(1);
    buildWall(g, {
      material: M.brick,
      base: { x: 0, y: 0, z: 0 },
      length: 1.2,
      height: 0.6,
      unit: brickUnit,
    });
    const isl = g.islands();
    expect(isl).toHaveLength(1);
    expect(isl[0]!.anchored).toBe(true);
    expect(g.bonds.some((b) => b.b === WORLD)).toBe(true);
  });

  it('shatters a plate around the impact and punches the middle out', () => {
    const g = new BondGraph(3);
    const id = buildPanel(g, {
      material: M.glass,
      centre: { x: 0, y: 1, z: 0 },
      width: 1,
      height: 1,
      thickness: 0.006,
    });
    const r = g.applyImpact(id, { x: 0, y: 1, z: 0 }, g.shatterImpulse(id) * 3);
    expect(r.shattered).toEqual([id]);
    expect(r.created.length).toBeGreaterThan(10);
    const islands = g.islands();
    // The rim stays in the frame; something in the middle comes away.
    expect(islands.some((i) => i.anchored)).toBe(true);
    expect(islands.some((i) => !i.anchored)).toBe(true);
    const volume = r.created.reduce((s, c) => s + g.pieces[c]!.volume, 0);
    expect(volume).toBeCloseTo(0.006, 6);
  });

  it('never breaks the indestructible cheese', () => {
    const g = new BondGraph(1);
    const id = buildPanel(g, {
      material: M.cheese,
      centre: { x: 0, y: 1, z: 0 },
      width: 1,
      height: 1,
      thickness: 0.2,
    });
    const r = g.applyImpact(id, { x: 0, y: 1, z: 0 }, 1e9);
    expect(r.broken + r.shattered.length).toBe(0);
  });
});

async function crash(material: keyof typeof M, kmh: number, unit = brickUnit) {
  const fw = await FractureWorld.create(1);
  fw.addGround(0);
  buildWall(fw.graph, { material: M[material], base: { x: 0, y: 0, z: 0 }, length: 6, height: 2.1, unit });
  fw.sync();
  const car = new CrashCar(fw, { rear: -8, speed: kmh / 3.6 });
  for (let i = 0; i < 360; i++) {
    car.preStep(FractureWorld.STEP);
    fw.step(FractureWorld.STEP);
    car.postStep(FractureWorld.STEP);
  }
  const loose = fw.livePieces().filter((p) => fw.isLoose(p.id)).length;
  const out = { front: car.front, crush: car.crush, peak: car.peakDecel, loose, car };
  fw.dispose();
  return out;
}

describe('crash car', () => {
  it('stops against a brick wall at 30 km/h with the front crumpled', async () => {
    const r = await crash('brick', 30);
    expect(r.front).toBeLessThan(0.05);
    expect(r.crush).toBeGreaterThan(0.1);
    expect(r.crush).toBeLessThan(0.4);
  }, 30_000);

  it('goes through a brick wall at 120 km/h and leaves a debris field', async () => {
    const r = await crash('brick', 120);
    expect(r.front).toBeGreaterThan(3);
    expect(r.loose).toBeGreaterThan(80);
  }, 30_000);

  it('cannot get through the indestructible cheese at any speed', async () => {
    const r = await crash('cheese', 120, { length: 1.5, height: 1, thickness: 0.8 });
    expect(r.loose).toBe(0);
    expect(r.front).toBeLessThan(0);
    expect(r.crush).toBeGreaterThan(0.9);
  }, 30_000);

  it('throws the belted dummies forward into the airbags', async () => {
    const r = await crash('brick', 50);
    const o = r.car.occupants[0]!;
    expect(r.car.airbagTime).not.toBeNull();
    expect(o.peakChest).toBeGreaterThan(10);
    expect(r.peak).toBeGreaterThan(10);
  }, 30_000);
});

async function shoot(target: 'glass' | 'plywood', speed: number, radius: number, mass: number) {
  const fw = await FractureWorld.create(2);
  fw.addGround(0);
  buildPanel(fw.graph, {
    material: M[target],
    centre: { x: 0, y: 1.2, z: 0 },
    width: 1.2,
    height: 1.2,
    thickness: target === 'glass' ? 0.006 : 0.018,
  });
  fw.sync();
  const rb = fw.addProjectile({
    shape: { kind: 'ball', radius },
    mass,
    position: { x: -3, y: 1.2, z: 0 },
    velocity: { x: speed, y: (9.81 * 3) / (2 * speed), z: 0 },
  });
  for (let i = 0; i < 90; i++) fw.step(FractureWorld.STEP);
  const x = rb.translation().x;
  const loose = fw.livePieces().filter((p) => fw.isLoose(p.id)).length;
  fw.dispose();
  return { through: x > 0.3, loose };
}

describe('projectiles', () => {
  it('bounces a slow tennis ball off glass and breaks through it when fast', async () => {
    expect(await shoot('glass', 10, 0.0335, 0.057)).toEqual({ through: false, loose: 0 });
    const fast = await shoot('glass', 25, 0.0335, 0.057);
    expect(fast.through).toBe(true);
    expect(fast.loose).toBeGreaterThan(0);
  });

  it('does not tunnel a fast golf ball through plywood (continuous collision)', async () => {
    expect((await shoot('plywood', 80, 0.0214, 0.0459)).through).toBe(false);
  });

  it('bursts a watermelon against a wall', async () => {
    const fw = await FractureWorld.create(4);
    fw.addGround(0);
    buildPanel(fw.graph, {
      material: M.cheese,
      centre: { x: 0, y: 1.2, z: 0 },
      width: 1.2,
      height: 1.2,
      thickness: 0.2,
    });
    const melon = buildScoredSphere(fw.graph, {
      material: M.melon,
      centre: { x: -2, y: 1.2, z: 0 },
      radius: 0.11,
      tag: 'melon',
    });
    fw.sync();
    expect(new Set(melon.map((id) => fw.bodyOfPiece(id))).size).toBe(1);
    for (const id of melon) fw.bodyOfPiece(id)!.setLinvel({ x: 25, y: 0.4, z: 0 }, true);
    for (let i = 0; i < 60; i++) fw.step(FractureWorld.STEP);
    expect(new Set(melon.map((id) => fw.bodyOfPiece(id))).size).toBeGreaterThan(4);
    fw.dispose();
  });

  it('is deterministic for a seed', async () => {
    const run = async () => {
      const fw = await FractureWorld.create(9);
      fw.addGround(0);
      buildPanel(fw.graph, {
        material: M.glass,
        centre: { x: 0, y: 1.2, z: 0 },
        width: 1.2,
        height: 1.2,
        thickness: 0.006,
      });
      fw.sync();
      fw.addProjectile({
        shape: { kind: 'ball', radius: 0.025 },
        mass: 0.51,
        position: { x: -3, y: 1.2, z: 0 },
        velocity: { x: 40, y: 0.37, z: 0 },
      });
      for (let i = 0; i < 60; i++) fw.step(FractureWorld.STEP);
      const poses = fw.livePieces().map((p) => fw.piecePose(p.id).position);
      fw.dispose();
      return poses;
    };
    expect(await run()).toEqual(await run());
  });
});
