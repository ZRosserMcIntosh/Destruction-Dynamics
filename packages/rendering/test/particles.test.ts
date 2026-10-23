import { SeededRandom } from '@destruction-dynamics/core';
import { describe, expect, it } from 'vitest';
import { ParticleSim } from '../src/particleSim';

const env = { gravity: 9.81, waterHeight: () => 0, groundHeight: () => -50 };

describe('ParticleSim', () => {
  it('respects capacity and the governed limit', () => {
    const s = new ParticleSim(100);
    s.limit = 60;
    const n = s.emit('dust', 80, new SeededRandom(1), (p) => {
      p.y = 10;
      p.life = 5;
    });
    expect(n).toBe(60);
    expect(s.dropped).toBe(20);
  });

  it('kills spray entering the water and bubbles reaching the surface', () => {
    const s = new ParticleSim(10);
    const r = new SeededRandom(1);
    s.emit('spray', 1, r, (p) => {
      p.y = 1;
      p.vy = -5;
      p.life = 10;
    });
    s.emit('bubble', 1, r, (p) => {
      p.y = -0.5;
      p.vy = 2;
      p.life = 10;
    });
    s.emit('mist', 1, r, (p) => {
      p.y = 20;
      p.life = 10;
    });
    for (let i = 0; i < 30; i++) s.update(1 / 30, env);
    expect(s.countByKind()).toMatchObject({ spray: 0, bubble: 0, mist: 1 });
  });

  it('expires particles by age and compacts storage', () => {
    const s = new ParticleSim(50);
    s.emit('glass', 50, new SeededRandom(2), (p, r) => {
      p.y = 100;
      p.life = r.range(0.1, 1);
    });
    s.update(0.5, env);
    expect(s.count).toBeGreaterThan(0);
    expect(s.count).toBeLessThan(50);
    for (let i = 0; i < s.count; i++) expect(s.age[i]).toBeLessThan(s.life[i]!);
    s.update(1, env);
    expect(s.count).toBe(0);
  });

  it('is deterministic for a seed', () => {
    const run = () => {
      const s = new ParticleSim(200);
      s.emit('spray', 200, new SeededRandom(9), (p, r) => {
        p.vx = r.range(-5, 5);
        p.vy = r.range(5, 20);
        p.y = 1;
        p.life = 4;
      });
      for (let i = 0; i < 20; i++) s.update(1 / 20, env);
      return Array.from(s.px.slice(0, s.count));
    };
    expect(run()).toEqual(run());
  });
});
