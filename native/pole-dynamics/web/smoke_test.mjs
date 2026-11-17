// Pole Dynamics — WebAssembly smoke test: a street, an 80 km/h hit, events,
// and the state hash (compare with the native build: tools/pdhash).
// Usage: node web/smoke_test.mjs [web/dist] [tile.json out]
import { writeFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import path from 'node:path';

const dist = path.resolve(process.argv[2] ?? new URL('./dist', import.meta.url).pathname);
const { default: createPoleModule } = await import(pathToFileURL(path.join(dist, 'poles.mjs')));
const P = await createPoleModule();

// The tests' street: six residential poles 35 m apart, heading east, none with a transformer.
const poles = [], spans = [];
for (let i = 0; i < 6; ++i) {
  let x = i * 35, y = 0;
  for (let k = 0; k < 400; ++k) {
    const xx = Math.round((x + 0.1 * (k % 20) - 1.0) * 10) / 10, yy = Math.round((0.1 * Math.floor(k / 20)) * 10) / 10;
    if (P.poleSeed(xx, yy) % 100 >= 14) { x = xx; y = yy; break; }
  }
  poles.push([x, y, 90, 0]);
}
for (let i = 0; i + 1 < poles.length; ++i) spans.push([poles[i][0], poles[i][1], poles[i + 1][0], poles[i + 1][1]]);
const tile = JSON.stringify({ tile: 'T_p0_p0', v: 1, poles, spans });
if (process.argv[3]) writeFileSync(process.argv[3], tile);

const w = P.world(1);
const n = w.loadTile(tile);
if (n !== 6) throw new Error('loadTile: ' + n + ' ' + w.error());
for (let i = 0; i < 30; ++i) w.step();
const ids = w.poleIds();
const target = w.pole(ids[2]);
const v = 80 / 3.6, base = target.piece[0].position;
const r = w.impact({ point: [base[0], base[1] - 0.15, 0.5], normal: [0, 1, 0], velocity: [0, v, 0], mass: 1300 });
const t0 = performance.now();
for (let i = 0; i < 60 * 8; ++i) w.step();
const ms = (performance.now() - t0) / (60 * 8);
const ev = w.events();
const states = ids.map((id) => { const p = w.pole(id); return `${p.stateName}(${p.tiltDeg.toFixed(1)}°)`; });
console.log(`impact: peak ${(r.peakForce / 1000).toFixed(0)} kN, broke ${r.broke}`);
console.log('states:', states.join(' '));
console.log('events:', [...new Set(ev.map((e) => e.name))].join(', '));
console.log(`mean step ${ms.toFixed(3)} ms; hash ${w.hash()}`);
if (!r.broke) throw new Error('expected a snap at 80 km/h');
w.destroy();
