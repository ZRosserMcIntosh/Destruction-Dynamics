// Street Dynamics — smoke test of the wasm build: a car-sized box at 50 km/h through a stop sign
// and into a steel bollard; prints what happened and the state hash.
// Usage: node web/smoke_test.mjs [dist-dir]
import { readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
const dist = resolve(process.argv[2] ?? new URL('./dist', import.meta.url).pathname);
const { default: create } = await import(join(dist, 'street.mjs'));
const S = await create(readFileSync(join(dist, 'street.wasm')));
const w = S.world(1);
if (w.loadLibrary(readFileSync(join(dist, 'street_library.json'), 'utf8')) < 9) throw new Error(w.error());
const sign = w.add('stop_sign_r1', 0, 0, 180);
const bollard = w.add('bollard_steel', 12, 0, 180);
const car = { id: 1, centre: [-3, 0, 0.6], half: [2.1, 0.88, 0.45], velocity: [50 / 3.6, 0, 0] };
const mass = 1300;
for (let k = 0; k < 240; ++k) {
  w.setBodies([car]);
  w.step(1 / 60);
  const J = w.bodyImpulse(1);
  car.velocity[0] = Math.max(0, car.velocity[0] + J[0] / mass);
  car.centre[0] += car.velocity[0] / 60;
}
const names = w.events().map((e) => e.name);
console.info(
  JSON.stringify({
    sign: w.object(sign).state,
    bollard: w.object(bollard).state,
    carKmh: +(car.velocity[0] * 3.6).toFixed(1),
    events: [...new Set(names)],
  }),
);
console.info('hash', w.hash());
if (w.object(sign).state === 'INTACT') process.exit(1);
