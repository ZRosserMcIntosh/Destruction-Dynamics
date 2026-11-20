// Pedestrian Dynamics — smoke test of the wasm build: a sedan at 40 km/h strikes a person
// crossing; prints the report and the state hash (the native build gives the same hash).
// Usage: node web/smoke_test.mjs [dist-dir]
import { readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
const dist = resolve(process.argv[2] ?? new URL('./dist', import.meta.url).pathname);
const { default: create } = await import(join(dist, 'ped.mjs'));
const P = await create(readFileSync(join(dist, 'ped.wasm')));
const w = P.world(1);
if (w.loadBody(readFileSync(join(dist, 'pedestrian_body.json'), 'utf8')) !== 16) throw new Error(w.error());
if (w.loadVehicles(readFileSync(join(dist, 'vehicle_profiles.json'), 'utf8')) < 4) throw new Error(w.error());
const i = w.add({ heading: Math.PI / 2 });
w.setAssist(i, 1);
const car = { id: 7, profile: w.profile('sedan'), position: [-2.55, 0, 0], velocity: [40 / 3.6, 0, 0] };
let J = 0;
const t0 = performance.now();
for (let k = 0; k < 600; ++k) {
  w.setVehicles([car]);
  w.step(1 / 60);
  J += w.vehicleImpulse(0).linear[0];
  car.position[0] += car.velocity[0] / 60;
}
const ms = (performance.now() - t0) / 600;
const r = w.report(i);
const events = w.events().map((e) => e.name + (e.region ? ':' + e.region : ''));
console.log(
  JSON.stringify({
    trajectory: r.trajectory,
    throw: +r.throwDistance.toFixed(2),
    head: r.headRegion,
    headSpeed: +r.headImpactSpeed.toFixed(2),
    hicVehicle: Math.round(r.hicVehicle),
    carImpulse: Math.round(J),
    events: events.slice(0, 6),
  }),
);
console.log('hash', w.hash(), 'ms/step', ms.toFixed(3));
if (!r.struck || !r.atRest || r.trajectory !== 'wrap') process.exit(1);
