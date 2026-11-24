// Pedestrian Dynamics — loader for the wasi-sdk build (web/build-wasm.sh).
// SPDX-License-Identifier: MIT
//
// const P = await createPedModule(bytes?); const w = P.world(1); ...
// A thin wrapper over the C ABI (include/ped/c_api.h): it knows the struct layouts and keeps one
// scratch buffer per kind in the module's memory. Everything is in the sim frame: ENU, x east,
// y north, z up, metres.
export default async function createPedModule(wasmBytes) {
  let bytes = wasmBytes;
  if (!bytes) {
    const url = new URL('./ped.wasm', import.meta.url);
    if (typeof process !== 'undefined' && process.versions?.node && url.protocol === 'file:') {
      const { readFile } = await import('node:fs/promises');
      bytes = await readFile(url);
    } else {
      bytes = await (await fetch(url)).arrayBuffer();
    }
  }
  let memory = null;
  const ENOSYS = 52;
  const wasi = new Proxy(
    {
      proc_exit: (c) => {
        throw new Error('ped.wasm exited with code ' + c);
      },
      fd_write: (fd, iovs, n, nwritten) => {
        const v = new DataView(memory.buffer);
        let s = '',
          total = 0;
        for (let i = 0; i < n; ++i) {
          const p = v.getUint32(iovs + i * 8, true),
            l = v.getUint32(iovs + i * 8 + 4, true);
          s += new TextDecoder().decode(new Uint8Array(memory.buffer, p, l));
          total += l;
        }
        v.setUint32(nwritten, total, true);
        if (s.trim()) (fd === 2 ? console.warn : console.info)(s.trimEnd());
        return 0;
      },
      environ_sizes_get: (c, s) => {
        const v = new DataView(memory.buffer);
        v.setUint32(c, 0, true);
        v.setUint32(s, 0, true);
        return 0;
      },
      environ_get: () => 0,
      // Only used to time ped_step for the stats; the simulation never reads it.
      clock_time_get: (id, prec, out) => {
        new DataView(memory.buffer).setBigUint64(out, BigInt(Math.round(performance.now() * 1e6)), true);
        return 0;
      },
      random_get: () => ENOSYS,
    },
    { get: (t, k) => t[k] ?? (() => ENOSYS) },
  );
  const { instance } = await WebAssembly.instantiate(bytes, { wasi_snapshot_preview1: wasi });
  const ex = instance.exports;
  memory = ex.memory;
  ex._initialize?.();
  const enc = new TextEncoder(),
    dec = new TextDecoder();
  const dv = () => new DataView(memory.buffer);
  const cstr = (p) => {
    const u = new Uint8Array(memory.buffer, p);
    let n = 0;
    while (u[n]) ++n;
    return dec.decode(new Uint8Array(memory.buffer, p, n));
  };
  const scratch = {};
  const buf = (name, size) => {
    const s = scratch[name];
    if (s && s.size >= size) return s.ptr;
    if (s) ex.ped_free(s.ptr);
    const ptr = ex.ped_alloc(size);
    scratch[name] = { ptr, size };
    return ptr;
  };
  const str = (name, s) => {
    const b = enc.encode(s + '\0');
    const p = buf(name, b.length);
    new Uint8Array(memory.buffer, p, b.length).set(b);
    return [p, b.length - 1];
  };

  const EVENT_SIZE = 56,
    VEHICLE_SIZE = 112,
    REPORT_SIZE = 184,
    STATS_SIZE = 32;
  const EVENTS = ['', 'VEHICLE_CONTACT', 'HEAD_IMPACT', 'LANDED', 'AT_REST', 'LIGAMENT_TEAR', 'HIT'];
  const REGIONS = [
    'bumper',
    'grille',
    'bonnet',
    'cowl',
    'windscreen',
    'A-pillar',
    'roof header',
    'roof',
    'body',
    'ground',
  ];
  const TRAJECTORIES = ['none', 'wrap', 'forward projection', 'fender vault', 'roof vault'];

  function world(seed = 1) {
    const w = ex.ped_create_world(seed >>> 0);
    const api = {
      ptr: w,
      destroy() {
        ex.ped_destroy_world(w);
      },
      error() {
        return cstr(ex.ped_error(w));
      },
      loadBody(json) {
        const [p, n] = str('json', json);
        return ex.ped_load_body(w, p, n);
      },
      loadVehicles(json) {
        const [p, n] = str('json', json);
        return ex.ped_load_vehicles(w, p, n);
      },
      profile(id) {
        const [p] = str('id', id);
        return ex.ped_profile_index(w, p);
      },
      segmentCount() {
        return ex.ped_segment_count(w);
      },
      segmentName(s) {
        return cstr(ex.ped_segment_name(w, s));
      },
      add({ x = 0, y = 0, heading = 0, stature = 1.75, mass = 75, ground = 0 } = {}) {
        return ex.ped_add(w, x, y, heading, stature, mass, ground);
      },
      count() {
        return ex.ped_count(w);
      },
      setGround(i, z) {
        ex.ped_set_ground(w, i, z);
      },
      setStrength(i, s) {
        ex.ped_set_strength(w, i, s);
      },
      strength(i) {
        return ex.ped_strength(w, i);
      },
      setAssist(i, a) {
        ex.ped_set_assist(w, i, a);
      },
      setJointTarget(i, segment, flex, abd = 0, twist = 0) {
        ex.ped_set_joint_target(w, i, segment, flex, abd, twist);
      },
      impulse(i, segment, point, J) {
        ex.ped_apply_impulse(w, i, segment, point[0], point[1], point[2], J[0], J[1], J[2]);
      },
      // [{id, profile, position:[x,y,z], orientation:[w,x,y,z], velocity:[..], angularVelocity:[..]}]
      setVehicles(list) {
        const p = buf('veh', Math.max(1, list.length) * VEHICLE_SIZE),
          v = dv();
        list.forEach((c, i) => {
          const o = p + i * VEHICLE_SIZE;
          v.setInt32(o, c.id, true);
          v.setInt32(o + 4, c.profile, true);
          [
            ...c.position,
            ...(c.orientation ?? [1, 0, 0, 0]),
            ...(c.velocity ?? [0, 0, 0]),
            ...(c.angularVelocity ?? [0, 0, 0]),
          ].forEach((x, k) => v.setFloat64(o + 8 + 8 * k, x, true));
        });
        ex.ped_set_vehicles(w, p, list.length);
      },
      step(dt = 1 / 60) {
        return ex.ped_step(w, dt);
      },
      // {linear:[x,y,z], angular:[x,y,z]} the people put on vehicle `index` (in setVehicles order)
      vehicleImpulse(index) {
        const p = buf('vimp', 48);
        ex.ped_vehicle_impulse(w, index, p, p + 24);
        const v = dv(),
          f = (k) => v.getFloat64(p + 8 * k, true);
        return { linear: [f(0), f(1), f(2)], angular: [f(3), f(4), f(5)] };
      },
      // Float32Array, 14 per segment: x y z, qw qx qy qz, end a (local), end b (local), radius
      parts(i) {
        let cap = 32 * 14,
          p = buf('parts', cap * 4),
          n = ex.ped_parts(w, i, p, cap);
        if (n < 0) {
          cap = -n;
          p = buf('parts', cap * 4);
          n = ex.ped_parts(w, i, p, cap);
        }
        return new Float32Array(memory.buffer, p, n).slice();
      },
      velocity(i, segment) {
        const p = buf('vel', 24);
        if (!ex.ped_segment_velocity(w, i, segment, p)) return [0, 0, 0];
        const v = dv();
        return [v.getFloat64(p, true), v.getFloat64(p + 8, true), v.getFloat64(p + 16, true)];
      },
      report(i) {
        const p = buf('report', REPORT_SIZE);
        if (!ex.ped_report(w, i, p)) return null;
        const v = dv(),
          i32 = (k) => v.getInt32(p + 4 * k, true),
          f = (k) => v.getFloat64(p + 32 + 8 * k, true);
        return {
          struck: i32(0) !== 0,
          vehicle: i32(1),
          atRest: i32(2) !== 0,
          trajectory: TRAJECTORIES[i32(3)] ?? '?',
          headRegion: i32(4) >= 0 ? REGIONS[i32(4)] : null,
          torn: i32(5) >>> 0,
          impactSpeed: f(0),
          firstContactTime: f(1),
          throwDistance: f(2),
          headImpactSpeed: f(3),
          headImpactTime: f(4),
          hic15: f(5),
          hicVehicle: f(6),
          hicGround: f(7),
          headPeakG: f(8),
          tibiaG: [f(9), f(10)],
          kneeBendDeg: [f(11), f(12)],
          maxPelvisHeight: f(13),
          carryTime: f(14),
          airTime: f(15),
          rest: [f(16), f(17), f(18)],
        };
      },
      events() {
        const out = [];
        const p = buf('events', 128 * EVENT_SIZE);
        let n;
        while ((n = ex.ped_poll_events(w, p, 128)) > 0) {
          const v = dv();
          for (let i = 0; i < n; ++i) {
            const o = p + i * EVENT_SIZE,
              type = v.getInt32(o, true),
              other = v.getInt32(o + 12, true);
            out.push({
              type,
              name: EVENTS[type] ?? '?',
              pedestrian: v.getInt32(o + 4, true),
              part: v.getInt32(o + 8, true),
              other,
              region: type === 1 || type === 2 ? REGIONS[other] : undefined,
              at: [v.getFloat64(o + 16, true), v.getFloat64(o + 24, true), v.getFloat64(o + 32, true)],
              value: v.getFloat64(o + 40, true),
              time: v.getFloat64(o + 48, true),
            });
          }
          if (n < 128) break;
        }
        return out;
      },
      stats() {
        const p = buf('stats', STATS_SIZE);
        ex.ped_stats(w, p);
        const v = dv();
        return {
          pedestrians: v.getInt32(p, true),
          awake: v.getInt32(p + 4, true),
          lastStepMs: v.getFloat64(p + 8, true),
          maxStepMs: v.getFloat64(p + 16, true),
          steps: Number(v.getBigInt64(p + 24, true)),
        };
      },
      hash() {
        return BigInt.asUintN(64, ex.ped_state_hash(w)).toString(16).padStart(16, '0');
      },
    };
    return api;
  }

  return { world, EVENTS, REGIONS, TRAJECTORIES, exports: ex };
}
