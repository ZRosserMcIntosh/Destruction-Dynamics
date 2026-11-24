// Street Dynamics — loader for the wasi-sdk build (web/build-wasm.sh).
// SPDX-License-Identifier: MIT
//
// const S = await createStreetModule(bytes?); const w = P.world(1); ...
// A thin wrapper over the C ABI (include/sd/c_api.h): it knows the struct layouts and keeps one
// scratch buffer per kind in the module's memory. Everything is in the sim frame: ENU, x east,
// y north, z up, metres.
export default async function createStreetModule(wasmBytes) {
  let bytes = wasmBytes;
  if (!bytes) {
    const url = new URL('./street.wasm', import.meta.url);
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
        throw new Error('street.wasm exited with code ' + c);
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
      // Only used to time sd_step for the stats; the simulation never reads it.
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
    if (s) ex.sd_free(s.ptr);
    const ptr = ex.sd_alloc(size);
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
    BODY_SIZE = 136,
    STATE_SIZE = 48,
    STATS_SIZE = 40;
  const EVENTS = [
    '',
    'OBJECT_HIT',
    'POST_BENT',
    'POST_DOWN',
    'POST_BROKEN',
    'BASE_RELEASED',
    'PANEL_DETACHED',
    'PANEL_LANDED',
    'DEBRIS_HIT_BODY',
    'REPAIRED',
  ];
  const STATES = ['INTACT', 'BENT', 'DOWN', 'BROKEN'];

  function world(seed = 1) {
    const w = ex.sd_create_world(seed >>> 0);
    const api = {
      ptr: w,
      destroy() {
        ex.sd_destroy_world(w);
      },
      error() {
        return cstr(ex.sd_error(w));
      },
      loadLibrary(json) {
        const [p, n] = str('json', json);
        return ex.sd_load_library(w, p, n);
      },
      types() {
        const out = [];
        for (let i = 0; i < ex.sd_type_count(w); ++i) out.push(cstr(ex.sd_type_id(w, i)));
        return out;
      },
      // returns the object's id, or -1
      add(type, x, y, headingDeg = 0, ground = 0) {
        const [p] = str('type', type);
        return ex.sd_add_object(w, p, x, y, headingDeg, ground);
      },
      setWetness(v) {
        ex.sd_set_weather(w, v);
      },
      // host vehicles as oriented boxes: [{id, centre:[x,y,z], orientation:[w,x,y,z], half:[x,y,z], velocity, angularVelocity}]
      setBodies(list) {
        const p = buf('bodies', Math.max(1, list.length) * BODY_SIZE),
          v = dv();
        list.forEach((b, i) => {
          const o = p + i * BODY_SIZE;
          v.setInt32(o, b.id, true);
          v.setInt32(o + 4, 0, true);
          [
            ...b.centre,
            ...(b.orientation ?? [1, 0, 0, 0]),
            ...b.half,
            ...(b.velocity ?? [0, 0, 0]),
            ...(b.angularVelocity ?? [0, 0, 0]),
          ].forEach((x, k) => v.setFloat64(o + 8 + 8 * k, x, true));
        });
        ex.sd_set_bodies(w, p, list.length);
      },
      step(dt = 1 / 60) {
        return ex.sd_step(w, dt);
      },
      // the impulse (N s, world) the objects put on host body `id` during the last step
      bodyImpulse(id) {
        const p = buf('bimp', 24);
        ex.sd_body_impulse(w, id, p);
        const v = dv();
        return [v.getFloat64(p, true), v.getFloat64(p + 8, true), v.getFloat64(p + 16, true)];
      },
      objectIds() {
        const out = [];
        for (let i = 0; i < ex.sd_object_count(w); ++i) out.push(ex.sd_object_id(w, i));
        return out;
      },
      object(id) {
        const p = buf('state', STATE_SIZE);
        if (!ex.sd_query_object(w, id, p)) return null;
        const v = dv(),
          i32 = (k) => v.getInt32(p + 4 * k, true),
          f = (k) => v.getFloat64(p + 16 + 8 * k, true);
        return {
          id: i32(0),
          state: STATES[i32(1)],
          type: i32(2),
          awake: i32(3) !== 0,
          tiltDeg: f(0),
          topHeight: f(1),
          plasticRad: f(2),
          stubHeight: f(3),
        };
      },
      // Float32Array, 13 per part: kind (0 segment, 1 panel, 2 bar), index, x y z, qw qx qy qz, half extents, loose
      parts(id) {
        let cap = 64 * 13,
          p = buf('parts', cap * 4),
          n = ex.sd_object_parts(w, id, p, cap);
        if (n < 0) {
          cap = -n;
          p = buf('parts', cap * 4);
          n = ex.sd_object_parts(w, id, p, cap);
        }
        return new Float32Array(memory.buffer, p, Math.max(0, n)).slice();
      },
      events() {
        const out = [];
        const p = buf('events', 128 * EVENT_SIZE);
        let n;
        while ((n = ex.sd_poll_events(w, p, 128)) > 0) {
          const v = dv();
          for (let i = 0; i < n; ++i) {
            const o = p + i * EVENT_SIZE,
              type = v.getInt32(o, true);
            out.push({
              type,
              name: EVENTS[type] ?? '?',
              object: v.getInt32(o + 4, true),
              part: v.getInt32(o + 8, true),
              at: [v.getFloat64(o + 16, true), v.getFloat64(o + 24, true), v.getFloat64(o + 32, true)],
              value: v.getFloat64(o + 40, true),
            });
          }
          if (n < 128) break;
        }
        return out;
      },
      repair(id) {
        return ex.sd_repair(w, id) !== 0;
      },
      stats() {
        const p = buf('stats', STATS_SIZE);
        ex.sd_stats(w, p);
        const v = dv();
        return {
          objects: v.getInt32(p, true),
          awake: v.getInt32(p + 4, true),
          lastStepMs: v.getFloat64(p + 16, true),
          maxStepMs: v.getFloat64(p + 24, true),
          steps: Number(v.getBigInt64(p + 32, true)),
        };
      },
      hash() {
        return BigInt.asUintN(64, ex.sd_state_hash(w)).toString(16).padStart(16, '0');
      },
    };
    return api;
  }

  return { world, EVENTS, STATES, exports: ex };
}
