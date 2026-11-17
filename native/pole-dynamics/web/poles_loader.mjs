// Pole Dynamics — loader for the wasi-sdk build (web/build-wasm.sh).
// SPDX-License-Identifier: MIT
//
// const P = await createPoleModule(bytes?); const w = P.world(1); ...
// A thin wrapper over the C ABI (include/pd/c_api.h): it knows the struct
// layouts and keeps one scratch buffer per kind in the module's memory.
// Everything is in the sim frame: ENU, x east, y north, z up, metres.
export default async function createPoleModule(wasmBytes) {
  let bytes = wasmBytes;
  if (!bytes) {
    const url = new URL('./poles.wasm', import.meta.url);
    if (typeof process !== 'undefined' && process.versions?.node && url.protocol === 'file:') {
      const { readFile } = await import('node:fs/promises');
      bytes = await readFile(url);
    } else {
      bytes = await (await fetch(url)).arrayBuffer();
    }
  }
  let memory = null;
  const ENOSYS = 52;
  const wasi = new Proxy({
    proc_exit: (c) => { throw new Error('poles.wasm exited with code ' + c); },
    fd_write: (fd, iovs, n, nwritten) => {
      const v = new DataView(memory.buffer); let s = '', total = 0;
      for (let i = 0; i < n; ++i) { const p = v.getUint32(iovs + i * 8, true), l = v.getUint32(iovs + i * 8 + 4, true); s += new TextDecoder().decode(new Uint8Array(memory.buffer, p, l)); total += l; }
      v.setUint32(nwritten, total, true); if (s.trim()) (fd === 2 ? console.warn : console.log)(s.trimEnd()); return 0;
    },
    environ_sizes_get: (c, s) => { const v = new DataView(memory.buffer); v.setUint32(c, 0, true); v.setUint32(s, 0, true); return 0; },
    environ_get: () => 0,
    // Only used to time pd_step for the stats; the simulation never reads it.
    clock_time_get: (id, prec, out) => { new DataView(memory.buffer).setBigUint64(out, BigInt(Math.round(performance.now() * 1e6)), true); return 0; },
    random_get: () => ENOSYS,
  }, { get: (t, k) => t[k] ?? (() => ENOSYS) });
  const { instance } = await WebAssembly.instantiate(bytes, { wasi_snapshot_preview1: wasi });
  const ex = instance.exports;
  memory = ex.memory;
  ex._initialize?.();
  const enc = new TextEncoder(), dec = new TextDecoder();
  const dv = () => new DataView(memory.buffer);
  const cstr = (p) => { const u = new Uint8Array(memory.buffer, p); let n = 0; while (u[n]) ++n; return dec.decode(new Uint8Array(memory.buffer, p, n)); };
  // Scratch buffers in module memory, grown on demand.
  const scratch = {};
  const buf = (name, size) => {
    const s = scratch[name];
    if (s && s.size >= size) return s.ptr;
    if (s) ex.pd_free(s.ptr);
    const ptr = ex.pd_alloc(size);
    scratch[name] = { ptr, size };
    return ptr;
  };
  const str = (name, s) => { const b = enc.encode(s + '\0'); const p = buf(name, b.length); new Uint8Array(memory.buffer, p, b.length).set(b); return [p, b.length - 1]; };

  const EVENT_SIZE = 56, STATE_SIZE = 240, IMPACT_SIZE = 96, RESULT_SIZE = 48, STATS_SIZE = 48, CAPSULE_SIZE = 88;
  const EVENTS = ['', 'POLE_CRACKED', 'POLE_LEANING', 'POLE_SNAPPED', 'POLE_FELL', 'WIRE_SNAPPED', 'WIRE_ON_GROUND_LIVE', 'ARC',
    'POWER_OUT', 'LANE_BLOCKED', 'TRANSFORMER_DETACHED', 'TRANSFORMER_LANDED', 'CLAMP_FAILED', 'POLE_HIT_BODY', 'LIGHT_CHANGED',
    'POLE_REPAIRED', 'LANE_CLEARED', 'POWER_RESTORED'];
  const STATES = ['INTACT', 'CRACKED', 'LEANING', 'SNAPPED', 'FALLEN'];

  function world(seed = 1) {
    const w = ex.pd_create_world(seed >>> 0);
    const api = {
      ptr: w,
      destroy() { ex.pd_destroy_world(w); },
      error() { return cstr(ex.pd_error(w)); },
      loadTile(json) { const [p, n] = str('json', json); return ex.pd_load_tile(w, p, n); },
      loadLanes(json) { const [p, n] = str('json', json); return ex.pd_load_lanes(w, p, n); },
      unloadTile(tx, ty) { ex.pd_unload_tile(w, tx, ty); },
      setWetness(v) { ex.pd_set_weather(w, v); },
      setParam(name, v) { const [p] = str('name', name); return ex.pd_set_param(w, p, v) !== 0; },
      getParam(name) { const [p] = str('name', name); return ex.pd_get_param(w, p); },
      // [{id, a:[x,y,z], b:[x,y,z], radius, velocity:[x,y,z]}]
      setBodies(list) {
        const p = buf('caps', Math.max(1, list.length) * CAPSULE_SIZE), v = dv();
        list.forEach((c, i) => {
          const o = p + i * CAPSULE_SIZE;
          v.setInt32(o, c.id, true); v.setInt32(o + 4, 0, true);
          [...c.a, ...c.b, c.radius, ...(c.velocity ?? [0, 0, 0])].forEach((x, k) => v.setFloat64(o + 8 + 8 * k, x, true));
        });
        ex.pd_set_bodies(w, p, list.length);
      },
      step(dt = 1 / 60) { return ex.pd_step(w, dt); },
      // {point, normal, velocity, mass, crushStiffness}
      impact(i) {
        const p = buf('impact', IMPACT_SIZE + RESULT_SIZE), v = dv();
        [...i.point, ...i.normal, ...i.velocity, i.mass ?? 1300, i.crushStiffness ?? 0, 0].forEach((x, k) => v.setFloat64(p + 8 * k, x, true));
        ex.pd_report_impact(w, p, p + IMPACT_SIZE);
        const r = dv(), o = p + IMPACT_SIZE;
        return { pole: r.getInt32(o, true), state: r.getInt32(o + 4, true), broke: r.getInt32(o + 8, true) !== 0,
          impulse: r.getFloat64(o + 16, true), deltaV: r.getFloat64(o + 24, true), peakForce: r.getFloat64(o + 32, true),
          duration: r.getFloat64(o + 40, true) };
      },
      pole(id) {
        const p = buf('state', STATE_SIZE);
        if (!ex.pd_query_state(w, id, p)) return null;
        const v = dv(), i32 = (k) => v.getInt32(p + 4 * k, true), f = (o) => v.getFloat64(p + o, true);
        const piece = (o) => ({ position: [f(o), f(o + 8), f(o + 16)], rotation: [f(o + 24), f(o + 32), f(o + 40), f(o + 48)], z0: f(o + 56), z1: f(o + 64) });
        return { id: i32(0), state: i32(1), stateName: STATES[i32(1)], pieces: i32(2), transformer: i32(3), lit: i32(4) !== 0,
          powered: i32(5) !== 0, section: i32(6), tiltDeg: f(32), piece: [piece(40), piece(112)],
          transformerPosition: [f(184), f(192), f(200)], transformerRotation: [f(208), f(216), f(224), f(232)] };
      },
      poleIds() { const n = ex.pd_pole_count(w), out = []; for (let i = 0; i < n; ++i) out.push(ex.pd_pole_id(w, i)); return out; },
      spanCount() { return ex.pd_span_count(w); },
      spanMode(s) { return ex.pd_span_mode(w, s); },
      spanInfo(s) {
        const p = buf('info', 16);
        ex.pd_span_info(w, s, p, p + 4, p + 8);
        const v = dv();
        return { a: v.getInt32(p, true), b: v.getInt32(p + 4, true), tension: v.getFloat64(p + 8, true) };
      },
      // [{kind, broken (mask), points: Float32Array x,y,z...}] for any span (analytic ones sampled).
      wires(s, samples = 24) {
        let cap = 4096, p = buf('nodes', cap * 4), n = ex.pd_get_wire_curve(w, s, samples, p, cap);
        if (n < 0) { cap = -n; p = buf('nodes', cap * 4); n = ex.pd_get_wire_curve(w, s, samples, p, cap); }
        const a = new Float32Array(memory.buffer, p, n), out = [];
        let k = 0;
        const wires = a[k++];
        for (let i = 0; i < wires; ++i) {
          const kind = a[k++]; k++; const m = a[k++];
          const points = a.slice(k, k + 3 * m); k += 3 * m;
          out.push({ kind, broken: a[k++] >>> 0, points });
        }
        return out;
      },
      events() {
        const out = [];
        const p = buf('events', 256 * EVENT_SIZE);
        let n;
        while ((n = ex.pd_poll_events(w, p, 256)) > 0) {
          const v = dv();
          for (let i = 0; i < n; ++i) {
            const o = p + i * EVENT_SIZE, type = v.getInt32(o, true);
            out.push({ type, name: EVENTS[type] ?? '?', pole: v.getInt32(o + 4, true), span: v.getInt32(o + 8, true), wire: v.getInt32(o + 12, true),
              at: [v.getFloat64(o + 16, true), v.getFloat64(o + 24, true), v.getFloat64(o + 32, true)], value: v.getFloat64(o + 40, true),
              ids: v.getBigInt64(o + 48, true) });
          }
        }
        return out;
      },
      saveTile(tx, ty) {
        let n = ex.pd_save_tile_state(w, tx, ty, buf('save', 16), 16);
        if (n < 0) n = ex.pd_save_tile_state(w, tx, ty, buf('save', -n), -n);
        return new Uint8Array(memory.buffer, scratch.save.ptr, n).slice();
      },
      loadTileState(tx, ty, bytes) { const p = buf('save', bytes.length); new Uint8Array(memory.buffer, p, bytes.length).set(bytes); return ex.pd_load_tile_state(w, tx, ty, p, bytes.length); },
      repair(id) { return ex.pd_repair_pole(w, id) !== 0; },
      stats() {
        const p = buf('stats', STATS_SIZE); ex.pd_stats(w, p);
        const v = dv(), i = (k) => v.getInt32(p + 4 * k, true);
        return { poles: i(0), spans: i(1), awakeSpans: i(2), frozenSpans: i(3), nodes: i(4), bodies: i(5),
          lastStepMs: v.getFloat64(p + 24, true), maxStepMs: v.getFloat64(p + 32, true), steps: Number(v.getBigInt64(p + 40, true)) };
      },
      hash() { return BigInt.asUintN(64, ex.pd_state_hash(w)).toString(16).padStart(16, '0'); },
    };
    return api;
  }

  const unpackLane = (id) => {
    const p = ex.pd_alloc(16);
    ex.pd_unpack_lane(BigInt.asIntN(64, BigInt(id)), p, p + 4, p + 8, p + 12);
    const v = dv(), r = { tileX: v.getInt32(p, true), tileY: v.getInt32(p + 4, true), edge: v.getInt32(p + 8, true), lane: v.getInt32(p + 12, true) };
    ex.pd_free(p);
    return r;
  };

  return { world, poleSeed: (x, y) => ex.pd_pole_seed(x, y), unpackLane, EVENTS, STATES, exports: ex };
}
