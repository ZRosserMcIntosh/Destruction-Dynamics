/* Pole Dynamics — utility poles and overhead wires.
 * SPDX-License-Identifier: MIT
 *
 * The C ABI, for Unity (P/Invoke, unity/PoleDynamics.cs) and the browser
 * (WebAssembly, web/). Same pattern as OpenRoad Dynamics' ord_* and
 * Ballistic Dynamics' bd_* exports: plain functions with a prefix, POD
 * structs, caller-owned output buffers.
 *
 * Frame: everything here is the simulation frame, ENU (x east, y north,
 * z up, metres from MASP), right-handed. Unity is Y up and left-handed: the
 * C# wrapper converts at the boundary (swap y and z for points and vectors;
 * a rotation q = (w, x, y, z) becomes (w, -x, -z, -y)). See pd/unity_frame.hpp.
 *
 * Time: call pd_step with the game's fixed step (1/60 s). The world runs
 * 4 internal steps per call (240 Hz) and never allocates inside pd_step.
 *
 * Threads: a world is not thread-safe; use one thread per world.
 */
#ifndef PD_C_API_H
#define PD_C_API_H

#include <stdint.h>

#if defined(_WIN32)
#define PD_EXPORT __declspec(dllexport)
#elif defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#define PD_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define PD_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Event types (pd_poll_events). */
enum {
    PD_POLE_CRACKED = 1,
    PD_POLE_LEANING = 2,
    PD_POLE_SNAPPED = 3,
    PD_POLE_FELL = 4,
    PD_WIRE_SNAPPED = 5,
    PD_WIRE_ON_GROUND_LIVE = 6,  /* value: volts; ids: hazard radius in mm */
    PD_ARC = 7,                  /* value: volts */
    PD_POWER_OUT = 8,            /* ids: section (its transformer pole ID); value: poles dark */
    PD_LANE_BLOCKED = 9,         /* ids: packed lane ID (pd_unpack_lane); value: lane index */
    PD_TRANSFORMER_DETACHED = 10,
    PD_TRANSFORMER_LANDED = 11,  /* value: impact speed m/s; ids bit 1: oil leak, bit 2: fire */
    PD_CLAMP_FAILED = 12,        /* wire: conductor slot; value: N */
    PD_POLE_HIT_BODY = 13,       /* span: host body ID; value: impulse N s */
    PD_LIGHT_CHANGED = 14,       /* value: 1 lit, 0 dark */
    PD_POLE_REPAIRED = 15,
    PD_LANE_CLEARED = 16,
    PD_POWER_RESTORED = 17
};

/* Pole states. */
enum { PD_INTACT = 0, PD_CRACKED = 1, PD_LEANING = 2, PD_SNAPPED = 3, PD_FALLEN = 4 };

typedef struct PdEvent {
    int32_t type;
    int32_t pole;   /* pole ID (its seed) */
    int32_t span;   /* span index (or host body ID for PD_POLE_HIT_BODY) */
    int32_t wire;   /* wire index in the span, or conductor slot */
    double x, y, z;
    double value;
    int64_t ids;
} PdEvent;

typedef struct PdImpact {
    double px, py, pz;      /* contact point */
    double nx, ny, nz;      /* push direction (into the pole) */
    double vx, vy, vz;      /* car velocity relative to the pole */
    double vehicle_mass;    /* kg */
    double crush_stiffness; /* N/m, 0: default (400 kN/m) */
    double crush_plateau;   /* N, 0: none */
} PdImpact;

typedef struct PdImpactResult {
    int32_t pole;           /* -1: no pole within 1.5 m */
    int32_t state;
    int32_t broke;
    int32_t reserved;
    double impulse;         /* N s along the normal */
    double delta_v;         /* m/s the car loses along the normal */
    double peak_force;      /* N */
    double duration;        /* s */
} PdImpactResult;

typedef struct PdPiece {
    double px, py, pz;      /* bottom of the piece on its axis */
    double qw, qx, qy, qz;  /* body (x along the line, y left, z up) to world */
    double z0, z1;          /* heights along the pole the piece spans */
} PdPiece;

typedef struct PdPoleState {
    int32_t id;
    int32_t state;
    int32_t pieces;         /* 1 or 2 */
    int32_t transformer;    /* 0 none, 1 attached, 2 detached body */
    int32_t lit;
    int32_t powered;
    int32_t section;
    int32_t reserved;
    double tilt_deg;
    PdPiece piece[2];
    double tx, ty, tz;      /* transformer */
    double tqw, tqx, tqy, tqz;
} PdPoleState;

typedef struct PdCapsule {
    int32_t id;
    int32_t reserved;
    double ax, ay, az, bx, by, bz;
    double radius;
    double vx, vy, vz;
} PdCapsule;

typedef struct PdStats {
    int32_t poles, spans, awake_spans, frozen_spans, nodes, bodies;
    double last_step_ms, max_step_ms;
    int64_t steps;
} PdStats;

PD_EXPORT void* pd_create_world(uint32_t seed);
PD_EXPORT void pd_destroy_world(void* world);
PD_EXPORT const char* pd_error(void* world);

/* poles_<tile>.json bytes. Returns poles added, -1 on error. */
PD_EXPORT int32_t pd_load_tile(void* world, const char* json, int32_t length);
/* lanes T_<i>_<j>.json bytes. Returns road edges added. */
PD_EXPORT int32_t pd_load_lanes(void* world, const char* json, int32_t length);
PD_EXPORT void pd_unload_tile(void* world, int32_t tile_x, int32_t tile_y);

PD_EXPORT void pd_set_weather(void* world, double wetness);
/* Tunables by name (docs/README.md). Returns 0 if the name is unknown. */
PD_EXPORT int32_t pd_set_param(void* world, const char* name, double value);
PD_EXPORT double pd_get_param(void* world, const char* name);

/* Host bodies (cars) for the next steps: capsules. */
PD_EXPORT void pd_set_bodies(void* world, const PdCapsule* bodies, int32_t count);
PD_EXPORT int32_t pd_step(void* world, double dt);
PD_EXPORT int32_t pd_report_impact(void* world, const PdImpact* impact, PdImpactResult* result);

PD_EXPORT int32_t pd_query_state(void* world, int32_t pole_id, PdPoleState* out);
PD_EXPORT int32_t pd_pole_count(void* world);
PD_EXPORT int32_t pd_pole_id(void* world, int32_t index);
PD_EXPORT int32_t pd_span_count(void* world);
/* 0 analytic (draw the catenary), 1 awake, 2 frozen (draw the nodes). */
PD_EXPORT int32_t pd_span_mode(void* world, int32_t span);
PD_EXPORT void pd_span_info(void* world, int32_t span, int32_t* pole_a, int32_t* pole_b, double* max_tension);
/* Woken or frozen spans: [wires, per wire: kind, broken count, n, x,y,z * n, broken mask].
 * Returns floats written (negative: buffer too small, minus the size needed); 0 for analytic spans. */
PD_EXPORT int32_t pd_get_wire_nodes(void* world, int32_t span, float* out, int32_t capacity);
/* Same layout for any span, analytic ones sampled at `samples` points. */
PD_EXPORT int32_t pd_get_wire_curve(void* world, int32_t span, int32_t samples, float* out, int32_t capacity);
PD_EXPORT int32_t pd_poll_events(void* world, PdEvent* out, int32_t capacity);

/* The tile's diff (damaged poles, displaced spans). Returns bytes written, or -(bytes needed) if capacity is too small. */
PD_EXPORT int32_t pd_save_tile_state(void* world, int32_t tile_x, int32_t tile_y, uint8_t* out, int32_t capacity);
PD_EXPORT int32_t pd_load_tile_state(void* world, int32_t tile_x, int32_t tile_y, const uint8_t* data, int32_t size);
/* Repair crews: put a pole back up and re-string its spans. */
PD_EXPORT int32_t pd_repair_pole(void* world, int32_t pole_id);

PD_EXPORT void pd_stats(void* world, PdStats* out);
PD_EXPORT uint64_t pd_state_hash(void* world);
PD_EXPORT int32_t pd_pole_seed(double x, double y);
PD_EXPORT void pd_unpack_lane(int64_t id, int32_t* tile_x, int32_t* tile_y, int32_t* edge, int32_t* lane);

/* Scratch memory for hosts without their own allocator (the browser). */
PD_EXPORT void* pd_alloc(int32_t bytes);
PD_EXPORT void pd_free(void* p);

#ifdef __cplusplus
}
#endif

#endif /* PD_C_API_H */
