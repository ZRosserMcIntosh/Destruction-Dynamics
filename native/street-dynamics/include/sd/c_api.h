/* Street Dynamics — destructible street furniture.
 * SPDX-License-Identifier: MIT
 *
 * The C ABI, for Unity (P/Invoke) and the browser (WebAssembly). Same pattern
 * as Pole Dynamics' pd_*, OpenRoad's ord_* and Ballistic Dynamics' bd_*:
 * plain functions, POD structs, caller-owned buffers.
 *
 * Frame: the simulation frame, ENU (x east, y north, z up, metres). Unity is
 * Y up and left-handed: swap y and z for points and vectors, and map a
 * rotation (w, x, y, z) to (w, -x, -z, -y), as Pole Dynamics' wrapper does.
 *
 * Coupling with the vehicle (OpenRoad Dynamics): each step,
 *   1. sd_set_bodies with the cars as boxes (pose and velocity at the step's start);
 *   2. step the vehicle; its ObstacleQuery calls sd_obstacle_point / sd_obstacle_edge
 *      and passes yield_force on (ObstacleContact::yieldForce): posts give way;
 *   3. sd_step: the posts are pushed, bent and dragged by the boxes.
 */
#ifndef SD_C_API_H
#define SD_C_API_H

#include <stdint.h>

#if defined(SD_EXPORT)
#elif defined(_WIN32)
#define SD_EXPORT __declspec(dllexport)
#else
#define SD_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SD_OBJECT_HIT = 1, SD_POST_BENT = 2, SD_POST_DOWN = 3, SD_POST_BROKEN = 4, SD_BASE_RELEASED = 5,
    SD_PANEL_DETACHED = 6, SD_PANEL_LANDED = 7, SD_DEBRIS_HIT_BODY = 8, SD_REPAIRED = 9
};
enum { SD_INTACT = 0, SD_BENT = 1, SD_DOWN = 2, SD_BROKEN = 3 };

typedef struct SdEvent {
    int32_t type, object, part, reserved;
    double x, y, z, value;
    int64_t ids;
} SdEvent;

typedef struct SdBody {   /* a host vehicle as an oriented box */
    int32_t id, reserved;
    double cx, cy, cz;
    double qw, qx, qy, qz;
    double hx, hy, hz;    /* half extents, body frame */
    double vx, vy, vz;
    double wx, wy, wz;
} SdBody;

typedef struct SdObstacle {
    int32_t hit, id;
    double depth;
    double nx, ny, nz;    /* out of the obstacle */
    double yield_force;   /* N: pass to ObstacleContact::yieldForce */
    double friction;
    double t;             /* edge queries: parameter along the edge */
} SdObstacle;

typedef struct SdObjectState {
    int32_t id, state, type, awake;
    double tilt_deg, top_height, plastic_rad, stub_height;
} SdObjectState;

typedef struct SdStats {
    int32_t objects, awake, bodies, reserved;
    double last_step_ms, max_step_ms;
    int64_t steps;
} SdStats;

SD_EXPORT void* sd_create_world(uint32_t seed);
SD_EXPORT void sd_destroy_world(void* world);
SD_EXPORT const char* sd_error(void* world);
SD_EXPORT int32_t sd_load_library(void* world, const char* json, int32_t length);
SD_EXPORT int32_t sd_type_count(void* world);
SD_EXPORT const char* sd_type_id(void* world, int32_t index);
/* street_<tile>.json: {"tile":"T_i_j","v":1,"objects":[[type, x, y, heading_deg], ...]} */
SD_EXPORT int32_t sd_load_tile(void* world, const char* json, int32_t length);
SD_EXPORT void sd_unload_tile(void* world, int32_t tile_x, int32_t tile_y);
SD_EXPORT int32_t sd_add_object(void* world, const char* type, double x, double y, double heading_deg, double ground_z);
SD_EXPORT void sd_set_weather(void* world, double wetness);
SD_EXPORT void sd_set_bodies(void* world, const SdBody* bodies, int32_t count);
/* A vehicle with its own crash model hit object `id`: the impulse (N s) it put on it this step,
 * at (px, py, pz), along -(nx, ny, nz) (the normal points out of the object). Applied at the
 * start of the next sd_step. */
SD_EXPORT void sd_apply_impact(void* world, int32_t id, double px, double py, double pz, double nx, double ny, double nz,
                               double impulse);
SD_EXPORT int32_t sd_step(void* world, double dt);
SD_EXPORT int32_t sd_any_near(void* world, double x, double y, double z, double radius);
SD_EXPORT void sd_obstacle_point(void* world, double px, double py, double pz, double margin, double vx, double vy, double vz,
                                 double step_time, SdObstacle* out);
SD_EXPORT void sd_obstacle_edge(void* world, double ax, double ay, double az, double bx, double by, double bz, double margin,
                                double vx, double vy, double vz, double step_time, SdObstacle* out);
SD_EXPORT void sd_body_impulse(void* world, int32_t body_id, double* out3);
SD_EXPORT int32_t sd_object_count(void* world);
SD_EXPORT int32_t sd_object_id(void* world, int32_t index);
SD_EXPORT int32_t sd_query_object(void* world, int32_t id, SdObjectState* out);
/* Per part 13 floats: kind (0 post segment, 1 panel, 2 bar), index, x y z, qw qx qy qz,
 * half extents (segment: r, r, half length; panel: half width, half thickness, half height), loose.
 * Returns floats written (negative: capacity too small). */
SD_EXPORT int32_t sd_object_parts(void* world, int32_t id, float* out, int32_t capacity);
SD_EXPORT int32_t sd_poll_events(void* world, SdEvent* out, int32_t capacity);
SD_EXPORT int32_t sd_save_tile_state(void* world, int32_t tile_x, int32_t tile_y, uint8_t* out, int32_t capacity);
SD_EXPORT int32_t sd_load_tile_state(void* world, int32_t tile_x, int32_t tile_y, const uint8_t* data, int32_t size);
SD_EXPORT int32_t sd_repair(void* world, int32_t id);
SD_EXPORT void sd_stats(void* world, SdStats* out);
SD_EXPORT uint64_t sd_state_hash(void* world);
SD_EXPORT void* sd_alloc(int32_t bytes);
SD_EXPORT void sd_free(void* p);

#ifdef __cplusplus
}
#endif
#endif
