/* Pedestrian Dynamics — physically simulated people.
 * SPDX-License-Identifier: MIT
 *
 * The C ABI, for Unity (P/Invoke) and the browser (WebAssembly). Same pattern as Street
 * Dynamics' sd_*, Pole Dynamics' pd_* and OpenRoad's ord_*: plain functions, POD structs,
 * caller-owned buffers.
 *
 * Frame: ENU (x east, y north, z up, metres). Unity is Y up and left-handed: swap y and z for
 * points and vectors, and map a rotation (w, x, y, z) to (w, -x, -z, -y).
 *
 * Each step with vehicles:
 *   1. ped_set_vehicles with every car near a person (pose and velocity at the step's start);
 *   2. ped_step;
 *   3. ped_vehicle_impulse for each car, and add the impulse and its moment to the car's own
 *      rigid body (momentum balances between the two).
 */
#ifndef PED_C_API_H
#define PED_C_API_H

#include <stdint.h>

#if defined(_WIN32)
#define PED_EXPORT __declspec(dllexport)
#else
#define PED_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    PED_VEHICLE_CONTACT = 1, PED_HEAD_IMPACT = 2, PED_LANDED = 3, PED_AT_REST = 4, PED_LIGAMENT_TEAR = 5, PED_HIT = 6
};
enum {
    PED_BUMPER = 0, PED_GRILLE = 1, PED_BONNET = 2, PED_COWL = 3, PED_WINDSCREEN = 4, PED_A_PILLAR = 5,
    PED_ROOF_HEADER = 6, PED_ROOF = 7, PED_BODY = 8, PED_GROUND = 9
};
enum { PED_NONE = 0, PED_WRAP = 1, PED_FORWARD_PROJECTION = 2, PED_FENDER_VAULT = 3, PED_ROOF_VAULT = 4 };

typedef struct PedEvent {
    int32_t type, pedestrian, part, other; /* other: region (contacts), kinematics class (at rest), parent (tears) */
    double x, y, z, value, time;
} PedEvent;

typedef struct PedVehicle {
    int32_t id, profile;
    double px, py, pz;          /* the profile's origin: ground under the middle of the car */
    double qw, qx, qy, qz;
    double vx, vy, vz;          /* the origin's velocity */
    double wx, wy, wz;
} PedVehicle;

typedef struct PedReport {
    int32_t struck, vehicle, at_rest, trajectory;
    int32_t head_region, torn, reserved0, reserved1;
    double impact_speed, first_contact_time, throw_distance;
    double head_impact_speed, head_impact_time;
    double hic15, hic_vehicle, hic_ground, head_peak_g;
    double tibia_peak_g_left, tibia_peak_g_right, knee_bend_left_deg, knee_bend_right_deg;
    double max_pelvis_height, carry_time, air_time;
    double rest_x, rest_y, rest_z;
} PedReport;

typedef struct PedStats {
    int32_t pedestrians, awake;
    double last_step_ms, max_step_ms;
    int64_t steps;
} PedStats;

PED_EXPORT void* ped_create_world(uint32_t seed);
PED_EXPORT void ped_destroy_world(void* world);
PED_EXPORT const char* ped_error(void* world);
PED_EXPORT int32_t ped_load_body(void* world, const char* json, int32_t length);       /* segments, or -1 */
PED_EXPORT int32_t ped_load_vehicles(void* world, const char* json, int32_t length);   /* profiles, or -1 */
PED_EXPORT int32_t ped_profile_index(void* world, const char* id);
PED_EXPORT int32_t ped_segment_count(void* world);
PED_EXPORT const char* ped_segment_name(void* world, int32_t segment);

PED_EXPORT int32_t ped_add(void* world, double x, double y, double heading_rad, double stature, double mass, double ground_z);
PED_EXPORT int32_t ped_count(void* world);
PED_EXPORT int32_t ped_id(void* world, int32_t index);
PED_EXPORT void ped_set_ground(void* world, int32_t index, double z);
PED_EXPORT void ped_set_strength(void* world, int32_t index, double strength);
PED_EXPORT double ped_strength(void* world, int32_t index);
PED_EXPORT void ped_set_assist(void* world, int32_t index, double assist);
PED_EXPORT void ped_set_joint_target(void* world, int32_t index, int32_t segment, double flex, double abd, double twist);
PED_EXPORT void ped_apply_impulse(void* world, int32_t index, int32_t segment, double px, double py, double pz, double jx,
                                  double jy, double jz);

PED_EXPORT void ped_set_vehicles(void* world, const PedVehicle* vehicles, int32_t count);
PED_EXPORT int32_t ped_step(void* world, double dt);
PED_EXPORT void ped_vehicle_impulse(void* world, int32_t vehicle_index, double* linear3, double* angular3);

/* Per segment 14 floats: x y z, qw qx qy qz, capsule end a (local), end b (local), radius.
 * Returns floats written (negative: capacity too small). */
PED_EXPORT int32_t ped_parts(void* world, int32_t index, float* out, int32_t capacity);
PED_EXPORT int32_t ped_report(void* world, int32_t index, PedReport* out);
PED_EXPORT int32_t ped_poll_events(void* world, PedEvent* out, int32_t capacity);
PED_EXPORT void ped_stats(void* world, PedStats* out);
PED_EXPORT uint64_t ped_state_hash(void* world);
PED_EXPORT void* ped_alloc(int32_t bytes);
PED_EXPORT void ped_free(void* p);

#ifdef __cplusplus
}
#endif

#endif
