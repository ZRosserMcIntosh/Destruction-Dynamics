// Street Dynamics — destructible street furniture.
// SPDX-License-Identifier: MIT
#include "sd/c_api.h"

#include <cstdlib>
#include <string_view>

#include "sd/world.hpp"

using namespace sd;

namespace {
World* W(void* w) { return static_cast<World*>(w); }
void fill(const ObstacleHit& h, Real t, SdObstacle* out) {
    if (!out) return;
    out->hit = h.hit ? 1 : 0;
    out->id = h.id;
    out->depth = h.depth;
    out->nx = h.normal.x, out->ny = h.normal.y, out->nz = h.normal.z;
    out->yield_force = h.yieldForce;
    out->friction = h.friction;
    out->t = t;
}
}  // namespace

extern "C" {

SD_EXPORT void* sd_create_world(uint32_t seed) { return new World(seed); }
SD_EXPORT void sd_destroy_world(void* w) { delete W(w); }
SD_EXPORT const char* sd_error(void* w) { return W(w)->error().c_str(); }
SD_EXPORT int32_t sd_load_library(void* w, const char* json, int32_t length) {
    const std::string e = W(w)->loadLibrary(std::string_view(json, static_cast<std::size_t>(length)));
    return e.empty() ? W(w)->library.size() : -1;
}
SD_EXPORT int32_t sd_type_count(void* w) { return W(w)->library.size(); }
SD_EXPORT const char* sd_type_id(void* w, int32_t i) {
    return i >= 0 && i < W(w)->library.size() ? W(w)->library.at(i).id.c_str() : "";
}
SD_EXPORT int32_t sd_load_tile(void* w, const char* json, int32_t length) {
    return W(w)->loadTile(std::string_view(json, static_cast<std::size_t>(length)));
}
SD_EXPORT void sd_unload_tile(void* w, int32_t tx, int32_t ty) { W(w)->unloadTile(tx, ty); }
SD_EXPORT int32_t sd_add_object(void* w, const char* type, double x, double y, double heading, double groundZ) {
    const int i = W(w)->addObject(type, x, y, heading, groundZ);
    return i >= 0 ? W(w)->objectId(i) : -1;
}
SD_EXPORT void sd_set_weather(void* w, double wetness) { W(w)->wetness = clamp(wetness, 0.0, 1.0); }
SD_EXPORT void sd_set_bodies(void* w, const SdBody* b, int32_t count) {
    HostBody hb[64];
    const int n = count < 64 ? count : 64;
    for (int i = 0; i < n; ++i) {
        const SdBody& s = b[i];
        hb[i] = HostBody{s.id, {s.cx, s.cy, s.cz}, {s.qw, s.qx, s.qy, s.qz}, {s.hx, s.hy, s.hz}, {s.vx, s.vy, s.vz}, {s.wx, s.wy, s.wz}};
    }
    W(w)->setBodies(hb, n);
}
SD_EXPORT void sd_apply_impact(void* w, int32_t id, double px, double py, double pz, double nx, double ny, double nz,
                               double impulse) {
    W(w)->applyImpact(id, {px, py, pz}, {nx, ny, nz}, impulse);
}
SD_EXPORT int32_t sd_step(void* w, double dt) { return W(w)->step(dt); }
SD_EXPORT int32_t sd_any_near(void* w, double x, double y, double z, double r) { return W(w)->anyNear({x, y, z}, r) ? 1 : 0; }
SD_EXPORT void sd_obstacle_point(void* w, double px, double py, double pz, double margin, double vx, double vy, double vz,
                                 double stepTime, SdObstacle* out) {
    fill(W(w)->obstaclePoint({px, py, pz}, margin, {vx, vy, vz}, stepTime), 0, out);
}
SD_EXPORT void sd_obstacle_edge(void* w, double ax, double ay, double az, double bx, double by, double bz, double margin,
                                double vx, double vy, double vz, double stepTime, SdObstacle* out) {
    Real t = 0;
    const ObstacleHit h = W(w)->obstacleEdge({ax, ay, az}, {bx, by, bz}, margin, {vx, vy, vz}, stepTime, t);
    fill(h, t, out);
}
SD_EXPORT void sd_body_impulse(void* w, int32_t id, double* out3) {
    const Vec3 j = W(w)->bodyImpulse(id);
    if (out3) out3[0] = j.x, out3[1] = j.y, out3[2] = j.z;
}
SD_EXPORT int32_t sd_object_count(void* w) { return W(w)->objectCount(); }
SD_EXPORT int32_t sd_object_id(void* w, int32_t i) { return i >= 0 && i < W(w)->objectCount() ? W(w)->objectId(i) : -1; }
SD_EXPORT int32_t sd_query_object(void* w, int32_t id, SdObjectState* out) {
    const int i = W(w)->objectIndex(id);
    if (i < 0 || !out) return 0;
    out->id = id;
    out->state = static_cast<int32_t>(W(w)->objectState(i));
    out->type = W(w)->library.index(W(w)->objectType(i).id);
    out->awake = W(w)->objectAwake(i) ? 1 : 0;
    out->tilt_deg = W(w)->objectTilt(i) / kDeg;
    out->top_height = W(w)->objectTopHeight(i);
    out->plastic_rad = W(w)->objectMaxPlastic(i);
    out->stub_height = W(w)->objectStubHeight(i);
    return 1;
}
SD_EXPORT int32_t sd_object_parts(void* w, int32_t id, float* out, int32_t capacity) {
    const int i = W(w)->objectIndex(id);
    return i < 0 ? 0 : W(w)->parts(i, out, capacity);
}
SD_EXPORT int32_t sd_poll_events(void* w, SdEvent* out, int32_t capacity) {
    static_assert(sizeof(SdEvent) == sizeof(Event), "event layout");
    return W(w)->pollEvents(reinterpret_cast<Event*>(out), capacity);
}
SD_EXPORT int32_t sd_save_tile_state(void* w, int32_t tx, int32_t ty, uint8_t* out, int32_t cap) {
    return W(w)->saveTileState(tx, ty, out, cap);
}
SD_EXPORT int32_t sd_load_tile_state(void* w, int32_t tx, int32_t ty, const uint8_t* d, int32_t n) {
    return W(w)->loadTileState(tx, ty, d, n);
}
SD_EXPORT int32_t sd_repair(void* w, int32_t id) { return W(w)->repair(id) ? 1 : 0; }
SD_EXPORT void sd_stats(void* w, SdStats* out) {
    const Stats s = W(w)->stats();
    out->objects = s.objects, out->awake = s.awake, out->bodies = s.bodies, out->reserved = 0;
    out->last_step_ms = s.lastStepMs, out->max_step_ms = s.maxStepMs, out->steps = s.steps;
}
SD_EXPORT uint64_t sd_state_hash(void* w) { return W(w)->stateHash(); }
SD_EXPORT void* sd_alloc(int32_t bytes) { return std::malloc(static_cast<std::size_t>(bytes > 0 ? bytes : 1)); }
SD_EXPORT void sd_free(void* p) { std::free(p); }

}  // extern "C"
