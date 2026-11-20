// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
#include "ped/c_api.h"

#include <cstdlib>
#include <string_view>

#include "ped/world.hpp"

using namespace ped;

namespace {
World* W(void* w) { return static_cast<World*>(w); }
bool valid(void* w, int32_t i) { return i >= 0 && i < W(w)->pedestrianCount(); }
}  // namespace

extern "C" {

PED_EXPORT void* ped_create_world(uint32_t seed) { return new World(seed); }
PED_EXPORT void ped_destroy_world(void* w) { delete W(w); }
PED_EXPORT const char* ped_error(void* w) { return W(w)->error().c_str(); }
PED_EXPORT int32_t ped_load_body(void* w, const char* json, int32_t length) {
    const std::string e = W(w)->loadBody(std::string_view(json, static_cast<std::size_t>(length)));
    return e.empty() ? W(w)->body.size() : -1;
}
PED_EXPORT int32_t ped_load_vehicles(void* w, const char* json, int32_t length) {
    const std::string e = W(w)->loadVehicles(std::string_view(json, static_cast<std::size_t>(length)));
    return e.empty() ? W(w)->vehicles.size() : -1;
}
PED_EXPORT int32_t ped_profile_index(void* w, const char* id) { return W(w)->vehicles.index(id); }
PED_EXPORT int32_t ped_segment_count(void* w) { return W(w)->segmentCount(); }
PED_EXPORT const char* ped_segment_name(void* w, int32_t s) {
    return s >= 0 && s < W(w)->segmentCount() ? W(w)->segmentName(s).c_str() : "";
}

PED_EXPORT int32_t ped_add(void* w, double x, double y, double heading, double stature, double mass, double groundZ) {
    return W(w)->addPedestrian(x, y, heading, stature, mass, groundZ);
}
PED_EXPORT int32_t ped_count(void* w) { return W(w)->pedestrianCount(); }
PED_EXPORT int32_t ped_id(void* w, int32_t i) { return valid(w, i) ? W(w)->pedestrianId(i) : -1; }
PED_EXPORT void ped_set_ground(void* w, int32_t i, double z) {
    if (valid(w, i)) W(w)->setGroundHeight(i, z);
}
PED_EXPORT void ped_set_strength(void* w, int32_t i, double s) {
    if (valid(w, i)) W(w)->setStrength(i, s);
}
PED_EXPORT double ped_strength(void* w, int32_t i) { return valid(w, i) ? W(w)->strength(i) : 0.0; }
PED_EXPORT void ped_set_assist(void* w, int32_t i, double a) {
    if (valid(w, i)) W(w)->setAssist(i, a);
}
PED_EXPORT void ped_set_joint_target(void* w, int32_t i, int32_t s, double flex, double abd, double twist) {
    if (valid(w, i)) W(w)->setJointTarget(i, s, flex, abd, twist);
}
PED_EXPORT void ped_apply_impulse(void* w, int32_t i, int32_t s, double px, double py, double pz, double jx, double jy,
                                  double jz) {
    if (valid(w, i)) W(w)->applyImpulse(i, s, {px, py, pz}, {jx, jy, jz});
}

PED_EXPORT void ped_set_vehicles(void* w, const PedVehicle* v, int32_t count) {
    VehicleState vs[World::kMaxVehicles];
    const int n = count < World::kMaxVehicles ? count : World::kMaxVehicles;
    for (int i = 0; i < n; ++i) {
        const PedVehicle& s = v[i];
        vs[i].id = s.id;
        vs[i].profile = s.profile;
        vs[i].position = {s.px, s.py, s.pz};
        vs[i].orientation = {s.qw, s.qx, s.qy, s.qz};
        vs[i].velocity = {s.vx, s.vy, s.vz};
        vs[i].angularVelocity = {s.wx, s.wy, s.wz};
    }
    W(w)->setVehicles(vs, n);
}
PED_EXPORT int32_t ped_step(void* w, double dt) { return W(w)->step(dt); }
PED_EXPORT void ped_vehicle_impulse(void* w, int32_t index, double* linear3, double* angular3) {
    Vec3 J, L;
    W(w)->vehicleImpulse(index, J, L);
    if (linear3) linear3[0] = J.x, linear3[1] = J.y, linear3[2] = J.z;
    if (angular3) angular3[0] = L.x, angular3[1] = L.y, angular3[2] = L.z;
}

PED_EXPORT int32_t ped_parts(void* w, int32_t i, float* out, int32_t capacity) {
    return valid(w, i) ? W(w)->parts(i, out, capacity) : 0;
}
PED_EXPORT int32_t ped_report(void* w, int32_t i, PedReport* out) {
    if (!valid(w, i) || !out) return 0;
    const Report& r = W(w)->report(i);
    *out = PedReport{};
    out->struck = r.struck ? 1 : 0;
    out->vehicle = r.vehicle;
    out->at_rest = r.atRest ? 1 : 0;
    out->trajectory = static_cast<int32_t>(r.trajectory);
    out->head_region = r.headRegion;
    out->torn = static_cast<int32_t>(r.torn);
    out->impact_speed = r.impactSpeed;
    out->first_contact_time = r.firstContactTime;
    out->throw_distance = r.throwDistance;
    out->head_impact_speed = r.headImpactSpeed;
    out->head_impact_time = r.headImpactTime;
    out->hic15 = r.hic15;
    out->hic_vehicle = r.hicVehicle;
    out->hic_ground = r.hicGround;
    out->head_peak_g = r.headPeakG;
    out->tibia_peak_g_left = r.tibiaPeakG[0];
    out->tibia_peak_g_right = r.tibiaPeakG[1];
    out->knee_bend_left_deg = r.kneeBendPeakDeg[0];
    out->knee_bend_right_deg = r.kneeBendPeakDeg[1];
    out->max_pelvis_height = r.maxPelvisHeight;
    out->carry_time = r.carryTime;
    out->air_time = r.airTime;
    out->rest_x = r.restPosition.x, out->rest_y = r.restPosition.y, out->rest_z = r.restPosition.z;
    return 1;
}
PED_EXPORT int32_t ped_poll_events(void* w, PedEvent* out, int32_t capacity) {
    Event buf[64];
    int32_t total = 0;
    while (total < capacity) {
        const int want = capacity - total < 64 ? capacity - total : 64;
        const int n = W(w)->pollEvents(buf, want);
        for (int k = 0; k < n; ++k) {
            const Event& e = buf[k];
            out[total + k] = PedEvent{static_cast<int32_t>(e.type), e.pedestrian, e.part, e.other, e.at.x, e.at.y, e.at.z, e.value, e.time};
        }
        total += n;
        if (n < want) break;
    }
    return total;
}
PED_EXPORT void ped_stats(void* w, PedStats* out) {
    if (!out) return;
    const Stats& s = W(w)->stats();
    *out = PedStats{s.pedestrians, s.awake, s.lastStepMs, s.maxStepMs, s.steps};
}
PED_EXPORT uint64_t ped_state_hash(void* w) { return W(w)->stateHash(); }
PED_EXPORT void* ped_alloc(int32_t bytes) { return std::malloc(static_cast<std::size_t>(bytes > 0 ? bytes : 1)); }
PED_EXPORT void ped_free(void* p) { std::free(p); }

}  // extern "C"
