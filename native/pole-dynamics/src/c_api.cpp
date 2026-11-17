// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// The C ABI over pd::World (include/pd/c_api.h).
#include "pd/c_api.h"

#include <cstdlib>
#include <cstring>
#include <string_view>

#include "pd/world.hpp"

using namespace pd;

namespace {

World* W(void* w) { return static_cast<World*>(w); }

// Tunables by name: one table, so Unity, the lab and the README agree.
Real* param(World& w, const char* name) {
    Params& p = w.params;
    struct Entry { const char* name; Real* ptr; };
    const Entry table[] = {
        {"soil.stiffness", &p.soil.stiffness},
        {"soil.yield", &p.soil.yieldMoment},
        {"soil.damping", &p.soil.damping},
        {"soil.fail_angle_deg", nullptr},
        {"soil.lean_angle_deg", nullptr},
        {"strength.ultimate_factor", &p.strength.ultimateFactor},
        {"strength.cracking_factor", &p.strength.crackingFactor},
        {"strength.weak_axis", &p.strength.weakAxisFactor},
        {"strength.residual", &p.strength.residualFactor},
        {"strength.rebar_tear", &p.strength.rebarTear},
        {"strength.variability", &p.strength.variability},
        {"strength.nominal.0", &p.strength.cls[0].nominalLoad},
        {"strength.nominal.1", &p.strength.cls[1].nominalLoad},
        {"strength.nominal.2", &p.strength.cls[2].nominalLoad},
        {"strength.nominal.3", &p.strength.cls[3].nominalLoad},
        {"strength.impact_crack.0", &p.strength.cls[0].impactCrack},
        {"strength.impact_ultimate.0", &p.strength.cls[0].impactUltimate},
        {"wire.mv.tension", &p.wires.type[0].tension},
        {"wire.mv.breaking", &p.wires.type[0].breaking},
        {"wire.mv.clamp", &p.wires.type[0].clampBreak},
        {"wire.lv.tension", &p.wires.type[1].tension},
        {"wire.lv.breaking", &p.wires.type[1].breaking},
        {"wire.lv.clamp", &p.wires.type[1].clampBreak},
        {"wire.telecom.tension", &p.wires.type[2].tension},
        {"wire.telecom.breaking", &p.wires.type[2].breaking},
        {"wire.telecom.clamp", &p.wires.type[2].clampBreak},
        {"wire.drop.clamp", &p.wires.type[3].clampBreak},
        {"wire.damping", &p.wires.damping},
        {"wire.recoil", &p.wires.recoil},
        {"sim.wake_displacement", &p.sim.wakeDisplacement},
        {"sim.car_crush_stiffness", &p.sim.carCrushStiffness},
        {"sim.hazard_radius", &p.sim.hazardRadius},
        {"sim.arc_interval", &p.sim.arcInterval},
        {"sim.mv_trip_time", &p.sim.mvTripTime},
        {"sim.lane_width", &p.sim.laneWidth},
        {"sim.wetness", &p.sim.wetness},
    };
    for (const Entry& e : table)
        if (std::strcmp(e.name, name) == 0) {
            if (e.ptr) return e.ptr;
            if (std::strcmp(name, "soil.fail_angle_deg") == 0) return &p.soil.failAngle;
            if (std::strcmp(name, "soil.lean_angle_deg") == 0) return &p.soil.leanAngle;
        }
    return nullptr;
}

bool isDegrees(const char* name) { return std::strstr(name, "_deg") != nullptr; }

}  // namespace

extern "C" {

PD_EXPORT void* pd_create_world(uint32_t seed) { return new World(seed); }
PD_EXPORT void pd_destroy_world(void* w) { delete W(w); }
PD_EXPORT const char* pd_error(void* w) { return W(w)->error().c_str(); }

PD_EXPORT int32_t pd_load_tile(void* w, const char* json, int32_t length) {
    return W(w)->loadTile(std::string_view(json, static_cast<std::size_t>(length)));
}
PD_EXPORT int32_t pd_load_lanes(void* w, const char* json, int32_t length) {
    return W(w)->loadLanes(std::string_view(json, static_cast<std::size_t>(length)));
}
PD_EXPORT void pd_unload_tile(void* w, int32_t tx, int32_t ty) { W(w)->unloadTile(tx, ty); }

PD_EXPORT void pd_set_weather(void* w, double wetness) { W(w)->params.sim.wetness = clamp(wetness, 0.0, 1.0); }

PD_EXPORT int32_t pd_set_param(void* w, const char* name, double value) {
    Real* p = param(*W(w), name);
    if (!p) return 0;
    *p = isDegrees(name) ? value * kDeg : value;
    return 1;
}
PD_EXPORT double pd_get_param(void* w, const char* name) {
    Real* p = param(*W(w), name);
    if (!p) return 0;
    return isDegrees(name) ? *p / kDeg : *p;
}

PD_EXPORT void pd_set_bodies(void* w, const PdCapsule* bodies, int32_t count) {
    Capsule caps[64];
    const int n = count < 64 ? count : 64;
    for (int i = 0; i < n; ++i) {
        const PdCapsule& c = bodies[i];
        caps[i] = Capsule{c.id, {c.ax, c.ay, c.az}, {c.bx, c.by, c.bz}, c.radius, {c.vx, c.vy, c.vz}};
    }
    W(w)->setBodies(caps, n);
}

PD_EXPORT int32_t pd_step(void* w, double dt) { return W(w)->step(dt); }

PD_EXPORT int32_t pd_report_impact(void* w, const PdImpact* in, PdImpactResult* out) {
    ImpactInput i;
    i.point = {in->px, in->py, in->pz};
    i.normal = {in->nx, in->ny, in->nz};
    i.relVelocity = {in->vx, in->vy, in->vz};
    i.vehicleMass = in->vehicle_mass > 0 ? in->vehicle_mass : 1300.0;
    i.crushStiffness = in->crush_stiffness;
    i.crushPlateau = in->crush_plateau;
    const ImpactResult r = W(w)->reportImpact(i);
    if (out) {
        out->pole = r.pole;
        out->state = r.state;
        out->broke = r.broke;
        out->reserved = 0;
        out->impulse = r.impulse;
        out->delta_v = r.vehicleDeltaV;
        out->peak_force = r.peakForce;
        out->duration = r.duration;
    }
    return r.pole;
}

PD_EXPORT int32_t pd_query_state(void* w, int32_t id, PdPoleState* out) {
    PoleQuery q;
    if (!W(w)->queryPole(id, q)) return 0;
    out->id = q.id;
    out->state = q.state;
    out->pieces = q.pieces;
    out->transformer = q.transformer;
    out->lit = q.lit;
    out->powered = q.powered;
    out->section = q.section;
    out->reserved = 0;
    out->tilt_deg = q.tiltDeg;
    for (int k = 0; k < 2; ++k) {
        const PieceTransform& t = q.piece[k];
        out->piece[k] = PdPiece{t.position.x, t.position.y, t.position.z, t.rotation.w, t.rotation.x,
                                t.rotation.y, t.rotation.z, t.z0, t.z1};
    }
    out->tx = q.transformerPosition.x;
    out->ty = q.transformerPosition.y;
    out->tz = q.transformerPosition.z;
    out->tqw = q.transformerRotation.w;
    out->tqx = q.transformerRotation.x;
    out->tqy = q.transformerRotation.y;
    out->tqz = q.transformerRotation.z;
    return 1;
}

PD_EXPORT int32_t pd_pole_count(void* w) { return W(w)->poleCount(); }
PD_EXPORT int32_t pd_pole_id(void* w, int32_t index) {
    return index >= 0 && index < W(w)->poleCount() ? W(w)->poleId(index) : -1;
}
PD_EXPORT int32_t pd_span_count(void* w) { return W(w)->spanCount(); }
PD_EXPORT int32_t pd_span_mode(void* w, int32_t span) {
    return span >= 0 && span < W(w)->spanCount() ? static_cast<int32_t>(W(w)->spanMode(span)) : -1;
}
PD_EXPORT void pd_span_info(void* w, int32_t span, int32_t* a, int32_t* b, double* t) {
    std::int32_t pa = -1, pb = -1;
    Real tension = 0;
    if (span >= 0 && span < W(w)->spanCount()) W(w)->spanInfo(span, pa, pb, tension);
    if (a) *a = pa;
    if (b) *b = pb;
    if (t) *t = tension;
}
PD_EXPORT int32_t pd_get_wire_nodes(void* w, int32_t span, float* out, int32_t capacity) {
    if (span < 0 || span >= W(w)->spanCount()) return 0;
    return W(w)->wireNodes(span, out, capacity);
}
PD_EXPORT int32_t pd_get_wire_curve(void* w, int32_t span, int32_t samples, float* out, int32_t capacity) {
    if (span < 0 || span >= W(w)->spanCount()) return 0;
    return W(w)->wireCurve(span, samples, out, capacity);
}
PD_EXPORT int32_t pd_poll_events(void* w, PdEvent* out, int32_t capacity) {
    static_assert(sizeof(PdEvent) == sizeof(Event), "event layout");
    return W(w)->pollEvents(reinterpret_cast<Event*>(out), capacity);
}

PD_EXPORT int32_t pd_save_tile_state(void* w, int32_t tx, int32_t ty, uint8_t* out, int32_t capacity) {
    return W(w)->saveTileState(tx, ty, out, capacity);
}
PD_EXPORT int32_t pd_load_tile_state(void* w, int32_t tx, int32_t ty, const uint8_t* data, int32_t size) {
    return W(w)->loadTileState(tx, ty, data, size);
}
PD_EXPORT int32_t pd_repair_pole(void* w, int32_t id) { return W(w)->repairPole(id) ? 1 : 0; }

PD_EXPORT void pd_stats(void* w, PdStats* out) {
    const Stats s = W(w)->stats();
    out->poles = s.poles;
    out->spans = s.spans;
    out->awake_spans = s.awakeSpans;
    out->frozen_spans = s.frozenSpans;
    out->nodes = s.nodes;
    out->bodies = s.bodies;
    out->last_step_ms = s.lastStepMs;
    out->max_step_ms = s.maxStepMs;
    out->steps = s.steps;
}
PD_EXPORT uint64_t pd_state_hash(void* w) { return W(w)->stateHash(); }
PD_EXPORT int32_t pd_pole_seed(double x, double y) { return poleSeed(x, y); }
PD_EXPORT void pd_unpack_lane(int64_t id, int32_t* tx, int32_t* ty, int32_t* edge, int32_t* lane) {
    const auto u = static_cast<uint64_t>(id);
    if (tx) *tx = static_cast<int16_t>((u >> 48) & 0xffff);
    if (ty) *ty = static_cast<int16_t>((u >> 32) & 0xffff);
    if (edge) *edge = static_cast<int32_t>((u >> 8) & 0xffffff);
    if (lane) *lane = static_cast<int32_t>(u & 0xff);
}

PD_EXPORT void* pd_alloc(int32_t bytes) { return std::malloc(static_cast<std::size_t>(bytes > 0 ? bytes : 1)); }
PD_EXPORT void pd_free(void* p) { std::free(p); }

}  // extern "C"
