// Street Dynamics — the coupling with OpenRoad Dynamics' vehicle.
// SPDX-License-Identifier: MIT
#include "vehicle_adapter.hpp"

#include <algorithm>

namespace sdtest {

namespace {
sd::Vec3 toSd(const ord::Vec3& v) { return {v.x, v.y, v.z}; }
ord::Vec3 toOrd(const sd::Vec3& v) { return {v.x, v.y, v.z}; }
ord::ObstacleContact toContact(const sd::ObstacleHit& h, bool rigid) {
    ord::ObstacleContact c;
    if (!h.hit) return c;
    c.hit = true;
    c.depth = h.depth;
    c.normal = toOrd(h.normal);
    c.friction = h.friction;
    c.id = h.id;
    c.yieldForce = rigid ? 0.0 : h.yieldForce;
    return c;
}
}  // namespace

void StreetObstacles::capture(const ord::Vehicle& v, ord::Real dt) {
    com_ = v.centerOfMassWorld();
    vel_ = v.state().velocity;
    omega_ = v.state().angularVelocity;
    dt_ = dt;
}

sd::Vec3 StreetObstacles::velocityAt(const ord::Vec3& p) const { return toSd(vel_ + ord::cross(omega_, p - com_)); }

bool StreetObstacles::anyNear(const ord::Vec3& c, ord::Real r) const { return w_.anyNear(toSd(c), r); }

ord::ObstacleContact StreetObstacles::point(const ord::Vec3& p, const ord::Vec3&, ord::Real margin) const {
    return toContact(w_.obstaclePoint(toSd(p), margin, velocityAt(p), dt_), rigid);
}

ord::ObstacleContact StreetObstacles::edge(const ord::Vec3& a, const ord::Vec3& b, ord::Real margin, ord::Real& t) const {
    sd::Real tt = 0;
    const sd::ObstacleHit h = w_.obstacleEdge(toSd(a), toSd(b), margin, velocityAt((a + b) * 0.5), dt_, tt);
    t = tt;
    return toContact(h, rigid);
}

int carBoxes(const ord::Vehicle& v, std::int32_t id, sd::HostBody* out) {
    const ord::DamageModel& d = v.damage();
    ord::Vec3 lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
    for (int i = 0; i < d.nodeCount(); ++i) {
        const ord::Vec3 r = d.node(i).rest;
        lo = {std::min(lo.x, r.x), std::min(lo.y, r.y), std::min(lo.z, r.z)};
        hi = {std::max(hi.x, r.x), std::max(hi.y, r.y), std::max(hi.z, r.z)};
    }
    // the bonnet line at 55 % of the height; the cabin is what stands above it
    const ord::Real split = lo.z + 0.55 * (hi.z - lo.z);
    ord::Vec3 clo{1e9, 1e9, split}, chi{-1e9, -1e9, hi.z};
    for (int i = 0; i < d.nodeCount(); ++i) {
        const ord::Vec3 r = d.node(i).rest;
        if (r.z <= split + 0.05) continue;
        clo.x = std::min(clo.x, r.x), clo.y = std::min(clo.y, r.y);
        chi.x = std::max(chi.x, r.x), chi.y = std::max(chi.y, r.y);
    }
    const ord::RigidBodyState& s = v.state();
    const ord::Vec3 com = v.centerOfMassWorld();
    auto box = [&](const ord::Vec3& a, const ord::Vec3& b, std::int32_t bid) {
        sd::HostBody h;
        h.id = bid;
        const ord::Vec3 cl = (a + b) * 0.5;
        const ord::Vec3 c = s.position + s.orientation.rotate(cl);
        h.centre = toSd(c);
        h.orientation = {s.orientation.w, s.orientation.x, s.orientation.y, s.orientation.z};
        h.half = toSd((b - a) * 0.5);
        h.velocity = toSd(s.velocity + ord::cross(s.angularVelocity, c - com));
        h.angularVelocity = toSd(s.angularVelocity);
        return h;
    };
    out[0] = box(lo, {hi.x, hi.y, split}, id);
    if (chi.x <= clo.x) return 1;
    out[1] = box(clo, chi, id + 1);
    return 2;
}

void applyImpulse(ord::Vehicle& v, const ord::Vec3& point, const ord::Vec3& impulse) {
    ord::RigidBodyState& s = v.mutableState();
    const ord::VehicleDefinition& d = v.definition();
    s.velocity += impulse * (1.0 / d.mass);
    const ord::Vec3 r = point - v.centerOfMassWorld();
    const ord::Vec3 tb = s.orientation.inverseRotate(ord::cross(r, impulse));
    s.angularVelocity += s.orientation.rotate({tb.x / d.inertia.x, tb.y / d.inertia.y, tb.z / d.inertia.z});
}

}  // namespace sdtest
