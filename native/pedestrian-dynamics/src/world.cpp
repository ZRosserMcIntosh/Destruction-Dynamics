// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
#include "ped/world.hpp"

#include <algorithm>
#include <cmath>

#include "ped/limits.hpp"
#include "ped/rot.hpp"

namespace ped {

namespace {

const char* const kTrajectoryNames[] = {"none", "wrap", "forward projection", "fender vault", "roof vault"};

// Closest points of two segments p1-q1, p2-q2: parameters s, t (Ericson, Real-Time Collision
// Detection 5.1.9).
void closestSegments(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Real& s, Real& t) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const Real a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    if (a <= 1e-12 && e <= 1e-12) { s = t = 0; return; }
    if (a <= 1e-12) { s = 0; t = clamp(f / e, 0, 1); return; }
    const Real c = dot(d1, r);
    if (e <= 1e-12) { t = 0; s = clamp(-c / a, 0, 1); return; }
    const Real b = dot(d1, d2), denom = a * e - b * b;
    s = denom > 1e-12 ? clamp((b * f - c * e) / denom, 0, 1) : 0;
    t = (b * s + f) / e;
    if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
    else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
}

}  // namespace

const char* trajectoryName(Trajectory t) {
    const int i = static_cast<int>(t);
    return i >= 0 && i <= 4 ? kTrajectoryNames[i] : "?";
}

World::World(std::uint32_t seed) : seed_(seed) {}

void World::resolveSegments() {
    pelvisSeg_ = 0;
    thoraxSeg_ = body.index("thorax");
    headSeg_ = body.index("head");
    shankSeg_[0] = body.index("shank_l");
    shankSeg_[1] = body.index("shank_r");
}

std::int32_t World::pedestrianId(int i) const { return peds_[static_cast<std::size_t>(i)].id; }

void World::removeAll() {
    peds_.clear();
    evHead_ = evCount_ = 0;
}

int World::addPedestrian(Real x, Real y, Real heading, Real stature, Real mass, Real groundZ) {
    if (body.size() == 0) {
        error_ = "no body model loaded";
        return -1;
    }
    resolveSegments();
    const BuiltBody bb = buildBody(body, stature, mass);
    Pedestrian p;
    p.id = nextId_++;
    p.count = bb.count;
    p.heading = heading;
    p.groundZ = groundZ;
    p.stature = stature;
    p.mass = mass;
    // per-person variation of the tissues' tear strengths, +/- 10 %
    std::uint32_t v = seed_ * 2654435761u ^ static_cast<std::uint32_t>(p.id) * 2246822519u;
    v ^= v >> 15, v *= 2246822519u, v ^= v >> 13;
    p.variation = v;
    const Quat yaw = axisAngle({0, 0, 1}, heading);
    p.assistTarget = yaw;
    const Vec3 origin{x, y, groundZ};
    for (int s = 0; s < bb.count; ++s) {
        const BuiltSegment& bs = bb.seg[static_cast<std::size_t>(s)];
        Body& b = p.body[static_cast<std::size_t>(s)];
        b = Body{};
        b.x = b.xPrev = origin + rotate(yaw, bs.centre);
        b.q = b.qPrev = yaw;
        b.settleX = b.x;
        b.settleQ = b.q;
        b.invMass = 1.0 / bs.mass;
        b.invInertia = {1.0 / bs.inertia.x, 1.0 / bs.inertia.y, 1.0 / bs.inertia.z};
        b.la = bs.a - bs.centre;
        b.lb = bs.b - bs.centre;
        b.radius = bs.radius;
        b.halfLength = bs.halfLength;
        b.tissueCompliance = 1.0 / bs.tissueStiffness;
        if (s == 0) continue;
        const SegmentDef& d = body.at(s);
        Joint& j = p.joint[static_cast<std::size_t>(s)];
        j = Joint{};
        j.parent = bs.parent;
        j.child = s;
        const BuiltSegment& par = bb.seg[static_cast<std::size_t>(bs.parent)];
        j.rp = bs.jointAt - par.centre;
        j.rc = bs.jointAt - bs.centre;
        j.t = d.joint.twistAxis;
        j.a = d.joint.flexAxis;
        j.b = d.joint.abdAxis;
        j.flexMin = d.joint.flexMin, j.flexMax = d.joint.flexMax;
        j.abdMin = d.joint.abdMin, j.abdMax = d.joint.abdMax;
        j.twistMin = d.joint.twistMin, j.twistMax = d.joint.twistMax;
        j.kp = bs.kp;
        j.torque = bs.torque;
        j.limitCompliance = 1.0 / std::max(bs.limitStiffness, 1.0);
        j.tearTorque = bs.tearTorque * (0.9 + 0.2 * static_cast<Real>((v >> (s % 16)) % 1000u) / 999.0);
        j.inertia = std::max(bs.inertia.x, std::max(bs.inertia.y, bs.inertia.z));
    }
    // Self-collision: every pair but parent-child and the pairs that already overlap standing.
    for (int i = 0; i < bb.count; ++i)
        for (int k = i + 1; k < bb.count; ++k) {
            const BuiltSegment &a = bb.seg[static_cast<std::size_t>(i)], &b = bb.seg[static_cast<std::size_t>(k)];
            if (a.parent == k || b.parent == i) continue;
            Real s = 0, t = 0;
            closestSegments(a.a, a.b, b.a, b.b, s, t);
            const Real d = length((a.a + (a.b - a.a) * s) - (b.a + (b.b - b.a) * t));
            if (d < a.radius + b.radius + 0.005 * stature) continue;
            p.pairs[static_cast<std::size_t>(p.pairCount++)] = {static_cast<std::uint8_t>(i), static_cast<std::uint8_t>(k)};
        }
    p.report.maxPelvisHeight = p.body[0].x.z - groundZ;
    peds_.push_back(p);
    stats_.pedestrians = static_cast<int>(peds_.size());
    return static_cast<int>(peds_.size()) - 1;
}

void World::setGroundHeight(int i, Real z) {
    Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    p.groundZ = z;
    wake(p);
}

void World::setStrength(int i, Real s) {
    Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    p.strength = clamp(s, 0, 1);
    wake(p);
}
Real World::strength(int i) const { return peds_[static_cast<std::size_t>(i)].strength; }
void World::setAssist(int i, Real a) {
    Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    p.assist = clamp(a, 0, 1);
    wake(p);
}

void World::setJointTarget(int i, int segment, Real flex, Real abd, Real twist) {
    Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    if (segment <= 0 || segment >= p.count) return;
    Joint& j = p.joint[static_cast<std::size_t>(segment)];
    // muscles pull within the joint's range (past it is the ligaments' business)
    clampToRange(flex, abd, twist, j.flexMin, j.flexMax, j.abdMin, j.abdMax, j.twistMin, j.twistMax);
    j.target = fromRotationVector(j.a * flex + j.b * abd) * axisAngle(j.t, twist);
    wake(p);
}

void World::setVehicles(const VehicleState* v, int count) {
    vehCount_ = std::min(std::max(count, 0), kMaxVehicles);
    for (int k = 0; k < vehCount_; ++k) {
        veh_[static_cast<std::size_t>(k)] = v[k];
        if (v[k].profile < 0 || v[k].profile >= vehicles.size()) veh_[static_cast<std::size_t>(k)].profile = -1;
    }
}

void World::vehicleImpulse(int index, Vec3& linear, Vec3& angular) const {
    if (index < 0 || index >= vehCount_) {
        linear = angular = {};
        return;
    }
    linear = vehJ_[static_cast<std::size_t>(index)];
    angular = vehL_[static_cast<std::size_t>(index)];
}

void World::applyImpulse(int i, int segment, const Vec3& point, const Vec3& impulse) {
    Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    if (segment < 0 || segment >= p.count) return;
    Body& b = p.body[static_cast<std::size_t>(segment)];
    b.v += impulse * b.invMass;
    b.w += invInertiaWorld(b, cross(point - b.x, impulse));
    // a blow takes strength away: half of it for a hard one (a 9 mm round is ~3 N s)
    const Real j = length(impulse);
    p.strength = std::max(limpStrength, p.strength * (1.0 - clamp(j / 40.0, 0, 0.5)));
    wake(p);
    emit(EventType::Hit, p, segment, -1, point, j);
}

void World::wake(Pedestrian& p) {
    if (p.awake) return;
    p.awake = true;
    p.still = 0;
}

void World::emit(EventType type, const Pedestrian& p, int part, int other, const Vec3& at, Real value) {
    Event& e = events_[static_cast<std::size_t>((evHead_ + evCount_) % static_cast<int>(events_.size()))];
    e = Event{type, p.id, part, other, at, value, subTime_};
    if (evCount_ < static_cast<int>(events_.size())) ++evCount_;
    else evHead_ = (evHead_ + 1) % static_cast<int>(events_.size());
}

int World::pollEvents(Event* out, int capacity) {
    int n = 0;
    while (evCount_ > 0 && n < capacity) {
        out[n++] = events_[static_cast<std::size_t>(evHead_)];
        evHead_ = (evHead_ + 1) % static_cast<int>(events_.size());
        --evCount_;
    }
    return n;
}

Vec3 World::segmentPosition(int i, int s) const { return peds_[static_cast<std::size_t>(i)].body[static_cast<std::size_t>(s)].x; }
Quat World::segmentOrientation(int i, int s) const { return peds_[static_cast<std::size_t>(i)].body[static_cast<std::size_t>(s)].q; }
Vec3 World::segmentVelocity(int i, int s) const { return peds_[static_cast<std::size_t>(i)].body[static_cast<std::size_t>(s)].v; }
Real World::segmentMass(int i, int s) const { return 1.0 / peds_[static_cast<std::size_t>(i)].body[static_cast<std::size_t>(s)].invMass; }
bool World::awake(int i) const { return peds_[static_cast<std::size_t>(i)].awake; }
const Report& World::report(int i) const { return peds_[static_cast<std::size_t>(i)].report; }

Vec3 World::centreOfMass(int i) const {
    const Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    Vec3 c;
    Real m = 0;
    for (int s = 0; s < p.count; ++s) {
        const Body& b = p.body[static_cast<std::size_t>(s)];
        c += b.x / b.invMass;
        m += 1.0 / b.invMass;
    }
    return c / m;
}

Vec3 World::momentum(int i) const {
    const Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    Vec3 m;
    for (int s = 0; s < p.count; ++s) m += p.body[static_cast<std::size_t>(s)].v / p.body[static_cast<std::size_t>(s)].invMass;
    return m;
}

Vec3 World::jointAngles(int i, int segment) const {
    const Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    if (segment <= 0 || segment >= p.count) return {};
    const Joint& j = p.joint[static_cast<std::size_t>(segment)];
    const Quat r = conjugate(p.body[static_cast<std::size_t>(j.parent)].q) * p.body[static_cast<std::size_t>(j.child)].q;
    Quat swing;
    const Real tw = swingTwist(r, j.t, swing);
    const Vec3 phi = rotationVector(swing);
    return {dot(phi, j.a), dot(phi, j.b), tw};
}

int World::parts(int i, float* out, int capacity) const {
    const Pedestrian& p = peds_[static_cast<std::size_t>(i)];
    const int need = p.count * 14;
    if (capacity < need) return -need;
    for (int s = 0; s < p.count; ++s) {
        const Body& b = p.body[static_cast<std::size_t>(s)];
        float* o = out + s * 14;
        const Real vals[14] = {b.x.x, b.x.y, b.x.z, b.q.w, b.q.x, b.q.y, b.q.z, b.la.x, b.la.y, b.la.z, b.lb.x, b.lb.y, b.lb.z, b.radius};
        for (int k = 0; k < 14; ++k) o[k] = static_cast<float>(vals[k]);
    }
    return need;
}

std::uint64_t World::stateHash() const {
    Hash64 h;
    for (const Pedestrian& p : peds_)
        for (int s = 0; s < p.count; ++s) {
            h.add(p.body[static_cast<std::size_t>(s)].x);
            h.add(p.body[static_cast<std::size_t>(s)].q);
        }
    return h.h;
}

}  // namespace ped
