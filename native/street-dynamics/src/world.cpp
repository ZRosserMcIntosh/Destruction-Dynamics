// Street Dynamics — destructible street furniture.
// SPDX-License-Identifier: MIT
//
// Objects, tiles, events, queries and persistence. The solver is in solver.cpp.
#include "sd/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "sd/dmath.hpp"
#include "sd/json.hpp"

namespace sd {

namespace {

std::int64_t pythonRound(Real v) {
    // Round half to even on the exact binary value, as Python's round() does.
    const Real f = std::floor(v);
    const Real d = v - f;
    Real r;
    if (d > 0.5) r = f + 1.0;
    else if (d < 0.5) r = f;
    else r = (std::fmod(f, 2.0) == 0.0) ? f : f + 1.0;
    return static_cast<std::int64_t>(r);
}

Quat axisAngle(const Vec3& axis, Real angle) {
    Real s, c;
    dm::sincos(0.5 * angle, s, c);
    return {c, axis.x * s, axis.y * s, axis.z * s};
}

constexpr Real kTile = 500.0;
int tileOf(Real v) { return static_cast<int>(std::floor((v + 0.5 * kTile) / kTile)); }
std::int64_t tileKey(int tx, int ty) {
    return (static_cast<std::int64_t>(tx) << 32) ^ static_cast<std::int64_t>(static_cast<std::uint32_t>(ty));
}
constexpr Real kCell = 8.0;

}  // namespace

std::int32_t objectSeed(Real x, Real y) {
    const auto a = static_cast<std::uint64_t>(pythonRound(x * 10.0));
    const auto b = static_cast<std::uint64_t>(pythonRound(y * 10.0));
    const std::uint64_t h = (a * 73856093ull) ^ (b * 19349663ull);
    return static_cast<std::int32_t>(h & 0x7fffffffull);
}

World::World(std::uint32_t seed) : seed_(seed) {
    contacts_.reserve(512);
    impacts_.reserve(256);
}

void World::index(int oi) {
    const Object& o = objects_[static_cast<std::size_t>(oi)];
    const auto cx = static_cast<std::int64_t>(std::floor(o.x / kCell)), cy = static_cast<std::int64_t>(std::floor(o.y / kCell));
    grid_[(cx << 32) ^ (cy & 0xffffffff)].push_back(oi);
}

int World::addObject(std::string_view type, Real x, Real y, Real headingDeg, Real groundZ) {
    const int t = library.index(type);
    if (t < 0) {
        error_ = "unknown type " + std::string(type);
        return -1;
    }
    const std::int32_t id = objectSeed(x, y);
    if (const auto it = byId_.find(id); it != byId_.end()) {
        Object& o = objects_[static_cast<std::size_t>(it->second)];
        o.active = true;
        return it->second;
    }
    Object o;
    o.id = id;
    o.type = t;
    o.x = x;
    o.y = y;
    o.heading = headingDeg * kDeg;
    o.groundZ = groundZ;
    o.tile = static_cast<int>(tileKey(tileOf(x), tileOf(y)) & 0x7fffffff);
    // Each object a little different (its welds, its steel, how its footing was poured): +/- 10 %.
    o.variation = static_cast<std::uint32_t>(id) * 2654435761u ^ seed_;
    const int oi = static_cast<int>(objects_.size());
    objects_.push_back(o);
    build(objects_.back(), oi);
    byId_[id] = oi;
    index(oi);
    awake_.reserve(objects_.size());
    plain_.reserve(objects_.size());
    frames_.reserve(objects_.size());
    stats_.objects = static_cast<int>(objects_.size());
    return oi;
}

// The object as rigid bodies and joints (a fresh build: also the repair).
void World::build(Object& o, int oi) {
    const Archetype& a = library.at(o.type);
    const PostMechanics& m = a.mech;
    const bool fresh = o.bodyCount == 0;
    const int legs = a.legs, n = a.segments;
    const int bodiesNeeded = legs * n + (legs > 1 ? 1 : 0) + static_cast<int>(a.panels.size());
    const int jointsNeeded = legs * n + (legs > 1 ? 2 : 0) + static_cast<int>(a.panels.size());
    if (fresh) {
        o.firstBody = static_cast<int>(bodies_.size());
        o.firstJoint = static_cast<int>(joints_.size());
        o.bodyCount = bodiesNeeded;
        o.jointCount = jointsNeeded;
        bodies_.resize(bodies_.size() + static_cast<std::size_t>(bodiesNeeded));
        joints_.resize(joints_.size() + static_cast<std::size_t>(jointsNeeded));
    }
    o.segmentsPerLeg = n;
    o.state = ObjectState::Intact;
    o.hitReported = o.bentReported = o.downReported = o.brokenReported = false;
    o.panelLanded = {};
    o.stub = 0;
    o.still = 0;
    o.contactForce = o.lastContactForce = 0;
    // Frame: x along the heading (the way the sign faces), y to its left, z up.
    Real sh, ch;
    dm::sincos(o.heading, sh, ch);
    const Vec3 fwd{sh, ch, 0}, up{0, 0, 1};
    const Quat qObj = axisAngle(up, 0.5 * kPi - o.heading);  // local x -> fwd
    const Real seg = a.height / n;
    const Real r = 0.5 * a.section.d;
    const Real segMass = m.massPerMetre * seg;
    // A thin-walled tube: transverse I = m (l^2 / 12 + r^2 / 2), axial m r^2 (solid: half that).
    const bool solid = a.section.kind == SectionKind::SolidRound || a.section.kind == SectionKind::SolidSquare;
    const Real iT = segMass * (seg * seg / 12.0 + 0.5 * r * r), iA = segMass * r * r * (solid ? 0.5 : 1.0);
    // The joints' elastic bending: a beam of E I between segment centres (l apart).
    const Real hingeCompliance = seg / std::max(m.bendStiffness, 1.0);
    const Real baseCompliance = 0.5 * seg / std::max(m.bendStiffness, 1.0) +
                                (a.base.footingStiffness > 0 ? 1.0 / a.base.footingStiffness : 0.0);
    int bi = o.firstBody, ji = o.firstJoint;
    std::array<int, 2> topSeg{-1, -1};
    for (int leg = 0; leg < legs; ++leg) {
        const Real along = legs > 1 ? (leg == 0 ? -0.5 : 0.5) * a.legSpacing : 0.0;
        const Vec3 base{o.x + fwd.x * along, o.y + fwd.y * along, o.groundZ};
        for (int k = 0; k < n; ++k, ++bi) {
            Body& b = bodies_[static_cast<std::size_t>(bi)];
            b = Body{};
            b.kind = 0;
            b.object = oi;
            b.half = {r, r, 0.5 * seg};
            b.x = b.xPrev = base + up * ((k + 0.5) * seg);
            b.q = b.qPrev = qObj;
            b.invMass = 1.0 / segMass;
            b.invInertia = {1.0 / iT, 1.0 / iT, 1.0 / iA};
            Joint& j = joints_[static_cast<std::size_t>(ji++)];
            j = Joint{};
            j.a = bi;
            j.ra = {0, 0, -0.5 * seg};
            if (k == 0) {
                j.kind = 1;
                j.b = -1;
                j.rb = base;
                j.rest = conjugate(qObj);
                j.compliance = baseCompliance;
            } else {
                // seg k joined to seg k - 1 at their shared end: q_b = q_a * rest
                j.a = bi - 1;
                j.b = bi;
                j.ra = {0, 0, 0.5 * seg};
                j.rb = {0, 0, -0.5 * seg};
                j.rest = Quat{};
                j.kind = 0;
                j.compliance = hingeCompliance;
            }
        }
        topSeg[static_cast<std::size_t>(leg)] = bi - 1;
    }
    if (legs > 1) {
        // The top bar of a frame: a capsule along the heading between the legs' tops.
        Body& bar = bodies_[static_cast<std::size_t>(bi)];
        bar = Body{};
        bar.kind = 2;
        bar.object = oi;
        const Real len = a.legSpacing, barMass = m.massPerMetre * len;
        bar.half = {r, r, 0.5 * len};
        bar.x = bar.xPrev = Vec3{o.x, o.y, o.groundZ + a.height};
        const Quat qBar = qObj * axisAngle({0, 1, 0}, 0.5 * kPi);  // local z -> fwd
        bar.q = bar.qPrev = qBar;
        bar.invMass = 1.0 / barMass;
        const Real ib = barMass * (len * len / 12.0 + 0.5 * r * r);
        bar.invInertia = {1.0 / ib, 1.0 / ib, 1.0 / (barMass * r * r)};
        for (int leg = 0; leg < 2; ++leg) {
            Joint& j = joints_[static_cast<std::size_t>(ji++)];
            j = Joint{};
            j.kind = 3;
            j.a = topSeg[static_cast<std::size_t>(leg)];
            j.b = bi;
            j.ra = {0, 0, 0.5 * seg};
            j.rb = {0, 0, (leg == 0 ? -0.5 : 0.5) * len};
            j.rest = conjugate(qObj) * qBar;
            j.compliance = hingeCompliance;
        }
        ++bi;
    }
    for (std::size_t p = 0; p < a.panels.size(); ++p, ++bi) {
        const PanelDef& pd = a.panels[p];
        Body& b = bodies_[static_cast<std::size_t>(bi)];
        b = Body{};
        b.kind = 1;
        b.object = oi;
        b.panel = static_cast<int>(p);
        b.half = {0.5 * pd.width, 0.5 * pd.thickness, 0.5 * pd.height};
        const Quat qp = qObj * axisAngle(up, pd.yaw - 0.5 * kPi);  // local y (the face's normal) -> fwd
        const Vec3 normal = rotate(qp, {0, 1, 0});
        const Real hc = clamp(pd.centre, 0.0, a.height);
        b.x = b.xPrev = Vec3{o.x, o.y, o.groundZ + hc} + normal * pd.offset;
        b.q = b.qPrev = qp;
        // a plate cut to its shape: its mass from the shape's area, its inertia as the rectangle's
        const Real mass = std::max(panelArea(pd) * pd.thickness * pd.density, 0.05);
        b.invMass = 1.0 / mass;
        const Real W = pd.width, H = pd.height, T = pd.thickness;
        b.invInertia = {12.0 / (mass * (T * T + H * H)), 12.0 / (mass * (W * W + H * H)), 12.0 / (mass * (W * W + T * T))};
        const int k = std::clamp(static_cast<int>(hc / seg), 0, n - 1);
        const int sb = o.firstBody + k;
        const Body& s = bodies_[static_cast<std::size_t>(sb)];
        Joint& j = joints_[static_cast<std::size_t>(ji++)];
        j = Joint{};
        j.kind = 2;
        j.panel = static_cast<int>(p);
        j.a = sb;
        j.b = bi;
        const Vec3 onAxis{o.x, o.y, o.groundZ + hc};
        j.ra = rotateInv(s.q, onAxis - s.x);
        j.rb = rotateInv(qp, onAxis - b.x);
        j.rest = conjugate(s.q) * qp;
        j.compliance = 2e-6;     // clamps and plate: stiff
        j.posCompliance = 1e-7;
    }
}

int World::objectIndex(std::int32_t id) const {
    const auto it = byId_.find(id);
    return it == byId_.end() ? -1 : it->second;
}

int World::loadTile(std::string_view text) {
    error_.clear();
    const json::ParseResult pr = json::parse(std::string(text));
    if (!pr.ok()) {
        error_ = pr.error;
        return -1;
    }
    const json::Value* objs = pr.value.find("objects");
    if (!objs || !objs->isArray()) {
        error_ = "objects missing";
        return -1;
    }
    const int before = static_cast<int>(objects_.size());
    std::vector<int> touched;
    for (const json::Value& row : objs->items()) {
        if (!row.isArray() || row.items().size() < 4 || !row.items()[0].isString()) continue;
        const auto& r = row.items();
        const Real z = r.size() > 4 && r[4].isNumber() ? r[4].asNumber() : 0.0;
        const int oi = addObject(r[0].asString(), r[1].asNumber(), r[2].asNumber(), r[3].asNumber(), z);
        if (oi >= 0) touched.push_back(oi);
    }
    // A tile seen before this session: its damage comes back.
    if (!touched.empty()) {
        const Object& o0 = objects_[static_cast<std::size_t>(touched[0])];
        const int tx = tileOf(o0.x), ty = tileOf(o0.y);
        if (const auto it = tileStore_.find(tileKey(tx, ty)); it != tileStore_.end()) {
            const std::vector<std::uint8_t> d = it->second;
            loadTileState(tx, ty, d.data(), static_cast<int>(d.size()));
        }
    }
    stats_.bodies = static_cast<int>(bodies_.size());
    (void)before;
    return static_cast<int>(touched.size());
}

void World::unloadTile(int tileX, int tileY) {
    std::vector<std::uint8_t> buf(64 * 1024);
    int n = saveTileState(tileX, tileY, buf.data(), static_cast<int>(buf.size()));
    if (n < 0) {
        buf.resize(static_cast<std::size_t>(-n));
        n = saveTileState(tileX, tileY, buf.data(), static_cast<int>(buf.size()));
    }
    if (n > 16) {
        buf.resize(static_cast<std::size_t>(n));
        tileStore_[tileKey(tileX, tileY)] = std::move(buf);
    }
    for (Object& o : objects_) {
        if (tileOf(o.x) != tileX || tileOf(o.y) != tileY) continue;
        o.active = false;
        o.awake = false;
    }
    awake_.erase(std::remove_if(awake_.begin(), awake_.end(), [&](int i) { return !objects_[static_cast<std::size_t>(i)].active; }),
                 awake_.end());
}

void World::emit(EventType t, const Object& o, int part, const Vec3& at, Real value, std::int64_t ids) {
    Event e;
    e.type = static_cast<std::int32_t>(t);
    e.object = o.id;
    e.part = part;
    e.x = at.x;
    e.y = at.y;
    e.z = at.z;
    e.value = value;
    e.ids = ids;
    const int idx = (eventHead_ + eventCount_) % kMaxEvents;
    events_[static_cast<std::size_t>(idx)] = e;
    if (eventCount_ < kMaxEvents) ++eventCount_;
    else eventHead_ = (eventHead_ + 1) % kMaxEvents;
}

int World::pollEvents(Event* out, int capacity) {
    int n = 0;
    while (eventCount_ > 0 && n < capacity) {
        out[n++] = events_[static_cast<std::size_t>(eventHead_)];
        eventHead_ = (eventHead_ + 1) % kMaxEvents;
        --eventCount_;
    }
    return n;
}

Real World::objectTilt(int i) const {
    // The lean of the post as a whole: from its base to the top of its top segment.
    const Object& o = objects_[static_cast<std::size_t>(i)];
    const Body& top = bodies_[static_cast<std::size_t>(o.firstBody + o.segmentsPerLeg - 1)];
    const Vec3 tip = top.x + rotate(top.q, {0, 0, top.half.z});
    const Joint& base = joints_[static_cast<std::size_t>(o.firstJoint)];
    const Vec3 d = tip - base.rb;
    const Real l = length(d);
    return l > 1e-9 ? dm::acos(clamp(d.z / l, -1.0, 1.0)) : 0.0;
}

Real World::objectTopHeight(int i) const {
    const Object& o = objects_[static_cast<std::size_t>(i)];
    Real top = -1e9;
    for (int k = 0; k < o.bodyCount; ++k) {
        const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
        if (b.kind == 1) continue;
        const Vec3 ax = rotate(b.q, {0, 0, 1}) * b.half.z;
        top = std::max(top, std::max(b.x.z + ax.z, b.x.z - ax.z) + b.half.x);
    }
    return top - o.groundZ;
}

Real World::objectMaxPlastic(int i) const {
    const Object& o = objects_[static_cast<std::size_t>(i)];
    Real m = 0;
    for (int k = 0; k < o.jointCount; ++k) {
        const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
        if (j.kind != 2) m = std::max(m, j.plastic + j.soilPlastic);
    }
    return m;
}

Real World::objectStubHeight(int i) const { return objects_[static_cast<std::size_t>(i)].stub; }

Vec3 World::objectPanelCentre(int i, int panel) const {
    const Object& o = objects_[static_cast<std::size_t>(i)];
    for (int k = 0; k < o.bodyCount; ++k) {
        const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
        if (b.kind == 1 && b.panel == panel) return b.x;
    }
    return {};
}

bool World::objectPanelAttached(int i, int panel) const {
    const Object& o = objects_[static_cast<std::size_t>(i)];
    for (int k = 0; k < o.jointCount; ++k) {
        const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
        if (j.kind == 2 && j.panel == panel) return !j.broken;
    }
    return false;
}

int World::parts(int i, float* out, int capacity) const {
    const Object& o = objects_[static_cast<std::size_t>(i)];
    int k = 0;
    auto put = [&](Real v) {
        if (k < capacity) out[k] = static_cast<float>(v);
        ++k;
    };
    for (int n = 0; n < o.bodyCount; ++n) {
        const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + n)];
        put(b.kind);
        put(b.kind == 1 ? b.panel : n);
        put(b.x.x), put(b.x.y), put(b.x.z);
        put(b.q.w), put(b.q.x), put(b.q.y), put(b.q.z);
        put(b.half.x), put(b.half.y), put(b.half.z);
        put(b.loose ? 1 : 0);
    }
    return k <= capacity ? k : -k;
}

std::uint64_t World::stateHash() const {
    Hash64 h;
    h.add(static_cast<std::uint64_t>(substeps_));
    for (const Object& o : objects_) {
        h.add(static_cast<std::uint64_t>(o.state));
        h.add(o.stub);
    }
    for (const Body& b : bodies_) {
        h.add(b.x);
        h.add(b.q);
        h.add(b.v);
        h.add(b.w);
    }
    for (const Joint& j : joints_) {
        h.add(j.plastic);
        h.add(j.soilPlastic);
        h.add(static_cast<std::uint64_t>(j.broken));
    }
    return h.h;
}

Vec3 World::bodyImpulse(std::int32_t id) const {
    for (std::size_t i = 0; i < hosts_.size(); ++i)
        if (hosts_[i].id == id) return hostImpulse_[i];
    return {};
}

void World::setBodies(const HostBody* bodies, int count) {
    hosts_.assign(bodies, bodies + std::max(count, 0));
    if (hostImpulse_.size() < hosts_.size()) hostImpulse_.resize(hosts_.size());
}

// ---------------------------------------------------------------- persistence

namespace {
struct Writer {
    std::uint8_t* out;
    int cap, n = 0;
    void raw(const void* p, int len) {
        if (n + len <= cap) std::memcpy(out + n, p, static_cast<std::size_t>(len));
        n += len;
    }
    void i32(std::int32_t v) { raw(&v, 4); }
    void f64(double v) { raw(&v, 8); }
    void vec(const Vec3& v) { f64(v.x), f64(v.y), f64(v.z); }
    void quat(const Quat& q) { f64(q.w), f64(q.x), f64(q.y), f64(q.z); }
};
struct Reader {
    const std::uint8_t* in;
    int size, n = 0;
    bool ok = true;
    void raw(void* p, int len) {
        if (n + len > size) {
            ok = false;
            std::memset(p, 0, static_cast<std::size_t>(len));
            return;
        }
        std::memcpy(p, in + n, static_cast<std::size_t>(len));
        n += len;
    }
    std::int32_t i32() { std::int32_t v; raw(&v, 4); return v; }
    double f64() { double v; raw(&v, 8); return v; }
    Vec3 vec() { Vec3 v; v.x = f64(); v.y = f64(); v.z = f64(); return v; }
    Quat quat() { Quat q; q.w = f64(); q.x = f64(); q.y = f64(); q.z = f64(); return q; }
};
}  // namespace

int World::saveTileState(int tileX, int tileY, std::uint8_t* out, int capacity) const {
    Writer w{out, capacity};
    w.i32(0x53443031);  // "SD01"
    w.i32(tileX);
    w.i32(tileY);
    for (const Object& o : objects_) {
        if (tileOf(o.x) != tileX || tileOf(o.y) != tileY) continue;
        bool damaged = o.state != ObjectState::Intact;
        for (int k = 0; k < o.jointCount && !damaged; ++k) {
            const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
            damaged = j.broken || j.plastic > 1e-4 || j.soilPlastic > 1e-4;
        }
        if (!damaged) continue;
        w.i32(1);
        w.i32(o.id);
        w.i32(static_cast<std::int32_t>(o.state));
        w.f64(o.stub);
        w.i32(o.bodyCount);
        w.i32(o.jointCount);
        for (int k = 0; k < o.bodyCount; ++k) {
            const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
            w.vec(b.x);
            w.quat(b.q);
            w.i32(b.loose ? 1 : 0);
        }
        for (int k = 0; k < o.jointCount; ++k) {
            const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
            w.quat(j.rest);
            w.f64(j.plastic);
            w.f64(j.soilPlastic);
            w.i32(j.broken ? 1 : 0);
        }
    }
    w.i32(0);
    return w.n <= capacity ? w.n : -w.n;
}

int World::loadTileState(int tileX, int tileY, const std::uint8_t* data, int size) {
    Reader r{data, size};
    if (size <= 0 || r.i32() != 0x53443031 || r.i32() != tileX || r.i32() != tileY) return -1;
    int restored = 0;
    for (;;) {
        const int tag = r.i32();
        if (!r.ok || tag != 1) break;
        const int oi = objectIndex(r.i32());
        const auto state = static_cast<ObjectState>(r.i32());
        const Real stub = r.f64();
        const int nb = r.i32(), nj = r.i32();
        Object scratch;
        Object& o = oi >= 0 ? objects_[static_cast<std::size_t>(oi)] : scratch;
        const bool fits = oi >= 0 && nb == o.bodyCount && nj == o.jointCount;
        for (int k = 0; k < nb; ++k) {
            const Vec3 x = r.vec();
            const Quat q = r.quat();
            const bool loose = r.i32() != 0;
            if (!fits) continue;
            Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
            b.x = b.xPrev = x;
            b.q = b.qPrev = q;
            b.v = b.w = {};
            b.loose = loose;
        }
        for (int k = 0; k < nj; ++k) {
            const Quat rest = r.quat();
            const Real plastic = r.f64(), soil = r.f64();
            const bool broken = r.i32() != 0;
            if (!fits) continue;
            Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
            j.rest = rest;
            j.plastic = plastic;
            j.soilPlastic = soil;
            j.broken = broken;
        }
        if (!fits) continue;
        o.state = state;
        o.stub = stub;
        o.awake = false;
        // what it was already said to have done
        o.hitReported = true;
        o.bentReported = state != ObjectState::Intact;
        o.downReported = state == ObjectState::Down || state == ObjectState::Broken;
        o.brokenReported = state == ObjectState::Broken;
        o.panelLanded = {true, true, true, true};
        ++restored;
    }
    return restored;
}

bool World::repair(std::int32_t id) {
    const int oi = objectIndex(id);
    if (oi < 0) return false;
    Object& o = objects_[static_cast<std::size_t>(oi)];
    build(o, oi);
    o.awake = false;
    awake_.erase(std::remove(awake_.begin(), awake_.end(), oi), awake_.end());
    emit(EventType::Repaired, o, -1, {o.x, o.y, o.groundZ}, 0);
    return true;
}

}  // namespace sd
