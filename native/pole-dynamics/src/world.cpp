// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Tiles, spans, attachments, catenaries, power sections, lanes, events,
// queries and persistence. Bodies (pole pieces, transformers, the impact
// pulse) are in bodies.cpp; ropes in wires.cpp.
#include "pd/world.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include "pd/dmath.hpp"
#include "pd/json.hpp"

namespace pd {

namespace {

constexpr int kMaxEvents = 2048;
constexpr int kMaxCaps = 64;
constexpr int kMaxJoins = 512;
constexpr Real kTile = 500.0;

std::int64_t cellKey(Real x, Real y) {
    return (pythonRound(x * 10.0) << 32) ^ (pythonRound(y * 10.0) & 0xffffffffll);
}
int tileOf(Real v) { return static_cast<int>(pythonRound(v / kTile)); }
std::int64_t tileKey(int tx, int ty) { return (static_cast<std::int64_t>(tx) << 32) ^ (ty & 0xffffffffll); }

// Rotation about z taking body x to `fwd`.
Quat yawQuat(const Vec3& fwd) {
    const Real a = dm::atan2(fwd.y, fwd.x);
    Real s, c;
    dm::sincos(0.5 * a, s, c);
    return {c, 0, 0, s};
}

}  // namespace

World::World(std::uint32_t seed, int ropeSlots, int bodySlots) : seed_(seed) {
    ropes_.resize(static_cast<std::size_t>(ropeSlots));
    const auto nodes = static_cast<std::size_t>(ropeSlots) * kNodesPerSlot;
    x_.resize(nodes);
    prev_.resize(nodes);
    invMass_.resize(nodes);
    rest_.resize(nodes);
    lambda_.resize(nodes);
    bodies_.resize(static_cast<std::size_t>(bodySlots));
    events_.resize(kMaxEvents);
    caps_.resize(kMaxCaps);
    joins_.resize(kMaxJoins);
    awake_.resize(static_cast<std::size_t>(ropeSlots));
    rng_ = 0x9e3779b97f4a7c15ull ^ seed;
}

void World::emit(EventType t, int pole, int span, int wire, const Vec3& at, Real value, std::int64_t ids) {
    Event e;
    e.type = static_cast<std::int32_t>(t);
    e.pole = pole >= 0 ? poles_[static_cast<std::size_t>(pole)].spec.id : -1;
    e.span = span;
    e.wire = wire;
    e.x = at.x;
    e.y = at.y;
    e.z = at.z;
    e.value = value;
    e.ids = ids;
    if (t == EventType::PoleCracked || t == EventType::PoleLeaning || t == EventType::PoleSnapped || t == EventType::PoleFell ||
        t == EventType::WireSnapped || t == EventType::ClampFailed || t == EventType::TransformerDetached)
        lastStructural_ = time();
    const int idx = (eventHead_ + eventCount_) % kMaxEvents;
    events_[static_cast<std::size_t>(idx)] = e;
    if (eventCount_ < kMaxEvents)
        ++eventCount_;
    else
        eventHead_ = (eventHead_ + 1) % kMaxEvents;  // oldest dropped
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

// ---------------------------------------------------------------- tiles

int World::loadTile(std::string_view text) {
    statsDirty_ = true;
    error_.clear();
    const json::ParseResult pr = json::parse(std::string(text));
    if (!pr.ok()) {
        error_ = pr.error;
        return -1;
    }
    const json::Value* poles = pr.value.find("poles");
    const json::Value* spans = pr.value.find("spans");
    if (!poles || !poles->isArray()) {
        error_ = "poles missing";
        return -1;
    }
    const int firstSpan = static_cast<int>(spans_.size());
    std::vector<int> added;
    for (const json::Value& row : poles->items()) {
        if (!row.isArray() || row.items().size() < 4) continue;
        const auto& r = row.items();
        const PoleSpec spec = makePole(r[0].asNumber(), r[1].asNumber(), r[2].asNumber(), static_cast<int>(r[3].asNumber()));
        const auto known = poleById_.find(spec.id);
        if (known != poleById_.end()) {
            poles_[static_cast<std::size_t>(known->second)].active = true;
            continue;
        }
        Pole p;
        p.spec = spec;
        p.base = {spec.x, spec.y, 0};
        p.fwd = headingVector(spec.headingDeg);
        p.street = streetVector(spec.headingDeg);
        p.rest = yawQuat(p.fwd);
        p.tilt = Quat{};
        p.stubTilt = Quat{};
        const Real u = static_cast<Real>((static_cast<std::uint32_t>(spec.id) >> 7) % 1000u) / 999.0;
        p.strength = 1.0 + params.strength.variability * (2.0 * u - 1.0);
        p.transformerAttached = spec.transformer;
        p.tile = static_cast<int>(tileKey(tileOf(spec.x), tileOf(spec.y)) & 0x7fffffff);
        const int index = static_cast<int>(poles_.size());
        poles_.push_back(p);
        poleById_[spec.id] = index;
        poleByCell_[cellKey(spec.x, spec.y)] = index;
        added.push_back(index);
    }
    auto findAt = [&](Real x, Real y) -> int {
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                const auto it = poleByCell_.find(cellKey(x + dx * 0.1, y + dy * 0.1));
                if (it != poleByCell_.end()) return it->second;
            }
        return -1;
    };
    if (spans && spans->isArray()) {
        for (const json::Value& row : spans->items()) {
            if (!row.isArray() || row.items().size() < 4) continue;
            const auto& r = row.items();
            const Vec3 pa{r[0].asNumber(), r[1].asNumber(), 0}, pb{r[2].asNumber(), r[3].asNumber(), 0};
            std::uint64_t ka = static_cast<std::uint64_t>(cellKey(pa.x, pa.y)), kb = static_cast<std::uint64_t>(cellKey(pb.x, pb.y));
            if (ka > kb) std::swap(ka, kb);
            const std::uint64_t key = (ka * 1099511628211ull) ^ (kb + 0x9e3779b97f4a7c15ull);
            const int ia = findAt(pa.x, pa.y), ib = findAt(pb.x, pb.y);
            const auto known = spanByKey_.find(key);
            if (known != spanByKey_.end()) {
                // Seen from the other tile: link the pole that was missing then.
                Span& s = spans_[static_cast<std::size_t>(known->second)];
                s.active = true;
                if (s.mode == SpanMode::Analytic) {
                    if (s.a < 0 && ia >= 0 && ia != s.b) s.a = ia;
                    if (s.b < 0 && ib >= 0 && ib != s.a) s.b = ib;
                }
                continue;
            }
            Span s;
            s.key = key;
            s.a = ia;
            s.b = ib;
            if (s.a < 0 && s.b < 0) continue;
            if (s.a < 0) {  // keep a loaded pole at end A
                std::swap(s.a, s.b);
                s.anchorB = pa;
                s.anchorA = pb;
            } else {
                s.anchorA = pa;
                s.anchorB = pb;
            }
            s.wires = 8;
            for (int k = 0; k < 8; ++k) {
                s.kind[static_cast<std::size_t>(k)] = k < 3 ? 0 : (k < 7 ? 1 : 2);
                s.slotA[static_cast<std::size_t>(k)] = k;
                s.slotB[static_cast<std::size_t>(k)] = k;
            }
            const int ca = poles_[static_cast<std::size_t>(s.a)].spec.telecomCables;
            const int cb = s.b >= 0 ? poles_[static_cast<std::size_t>(s.b)].spec.telecomCables : ca;
            s.telecomMass = params.wires.type[2].massPerMetre + params.wires.telecomPerCable * 0.5 * (ca + cb);
            spanByKey_[key] = static_cast<int>(spans_.size());
            spans_.push_back(s);
        }
    }
    // Service drops: from the LV rack to the houses behind the pole.
    for (int pi : added) {
        const Pole& p = poles_[static_cast<std::size_t>(pi)];
        const int n = p.spec.serviceDrops;
        for (int d = 0; d < n && d < 4; ++d) {
            Span s;
            s.drop = true;
            s.a = pi;
            s.b = -1;
            s.dropSlot = 8 + d;
            const Real along = (d - 0.5 * (n - 1)) * 4.0;
            s.anchorB = p.base - p.street * params.wires.dropLength + p.fwd * along;
            s.anchorB.z = params.wires.dropHeight;
            s.wires = 1;
            s.kind[0] = 3;
            s.slotA[0] = s.dropSlot;
            s.slotB[0] = -1;
            s.key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.spec.id)) << 8) | static_cast<std::uint64_t>(d) | (1ull << 62);
            spanByKey_[s.key] = static_cast<int>(spans_.size());
            spans_.push_back(s);
        }
    }
    linkSpans(firstSpan);
    clampForce_.assign(poles_.size() * 12, Vec3{});
    touched_.assign(poles_.size(), 0);
    touchedFlag_.assign(poles_.size(), 0);
    touchedCount_ = 0;
    reachedScratch_.assign(poles_.size(), 0);
    queueScratch_.clear();
    queueScratch_.reserve(poles_.size() + 1);
    sectionScratch_.clear();
    sectionScratch_.reserve(poles_.size() + 1);
    footprintScratch_.clear();
    footprintScratch_.reserve(static_cast<std::size_t>(kWiresPerSlot * kNodesPerWire + 16));
    for (int pi : added) computeRestForces(pi);
    // A neighbour linked from the other tile gets its rest loads again.
    for (std::size_t si = static_cast<std::size_t>(firstSpan); si < spans_.size(); ++si) {
        const Span& s = spans_[si];
        if (s.a >= 0) computeRestForces(s.a);
        if (s.b >= 0) computeRestForces(s.b);
    }
    recomputeSections();
    recomputePower(false);
    const auto stored = tileStore_.find(tileKey(tileOf(added.empty() ? 0 : poles_[static_cast<std::size_t>(added[0])].spec.x),
                                                tileOf(added.empty() ? 0 : poles_[static_cast<std::size_t>(added[0])].spec.y)));
    if (!added.empty() && stored != tileStore_.end()) {
        const auto& d = stored->second;
        const Pole& p0 = poles_[static_cast<std::size_t>(added[0])];
        loadTileState(tileOf(p0.spec.x), tileOf(p0.spec.y), d.data(), static_cast<int>(d.size()));
    }
    stats_.poles = static_cast<int>(poles_.size());
    stats_.spans = static_cast<int>(spans_.size());
    return static_cast<int>(added.size());
}

void World::linkSpans(int firstNewSpan) {
    for (std::size_t si = static_cast<std::size_t>(firstNewSpan); si < spans_.size(); ++si) {
        const Span& s = spans_[si];
        for (int end = 0; end < 2; ++end) {
            const int pi = end == 0 ? s.a : s.b;
            if (pi < 0) continue;
            Pole& p = poles_[static_cast<std::size_t>(pi)];
            bool have = false;
            for (int i = 0; i < p.spanCount; ++i) have = have || p.span[i] == static_cast<int>(si);
            if (!have && p.spanCount < 12) p.span[p.spanCount++] = static_cast<int>(si);
        }
    }
    // Spans linked later from the other tile.
    for (std::size_t si = 0; si < static_cast<std::size_t>(firstNewSpan); ++si) {
        const Span& s = spans_[si];
        for (int pi : {s.a, s.b}) {
            if (pi < 0) continue;
            Pole& p = poles_[static_cast<std::size_t>(pi)];
            bool have = false;
            for (int i = 0; i < p.spanCount; ++i) have = have || p.span[i] == static_cast<int>(si);
            if (!have && p.spanCount < 12) p.span[p.spanCount++] = static_cast<int>(si);
        }
    }
}

void World::unloadTile(int tileX, int tileY) {
    statsDirty_ = true;
    // Keep the session's damage: store the tile's diff, then deactivate it.
    std::vector<std::uint8_t> buf(1 << 20);
    int n = saveTileState(tileX, tileY, buf.data(), static_cast<int>(buf.size()));
    if (n < 0) {
        buf.resize(static_cast<std::size_t>(-n));
        n = saveTileState(tileX, tileY, buf.data(), static_cast<int>(buf.size()));
    }
    if (n > 8) {
        buf.resize(static_cast<std::size_t>(n));
        tileStore_[tileKey(tileX, tileY)] = std::move(buf);
    }
    for (std::size_t i = 0; i < poles_.size(); ++i) {
        Pole& p = poles_[i];
        if (tileOf(p.spec.x) != tileX || tileOf(p.spec.y) != tileY) continue;
        p.active = false;
        if (p.body >= 0) bodies_[static_cast<std::size_t>(p.body)].active = false, p.body = -1;
        if (p.transformerBody >= 0) bodies_[static_cast<std::size_t>(p.transformerBody)].active = false, p.transformerBody = -1;
        for (int k = 0; k < p.spanCount; ++k) {
            Span& s = spans_[static_cast<std::size_t>(p.span[k])];
            if (s.rope >= 0) sleepSpan(p.span[k], true);
            const int other = s.a == static_cast<int>(i) ? s.b : s.a;
            if (other < 0 || !poles_[static_cast<std::size_t>(other)].active) s.active = false;
        }
    }
}

// ---------------------------------------------------------------- geometry

Vec3 World::attachmentLocal(int pole, int slot) const {
    const PoleGeometry& g = params.geometry;
    (void)pole;
    Real off = 0, h = 0;
    if (slot < 3) {
        off = g.mvOffset[static_cast<std::size_t>(slot)];
        h = g.mvHeight;
    } else if (slot < 7) {
        off = g.lvOffset;
        h = g.lvHeight[static_cast<std::size_t>(slot - 3)];
    } else if (slot == 7) {
        off = g.telecomOffset;
        h = g.telecomHeight;
    } else {
        off = -0.15;
        h = g.lvHeight[3] - 0.15 * (slot - 8);
    }
    return {0, -off, h};  // body frame: x along the line, y left (street is -y), z up
}

Vec3 World::attachmentPoint(int pole, int slot, Real* heightOut) const {
    const Pole& p = poles_[static_cast<std::size_t>(pole)];
    const Vec3 l = attachmentLocal(pole, slot);
    if (heightOut) *heightOut = l.z;
    if (p.body >= 0) {
        const Body& b = bodies_[static_cast<std::size_t>(p.body)];
        if (l.z >= b.z0 - 1e-9) return bodyPoint(b, pieceLocal(b, pole, slot));
    }
    const Quat q = (p.breakHeight > 0 ? p.stubTilt : p.tilt) * p.rest;
    return p.base + rotate(q, l);
}

Vec3 World::spanEnd(const Span& s, int end, int wire) const {
    const int pi = end == 0 ? s.a : s.b;
    const int slot = end == 0 ? s.slotA[static_cast<std::size_t>(wire)] : s.slotB[static_cast<std::size_t>(wire)];
    if (pi >= 0 && slot >= 0) return attachmentPoint(pi, slot);
    // A fixed end: a house, or a pole in a tile that is not loaded (same
    // attachment heights, the span direction as its heading).
    const Vec3 anchor = end == 0 ? s.anchorA : s.anchorB;
    if (s.drop) return anchor;
    const Vec3 other = end == 0 ? s.anchorB : s.anchorA;
    const Vec3 dir = normalized(Vec3{other.x - anchor.x, other.y - anchor.y, 0});
    const Vec3 street{dir.y, -dir.x, 0};
    const Vec3 l = attachmentLocal(-1, s.slotA[static_cast<std::size_t>(wire)]);
    return Vec3{anchor.x, anchor.y, 0} + street * (-l.y) + Vec3{0, 0, l.z};
}

const WireType& World::wireType(const Span& s, int wire) const {
    return params.wires.type[static_cast<std::size_t>(s.kind[static_cast<std::size_t>(wire)])];
}

Real World::wireMassPerMetre(const Span& s, int wire) const {
    return s.kind[static_cast<std::size_t>(wire)] == 2 ? s.telecomMass : wireType(s, wire).massPerMetre;
}

void World::restCatenary(const Span& s, int wire, const Vec3& pa, const Vec3& pb, int n, Vec3* out) const {
    // Parabola through the ends with the sag of the rest tension: s = w L^2 / (8 H).
    const Real lh = std::sqrt(sq(pb.x - pa.x) + sq(pb.y - pa.y));
    const Real w = wireMassPerMetre(s, wire) * kGravity;
    const Real sag = w * lh * lh / (8.0 * wireType(s, wire).tension);
    for (int i = 0; i < n; ++i) {
        const Real t = static_cast<Real>(i) / (n - 1);
        out[i] = pa + (pb - pa) * t;
        out[i].z -= 4.0 * sag * t * (1.0 - t);
    }
}

void World::computeRestForces(int pole) {
    Pole& p = poles_[static_cast<std::size_t>(pole)];
    for (auto& f : p.restForce) f = {};
    for (int k = 0; k < p.spanCount; ++k) {
        Span& s = spans_[static_cast<std::size_t>(p.span[k])];
        const int end = s.a == pole ? 0 : 1;
        for (int w = 0; w < s.wires; ++w) {
            const int slot = end == 0 ? s.slotA[static_cast<std::size_t>(w)] : s.slotB[static_cast<std::size_t>(w)];
            if (slot < 0) continue;
            const Vec3 pa = spanEnd(s, 0, w), pb = spanEnd(s, 1, w);
            // Tension at the end along the parabola's end tangent: horizontal H.
            const Vec3 near = end == 0 ? pa : pb, far = end == 0 ? pb : pa;
            const Real lh = std::sqrt(sq(far.x - near.x) + sq(far.y - near.y));
            if (lh < 1e-6) continue;
            const Real H = wireType(s, w).tension;
            const Real wl = wireMassPerMetre(s, w) * kGravity;
            const Real slope = (far.z - near.z) / lh + wl * lh / (2.0 * H);  // dz/dx at the near end, toward far
            const Vec3 hdir{(far.x - near.x) / lh, (far.y - near.y) / lh, 0};
            const Vec3 f{hdir.x * H, hdir.y * H, slope * H};
            s.restEnd[end][static_cast<std::size_t>(w)] = f;
            p.restForce[static_cast<std::size_t>(slot)] += f;
        }
    }
    // An unbalanced pole (a dead end or a corner) is guyed against its pull.
    Vec3 sum;
    Real mz = 0, f = 0;
    for (int slot = 0; slot < 12; ++slot) {
        const Vec3 r = p.restForce[slot];
        sum += Vec3{r.x, r.y, 0};
        const Real fh = std::sqrt(r.x * r.x + r.y * r.y);
        f += fh;
        mz += fh * attachmentLocal(pole, slot).z;
    }
    p.guyDir = length(sum) > 300.0 ? normalized(sum) : Vec3{};
    p.guyHeight = f > 0 ? mz / f : 0;
    p.guyCapacity = 2.0 * length(sum);
}

Vec3 World::guyReaction(const Pole& p, const Vec3& pull) const {
    // A guy only pulls: it takes the horizontal pull along its direction.
    if (dot(p.guyDir, p.guyDir) == 0) return {};
    const Real along = dot(pull, p.guyDir);
    return along > 0 ? p.guyDir * (-std::min(along, p.guyCapacity)) : Vec3{};
}

// ---------------------------------------------------------------- power

void World::recomputeSections() {
    // Each pole's section is its nearest transformer along the line (hops,
    // then the lower ID): the low-voltage network splits midway between
    // transformers.
    const std::size_t n = poles_.size();
    std::vector<int> dist(n, 1 << 30);
    std::vector<int> queue;
    queue.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        poles_[i].section = -1;
        if (poles_[i].spec.transformer && poles_[i].active) {
            dist[i] = 0;
            poles_[i].section = poles_[i].spec.id;
            queue.push_back(static_cast<int>(i));
        }
    }
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        const int u = queue[qi];
        const Pole& pu = poles_[static_cast<std::size_t>(u)];
        for (int k = 0; k < pu.spanCount; ++k) {
            const Span& s = spans_[static_cast<std::size_t>(pu.span[k])];
            if (s.drop) continue;
            const int v = s.a == u ? s.b : s.a;
            if (v < 0) continue;
            Pole& pv = poles_[static_cast<std::size_t>(v)];
            const int d = dist[static_cast<std::size_t>(u)] + 1;
            if (d < dist[static_cast<std::size_t>(v)] ||
                (d == dist[static_cast<std::size_t>(v)] && pu.section < pv.section)) {
                if (d < dist[static_cast<std::size_t>(v)]) queue.push_back(v);
                dist[static_cast<std::size_t>(v)] = d;
                pv.section = pu.section;
            }
        }
    }
}

void World::recomputePower(bool emitEvents) {
    // A pole is powered when its section's transformer is in place and fed,
    // and an intact low-voltage path (every LV conductor of each span whole
    // and on its insulators) joins them inside the section.
    const std::size_t n = poles_.size();
    std::vector<std::uint8_t>& reached = reachedScratch_;
    std::fill(reached.begin(), reached.end(), std::uint8_t{0});
    std::vector<int>& queue = queueScratch_;
    queue.clear();
    for (std::size_t i = 0; i < n; ++i) {
        const Pole& p = poles_[i];
        if (!p.spec.transformer || !p.transformerAttached || !p.active) continue;
        if (p.state == PoleState::Snapped || p.state == PoleState::Fallen) continue;
        bool mvOk = true;  // the transformer's own MV taps
        for (int k = 0; k < p.spanCount && mvOk; ++k) {
            const Span& s = spans_[static_cast<std::size_t>(p.span[k])];
            if (s.drop) continue;
            if (s.rope >= 0)
                for (int w = 0; w < 3; ++w) mvOk = mvOk && ropes_[static_cast<std::size_t>(s.rope)].wire[static_cast<std::size_t>(w)].broken == 0;
        }
        for (int c = 0; c < 3; ++c) mvOk = mvOk && !p.clampBroken[static_cast<std::size_t>(c)];
        if (!mvOk) continue;
        reached[i] = 1;
        queue.push_back(static_cast<int>(i));
    }
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        const int u = queue[qi];
        const Pole& pu = poles_[static_cast<std::size_t>(u)];
        for (int k = 0; k < pu.spanCount; ++k) {
            const Span& s = spans_[static_cast<std::size_t>(pu.span[k])];
            if (s.drop) continue;
            const int v = s.a == u ? s.b : s.a;
            if (v < 0 || reached[static_cast<std::size_t>(v)]) continue;
            const Pole& pv = poles_[static_cast<std::size_t>(v)];
            if (pv.section != pu.section) continue;
            bool lvOk = true;
            for (int c = 3; c < 7; ++c) lvOk = lvOk && !pu.clampBroken[static_cast<std::size_t>(c)] && !pv.clampBroken[static_cast<std::size_t>(c)];
            if (s.rope >= 0)
                for (int w = 3; w < 7; ++w) lvOk = lvOk && ropes_[static_cast<std::size_t>(s.rope)].wire[static_cast<std::size_t>(w)].broken == 0;
            if (!lvOk) continue;
            reached[static_cast<std::size_t>(v)] = 1;
            queue.push_back(v);
        }
    }
    // Sections that lost poles, for one POWER_OUT event each.
    std::vector<std::int32_t>& out = sectionScratch_;
    out.clear();
    for (std::size_t i = 0; i < n; ++i) {
        Pole& p = poles_[i];
        const bool powered = reached[i] != 0;
        const bool lit = powered && p.spec.light && p.state != PoleState::Snapped && p.state != PoleState::Fallen;
        if (emitEvents && p.powered && !powered) {
            if (std::find(out.begin(), out.end(), p.section) == out.end()) out.push_back(p.section);
        }
        if (emitEvents && p.powered != powered && powered)
            emit(EventType::PowerRestored, static_cast<int>(i), -1, -1, p.base, 1, p.section);
        if (emitEvents && p.lit != lit && p.spec.light)
            emit(EventType::LightChanged, static_cast<int>(i), -1, -1, p.base + Vec3{0, 0, params.geometry.lightLow}, lit ? 1 : 0, p.section);
        p.powered = powered;
        p.lit = lit;
    }
    for (std::int32_t sec : out) {
        int count = 0;
        Vec3 at;
        for (std::size_t i = 0; i < n; ++i)
            if (poles_[i].section == sec && !poles_[i].powered) {
                ++count;
                if (poles_[i].spec.id == sec) at = poles_[i].base;
            }
        emit(EventType::PowerOut, -1, -1, -1, at, count, sec);
    }
}

// ---------------------------------------------------------------- lanes

int World::loadLanes(std::string_view text) {
    error_.clear();
    const json::ParseResult pr = json::parse(std::string(text));
    if (!pr.ok()) {
        error_ = pr.error;
        return -1;
    }
    const json::Value* t = pr.value.find("t");
    const json::Value* e = pr.value.find("e");
    if (!t || !t->isString() || !e || !e->isArray()) {
        error_ = "lanes: t or e missing";
        return -1;
    }
    // "T_m10_p3": tile x = -10, y = 3. Tiles are centred on (500 i, 500 j);
    // lane points are 0.5 m pixels from the tile's north-west corner, y down.
    const std::string& name = t->asString();
    int tx = 0, ty = 0;
    {
        std::size_t p1 = name.find('_'), p2 = name.find('_', p1 + 1);
        if (p1 == std::string::npos || p2 == std::string::npos) {
            error_ = "lanes: bad tile name";
            return -1;
        }
        auto num = [](const std::string& s) { return (s[0] == 'm' ? -1 : 1) * std::atoi(s.c_str() + 1); };
        tx = num(name.substr(p1 + 1, p2 - p1 - 1));
        ty = num(name.substr(p2 + 1));
    }
    const Real ox = tx * kTile - 0.5 * kTile, oy = ty * kTile + 0.5 * kTile;
    int edgeIndex = 0, added = 0;
    for (const json::Value& row : e->items()) {
        const auto& r = row.items();
        if (r.size() < 9) {
            ++edgeIndex;
            continue;
        }
        LaneEdge le;
        le.tileX = tx;
        le.tileY = ty;
        le.edge = edgeIndex++;
        le.lanes = std::max(1, static_cast<int>(r[1].asNumber() + r[2].asNumber()));
        le.first = static_cast<int>(lanePts_.size());
        le.minX = le.minY = 1e300;
        le.maxX = le.maxY = -1e300;
        for (std::size_t k = 5; k + 1 < r.size(); k += 2) {
            const Vec3 q{ox + 0.5 * r[k].asNumber(), oy - 0.5 * r[k + 1].asNumber(), 0};
            lanePts_.push_back(q);
            le.minX = std::min(le.minX, q.x);
            le.maxX = std::max(le.maxX, q.x);
            le.minY = std::min(le.minY, q.y);
            le.maxY = std::max(le.maxY, q.y);
        }
        le.count = static_cast<int>(lanePts_.size()) - le.first;
        lanes_.push_back(le);
        ++added;
    }
    return added;
}

void World::checkLanes(int pole, int span) {
    // Ground footprint of what is down: a fallen piece (its axis below the
    // clearance) or a wire hanging below the clearance.
    std::vector<Vec3>& pts = footprintScratch_;
    pts.clear();
    const Real clearance = params.sim.laneClearance;
    if (pole >= 0) {
        const Pole& p = poles_[static_cast<std::size_t>(pole)];
        if (p.body >= 0) {
            const Body& b = bodies_[static_cast<std::size_t>(p.body)];
            for (int i = 0; i <= 10; ++i) {
                const Real z = b.z0 + (b.z1 - b.z0) * i / 10.0;
                const Vec3 q = bodyPoint(b, Vec3{-b.comOffset.x, -b.comOffset.y, z - b.comHeight});
                if (q.z < clearance) pts.push_back(q);
            }
        }
    }
    if (span >= 0) {
        const Span& s = spans_[static_cast<std::size_t>(span)];
        if (s.rope >= 0) {
            const Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
            for (int w = 0; w < s.wires; ++w)
                for (int i = 0; i < r.wire[static_cast<std::size_t>(w)].n; ++i) {
                    const Vec3& q = x_[static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire + i)];
                    if (q.z < clearance) pts.push_back(q);
                }
        }
    }
    if (pts.empty()) return;
    const Real lw = params.sim.laneWidth;
    for (const LaneEdge& le : lanes_) {
        const Real half = 0.5 * lw * le.lanes + 1.0;
        bool near = false;
        for (const Vec3& q : pts)
            near = near || (q.x > le.minX - half && q.x < le.maxX + half && q.y > le.minY - half && q.y < le.maxY + half);
        if (!near) continue;
        std::uint32_t lanesHit = 0;
        for (int k = 0; k + 1 < le.count; ++k) {
            const Vec3 a = lanePts_[static_cast<std::size_t>(le.first + k)], b = lanePts_[static_cast<std::size_t>(le.first + k + 1)];
            const Vec3 ab = b - a;
            const Real l2 = dot(ab, ab);
            if (l2 < 1e-9) continue;
            const Vec3 right = normalized(Vec3{ab.y, -ab.x, 0});
            for (const Vec3& q : pts) {
                const Vec3 aq{q.x - a.x, q.y - a.y, 0};
                const Real t = dot(aq, ab) / l2;
                if (t < 0 || t > 1) continue;
                // Lanes from the left edge of the road (looking along the edge) to the right.
                const Real off = dot(aq, right) + 0.5 * lw * le.lanes;
                const int lane = static_cast<int>(std::floor(off / lw));
                if (lane >= 0 && lane < le.lanes && lane < 32) lanesHit |= 1u << lane;
            }
        }
        std::uint64_t& mask = span >= 0 ? spans_[static_cast<std::size_t>(span)].laneMask
                                         : poles_[static_cast<std::size_t>(pole)].laneMask;
        const int edgeSlot = static_cast<int>(&le - lanes_.data());
        for (int lane = 0; lane < le.lanes && lane < 32; ++lane)
            if (lanesHit & (1u << lane)) {
                const std::uint64_t bit = 1ull << ((edgeSlot * 8 + lane) & 63);
                if (mask & bit) continue;  // already reported
                mask |= bit;
                const int who = pole >= 0 ? pole : spans_[static_cast<std::size_t>(span)].a;
                emit(EventType::LaneBlocked, who, span, -1, pts[0], lane, packLane(le.tileX, le.tileY, le.edge, lane));
            }
    }
}

// ---------------------------------------------------------------- queries

int World::poleIndex(std::int32_t id) const {
    const auto it = poleById_.find(id);
    return it == poleById_.end() ? -1 : it->second;
}

int World::findPoleNear(const Vec3& p, Real radius) const {
    int best = -1;
    Real bd = radius * radius;
    for (std::size_t i = 0; i < poles_.size(); ++i) {
        const Pole& q = poles_[i];
        if (!q.active) continue;
        const Real d = sq(q.base.x - p.x) + sq(q.base.y - p.y);
        if (d < bd) {
            bd = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

Real World::poleTilt(int index) const {
    const Pole& p = poles_[static_cast<std::size_t>(index)];
    Vec3 up;
    if (p.body >= 0)
        up = rotate(bodies_[static_cast<std::size_t>(p.body)].q, Vec3{0, 0, 1});
    else
        up = rotate(p.tilt, Vec3{0, 0, 1});
    return dm::acos(clamp(up.z, -1.0, 1.0));
}

bool World::queryPole(std::int32_t id, PoleQuery& out) const {
    const int i = poleIndex(id);
    if (i < 0) return false;
    const Pole& p = poles_[static_cast<std::size_t>(i)];
    if (!p.active) return false;  // its tile is unloaded
    out = PoleQuery{};
    out.id = id;
    out.state = static_cast<int>(p.state);
    out.lit = p.lit ? 1 : 0;
    out.powered = p.powered ? 1 : 0;
    out.section = p.section;
    out.tiltDeg = poleTilt(i) / kDeg;
    const Quat standing = (p.breakHeight > 0 ? p.stubTilt : p.tilt) * p.rest;
    if (p.body < 0) {
        out.pieces = 1;
        out.piece[0] = {p.base, standing, p.breakHeight > 0 ? 0.0 : 0.0, p.breakHeight > 0 ? p.breakHeight : params.geometry.lengthAbove};
    } else {
        const Body& b = bodies_[static_cast<std::size_t>(p.body)];
        const Vec3 bottom = bodyPoint(b, Vec3{-b.comOffset.x, -b.comOffset.y, b.z0 - b.comHeight});
        if (p.breakHeight > 0) {
            out.pieces = 2;
            out.piece[0] = {p.base, standing, 0.0, p.breakHeight};
            out.piece[1] = {bottom, b.q, b.z0, b.z1};
        } else {
            out.pieces = 1;
            out.piece[0] = {bottom, b.q, b.z0, b.z1};
        }
    }
    if (p.spec.transformer) {
        if (p.transformerAttached) {
            out.transformer = 1;
            out.transformerPosition = transformerPoint(i);
            out.transformerRotation = p.body >= 0 ? bodies_[static_cast<std::size_t>(p.body)].q : standing;
        } else if (p.transformerBody >= 0) {
            const Body& t = bodies_[static_cast<std::size_t>(p.transformerBody)];
            out.transformer = 2;
            out.transformerPosition = t.x;
            out.transformerRotation = t.q;
        }
    }
    return true;
}

SpanMode World::spanMode(int span) const { return spans_[static_cast<std::size_t>(span)].mode; }
int World::spanWireCount(int span) const { return spans_[static_cast<std::size_t>(span)].wires; }

void World::spanInfo(int span, std::int32_t& poleA, std::int32_t& poleB, Real& maxTension) const {
    const Span& s = spans_[static_cast<std::size_t>(span)];
    poleA = s.a >= 0 ? poles_[static_cast<std::size_t>(s.a)].spec.id : -1;
    poleB = s.b >= 0 ? poles_[static_cast<std::size_t>(s.b)].spec.id : -1;
    maxTension = 0;
    for (int w = 0; w < s.wires; ++w) maxTension = std::max(maxTension, spanTension(span, w));
}

Real World::spanTension(int span, int wire) const {
    const Span& s = spans_[static_cast<std::size_t>(span)];
    if (s.rope < 0) return wireType(s, wire).tension;
    const Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
    const Wire& w = r.wire[static_cast<std::size_t>(wire)];
    Real t = 0;
    for (int i = 0; i + 1 < w.n; ++i)
        if (!(w.broken & (1u << i))) t = std::max(t, segTension(s.rope, wire, i));
    return t;
}

int World::wireNodes(int span, float* out, int capacity) const {
    const Span& s = spans_[static_cast<std::size_t>(span)];
    if (s.rope < 0) return 0;
    const Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
    int k = 0;
    auto put = [&](Real v) {
        if (k < capacity) out[k] = static_cast<float>(v);
        ++k;
    };
    put(s.wires);
    for (int w = 0; w < s.wires; ++w) {
        const Wire& wi = r.wire[static_cast<std::size_t>(w)];
        int brokenCount = 0;
        for (int i = 0; i + 1 < wi.n; ++i) brokenCount += (wi.broken >> i) & 1u;
        put(wi.kind);
        put(brokenCount);
        put(wi.n);
        for (int i = 0; i < wi.n; ++i) {
            const Vec3& q = x_[static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire + i)];
            put(q.x);
            put(q.y);
            put(q.z);
        }
        // Broken segments as a bit mask in the next value (renderers skip them).
        put(static_cast<Real>(wi.broken));
    }
    return k <= capacity ? k : -k;
}

int World::wireCurve(int span, int samples, float* out, int capacity) const {
    const Span& s = spans_[static_cast<std::size_t>(span)];
    if (s.rope >= 0) return wireNodes(span, out, capacity);
    int k = 0;
    auto put = [&](Real v) {
        if (k < capacity) out[k] = static_cast<float>(v);
        ++k;
    };
    put(s.wires);
    Vec3 pts[64];
    samples = std::clamp(samples, 2, 64);
    for (int w = 0; w < s.wires; ++w) {
        restCatenary(s, w, spanEnd(s, 0, w), spanEnd(s, 1, w), samples, pts);
        put(s.kind[static_cast<std::size_t>(w)]);
        put(0);
        put(samples);
        for (int i = 0; i < samples; ++i) {
            put(pts[i].x);
            put(pts[i].y);
            put(pts[i].z);
        }
        put(0);
    }
    return k <= capacity ? k : -k;
}

std::uint64_t World::stateHash() const {
    Hash64 h;
    h.add(static_cast<std::uint64_t>(steps_));
    for (const Pole& p : poles_) {
        h.add(static_cast<std::uint64_t>(p.state));
        h.add(p.tilt);
        h.add(p.stubTilt);
        for (auto c : p.clampBroken) h.add(static_cast<std::uint64_t>(c));
    }
    for (const Body& b : bodies_) {
        if (!b.active) continue;
        h.add(b.x);
        h.add(b.q);
        h.add(b.v);
        h.add(b.w);
        h.add(b.plastic);
    }
    for (std::size_t r = 0; r < ropes_.size(); ++r) {
        if (!ropes_[r].used) continue;
        for (int w = 0; w < kWiresPerSlot; ++w) {
            const Wire& wi = ropes_[r].wire[static_cast<std::size_t>(w)];
            h.add(static_cast<std::uint64_t>(wi.broken));
            for (int i = 0; i < wi.n; ++i) h.add(x_[r * kNodesPerSlot + static_cast<std::size_t>(w * kNodesPerWire + i)]);
        }
    }
    return h.h;
}

// ---------------------------------------------------------------- step

void World::setBodies(const Capsule* caps, int n) {
    capCount_ = std::clamp(n, 0, kMaxCaps);
    for (int i = 0; i < capCount_; ++i) caps_[static_cast<std::size_t>(i)] = caps[i];
}

int World::step(Real dt) {
    const auto t0 = std::chrono::steady_clock::now();
    const Real h = subDt();
    acc_ += dt;
    int n = static_cast<int>(std::floor(acc_ / h + 1e-9));
    const int cap = params.sim.substeps * 8;
    if (n > cap) n = cap;
    acc_ -= n * h;
    if (acc_ < 0) acc_ = 0;
    int worked = 0;
    for (int i = 0; i < n; ++i) {
        ++steps_;
        // Anything to do? (Cheap when nothing happens.)
        bool anyBody = false;
        for (const Body& b : bodies_) anyBody = anyBody || (b.active && !b.asleep);
        if (!anyBody && awakeCount_ == 0) continue;
        ++worked;
        ropeForces();
        stepBodies(h);
        stepRopes(h);
        checkClamps();
        updatePoleStates();
        checkRopeSleep(h);
        if (i == n - 1) hazards(h * n);  // once per host step
        if (powerDirty_) {
            powerDirty_ = false;
            recomputePower(true);
        }
    }
    const auto t1 = std::chrono::steady_clock::now();
    stats_.lastStepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.maxStepMs = std::max(stats_.maxStepMs, stats_.lastStepMs);
    stats_.steps = steps_;
    if (worked == 0 && !statsDirty_) return 0;  // idle: nothing to count either
    statsDirty_ = false;
    int awakeN = 0, frozenN = 0, nodes = 0, bodies = 0;
    for (const Span& s : spans_) {
        if (s.mode == SpanMode::Awake) ++awakeN;
        if (s.mode == SpanMode::Frozen) ++frozenN;
        if (s.rope >= 0)
            for (int w = 0; w < s.wires; ++w) nodes += ropes_[static_cast<std::size_t>(s.rope)].wire[static_cast<std::size_t>(w)].n;
    }
    for (const Body& b : bodies_) bodies += b.active ? 1 : 0;
    stats_.awakeSpans = awakeN;
    stats_.frozenSpans = frozenN;
    stats_.nodes = nodes;
    stats_.bodies = bodies;
    return worked;
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
    void vec(const Vec3& v) { f64(v.x); f64(v.y); f64(v.z); }
    void quat(const Quat& q) { f64(q.w); f64(q.x); f64(q.y); f64(q.z); }
};
struct Reader {
    const std::uint8_t* in;
    int size, n = 0;
    bool ok = true;
    void raw(void* p, int len) {
        if (n + len > size) {
            ok = false;
            return;
        }
        std::memcpy(p, in + n, static_cast<std::size_t>(len));
        n += len;
    }
    std::int32_t i32() { std::int32_t v = 0; raw(&v, 4); return v; }
    double f64() { double v = 0; raw(&v, 8); return v; }
    Vec3 vec() { Vec3 v; v.x = f64(); v.y = f64(); v.z = f64(); return v; }
    Quat quat() { Quat q; q.w = f64(); q.x = f64(); q.y = f64(); q.z = f64(); return q; }
};
}  // namespace

int World::saveTileState(int tileX, int tileY, std::uint8_t* out, int capacity) const {
    // The diff: damaged poles (state, stub and piece transforms) and every
    // span that is not on its catenary (node positions, broken segments).
    Writer w{out, capacity};
    w.i32(0x50443031);  // "PD01"
    w.i32(tileX);
    w.i32(tileY);
    for (std::size_t i = 0; i < poles_.size(); ++i) {
        const Pole& p = poles_[i];
        if (tileOf(p.spec.x) != tileX || tileOf(p.spec.y) != tileY) continue;
        bool damaged = p.state != PoleState::Intact || !p.transformerAttached;
        for (auto c : p.clampBroken) damaged = damaged || c != 0;
        if (!damaged) continue;
        w.i32(1);
        w.i32(p.spec.id);
        w.i32(static_cast<int>(p.state));
        w.f64(p.breakHeight);
        w.quat(p.tilt);
        w.quat(p.stubTilt);
        w.i32(p.transformerAttached ? 1 : 0);
        for (auto c : p.clampBroken) w.i32(c);
        const bool body = p.body >= 0;
        w.i32(body ? 1 : 0);
        if (body) {
            const Body& b = bodies_[static_cast<std::size_t>(p.body)];
            w.vec(b.x);
            w.quat(b.q);
            w.f64(b.z0);
            w.f64(b.z1);
            w.i32(b.joint);
            w.vec(b.anchor);
        }
        for (int k = 0; k < p.spanCount; ++k) {
            const Span& s = spans_[static_cast<std::size_t>(p.span[k])];
            if (s.rope < 0 || s.a != static_cast<int>(i)) continue;
            const Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
            // Spans by their poles' IDs (stable across sessions), not by index.
            w.i32(2);
            w.i32(p.spec.id);
            w.i32(s.b >= 0 ? poles_[static_cast<std::size_t>(s.b)].spec.id : -1);
            w.i32(k);
            for (int wi = 0; wi < s.wires; ++wi) {
                const Wire& ww = r.wire[static_cast<std::size_t>(wi)];
                w.i32(ww.n);
                w.i32(static_cast<std::int32_t>(ww.broken));
                w.i32(ww.pinnedA);
                w.i32(ww.pinnedB);
                for (int ni = 0; ni < ww.n; ++ni) w.vec(x_[static_cast<std::size_t>(s.rope * kNodesPerSlot + wi * kNodesPerWire + ni)]);
            }
        }
    }
    w.i32(0);
    return w.n <= capacity ? w.n : -w.n;  // negative: the size it needs
}

int World::loadTileState(int tileX, int tileY, const std::uint8_t* data, int size) {
    statsDirty_ = true;
    Reader r{data, size};
    if (size <= 0 || r.i32() != 0x50443031 || r.i32() != tileX || r.i32() != tileY) return -1;
    int restored = 0;
    int lastPole = -1;
    for (;;) {
        const int tag = r.i32();
        if (!r.ok || tag == 0) break;
        if (tag == 1) {
            const int id = r.i32();
            const int pi = poleIndex(id);
            Pole tmp;
            Pole& p = pi >= 0 ? poles_[static_cast<std::size_t>(pi)] : tmp;
            p.state = static_cast<PoleState>(r.i32());
            p.breakHeight = r.f64();
            p.tilt = r.quat();
            p.stubTilt = r.quat();
            p.transformerAttached = r.i32() != 0;
            for (auto& c : p.clampBroken) c = static_cast<std::uint8_t>(r.i32());
            p.crackedReported = p.leanReported = p.snappedReported = p.fellReported = true;
            if (r.i32() == 1) {
                const Vec3 x = r.vec();
                const Quat q = r.quat();
                const Real z0 = r.f64(), z1 = r.f64();
                const int joint = r.i32();
                const Vec3 anchor = r.vec();
                if (pi >= 0) {
                    makeDynamic(pi);
                    Body& b = bodies_[static_cast<std::size_t>(p.body)];
                    const MassProps mp = massProperties(params.geometry, p.spec, z0, z1, p.transformerAttached);
                    b.z0 = z0;
                    b.z1 = z1;
                    b.mass = mp.mass;
                    b.invMass = 1.0 / mp.mass;
                    b.iPerp = mp.inertiaPerp;
                    b.iAxial = mp.inertiaAxial;
                    b.comHeight = mp.comHeight;
                    b.comOffset = mp.comOffset;
                    b.x = x;
                    b.q = q;
                    b.joint = joint;
                    b.anchor = anchor;
                    b.anchorLocal = Vec3{-mp.comOffset.x, -mp.comOffset.y, z0 - mp.comHeight};
                    b.yield = momentCapacity(p, Vec3{1, 0, 0}, params.strength.residualFactor);
                    b.asleep = true;
                }
            }
            lastPole = pi;
            ++restored;
        } else if (tag == 2) {
            const int pa = poleIndex(r.i32());
            const std::int32_t otherId = r.i32();
            const int slot = r.i32();
            if (pa < 0) break;
            const Pole& owner = poles_[static_cast<std::size_t>(pa)];
            int si = -1;
            for (int k = 0; k < owner.spanCount && si < 0; ++k) {
                const Span& c = spans_[static_cast<std::size_t>(owner.span[k])];
                if (c.a != pa) continue;
                const std::int32_t cb = c.b >= 0 ? poles_[static_cast<std::size_t>(c.b)].spec.id : -1;
                if (cb == otherId && (otherId >= 0 || k == slot)) si = owner.span[k];
            }
            if (si < 0) break;
            Span& s = spans_[static_cast<std::size_t>(si)];
            if (s.rope < 0) wakeSpan(si);
            if (s.rope < 0) break;
            Rope& rope = ropes_[static_cast<std::size_t>(s.rope)];
            for (int wi = 0; wi < s.wires; ++wi) {
                Wire& ww = rope.wire[static_cast<std::size_t>(wi)];
                ww.n = std::clamp(r.i32(), 0, kNodesPerWire);
                ww.broken = static_cast<std::uint32_t>(r.i32());
                ww.pinnedA = static_cast<std::uint8_t>(r.i32());
                ww.pinnedB = static_cast<std::uint8_t>(r.i32());
                for (int ni = 0; ni < ww.n; ++ni) {
                    const auto k = static_cast<std::size_t>(s.rope * kNodesPerSlot + wi * kNodesPerWire + ni);
                    x_[k] = prev_[k] = r.vec();
                }
            }
            s.mode = SpanMode::Frozen;
            for (int k = 0; k < awakeCount_; ++k)
                if (awake_[static_cast<std::size_t>(k)] == si) awake_[static_cast<std::size_t>(k)] = awake_[static_cast<std::size_t>(--awakeCount_)];
        } else {
            break;
        }
    }
    (void)lastPole;
    recomputePower(false);
    return restored;
}

bool World::repairPole(std::int32_t id) {
    statsDirty_ = true;
    // The crew hook: put the pole back up, re-string its spans.
    const int pi = poleIndex(id);
    if (pi < 0) return false;
    Pole& p = poles_[static_cast<std::size_t>(pi)];
    if (p.body >= 0) bodies_[static_cast<std::size_t>(p.body)].active = false;
    if (p.transformerBody >= 0) bodies_[static_cast<std::size_t>(p.transformerBody)].active = false;
    p.body = p.transformerBody = -1;
    p.state = PoleState::Intact;
    p.tilt = p.stubTilt = Quat{};
    p.breakHeight = 0;
    p.transformerAttached = p.spec.transformer;
    p.clampBroken.fill(0);
    p.crackedReported = p.leanReported = p.snappedReported = p.fellReported = false;
    p.laneMask = 0;
    for (int k = 0; k < p.spanCount; ++k) {
        const int si = p.span[k];
        Span& s = spans_[static_cast<std::size_t>(si)];
        const int other = s.a == pi ? s.b : s.a;
        const bool otherOk = other < 0 || (poles_[static_cast<std::size_t>(other)].state != PoleState::Snapped &&
                                           poles_[static_cast<std::size_t>(other)].state != PoleState::Fallen);
        if (s.rope >= 0 && otherOk) sleepSpan(si, true);
        s.laneMask = 0;
    }
    for (Join& j : joins_)
        if (j.active && j.pole == pi) j.active = false;
    computeRestForces(pi);
    emit(EventType::PoleRepaired, pi, -1, -1, p.base, 0, 0);
    emit(EventType::LaneCleared, pi, -1, -1, p.base, 0, 0);
    recomputePower(true);
    return true;
}

}  // namespace pd
