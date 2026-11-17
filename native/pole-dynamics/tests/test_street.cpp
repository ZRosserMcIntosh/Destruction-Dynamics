// Pole Dynamics — the street scenarios from the brief, end to end, with traces.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "pd/world.hpp"
#include "pd_test.hpp"
#include "scenarios.hpp"

using namespace pd;

namespace {

struct Outcome {
    std::vector<std::int32_t> ids;
    std::vector<int> state;        // final, per pole
    std::vector<Real> tilt;        // final, degrees
    std::vector<Real> maxTilt;
    std::vector<int> events;       // per EventType count
    std::vector<Event> all;
    std::vector<int> powered, section;
    std::uint64_t hash = 0;
    ImpactResult impact;
    int count(EventType t) const { return events[static_cast<std::size_t>(t)]; }
};

// A residential street of six poles (35 m spans), settled for half a second,
// then a 1,300 kg car into pole `hit` at bumper height (0.5 m).
Outcome street(Real kmh, int hit, Real seconds, const char* traceName, int transformerAt = -1, int transformerAt2 = -1,
               Real wetness = 0.0) {
    pdscn::StreetSpec spec;
    spec.transformerAt = transformerAt;
    spec.transformerAt2 = transformerAt2;
    spec.noTransformers = true;
    Outcome o;
    World w(1);
    w.params.sim.wetness = wetness;
    w.loadTile(pdscn::streetTile(spec, &o.ids));
    w.loadLanes(pdscn::streetLanes(spec));
    pdscn::Trace t;
    t.poles = o.ids;
    for (int i = 0; i < 5; ++i) t.spans.push_back(i);
    for (int i = 0; i < 30; ++i) w.step(1.0 / 60.0);
    {
        Event drop[64];
        while (w.pollEvents(drop, 64) > 0) {}
    }
    o.impact = pdscn::hitPole(w, o.ids[static_cast<std::size_t>(hit)], kmh);
    o.events.assign(32, 0);
    t.record(w);
    const int steps = static_cast<int>(seconds * 60.0 + 0.5);
    Event ev[256];
    for (int i = 0; i < steps; ++i) {
        w.step(1.0 / 60.0);
        int n;
        while ((n = w.pollEvents(ev, 256)) > 0)
            for (int k = 0; k < n; ++k) {
                ++o.events[static_cast<std::size_t>(ev[k].type)];
                o.all.push_back(ev[k]);
                char buf[200];
                std::snprintf(buf, sizeof buf, "%.3f %s pole=%d span=%d wire=%d value=%.1f ids=%lld", w.time(),
                              pdscn::eventName(ev[k].type), ev[k].pole, ev[k].span, ev[k].wire, ev[k].value,
                              static_cast<long long>(ev[k].ids));
                t.events.emplace_back(buf);
            }
        t.record(w);
    }
    for (std::size_t i = 0; i < o.ids.size(); ++i) {
        PoleQuery q;
        w.queryPole(o.ids[i], q);
        o.state.push_back(q.state);
        o.tilt.push_back(q.tiltDeg);
        o.powered.push_back(q.powered);
        o.section.push_back(q.section);
        o.maxTilt.push_back(*std::max_element(t.tilt[i].begin(), t.tilt[i].end()));
    }
    o.hash = w.stateHash();
    if (traceName) {
        const std::string base = std::string(PD_TRACE_DIR) + "/" + traceName;
        t.writeCsv(base + ".csv");
        t.writeJson(base + ".json", traceName);
    }
    std::printf("    %s: impact peak %.0f kN, dv %.2f m/s; states", traceName ? traceName : "-", o.impact.peakForce / 1000,
                o.impact.vehicleDeltaV);
    for (std::size_t i = 0; i < o.ids.size(); ++i) std::printf(" %d(%.1f°)", o.state[i], o.tilt[i]);
    std::printf("\n");
    return o;
}

int at(const Outcome& o, int i) { return o.state[static_cast<std::size_t>(i)]; }

}  // namespace

TEST_CASE("street: 30 km/h at 0.5 m cracks the pole and nothing else") {
    const Outcome o = street(30, 2, 6, "street_30kmh");
    CHECK(at(o, 2) == static_cast<int>(PoleState::Cracked));
    CHECK_RANGE(o.tilt[2], 0.3, 5.0);
    for (int i : {0, 1, 3, 4, 5}) CHECK(at(o, i) == 0);
    CHECK(o.count(EventType::PoleCracked) == 1);
    CHECK(o.count(EventType::WireSnapped) == 0);
    CHECK(o.count(EventType::PoleSnapped) == 0);
}

TEST_CASE("street: 50 km/h leans the pole in the soil") {
    const Outcome o = street(50, 2, 6, "street_50kmh");
    CHECK(at(o, 2) == static_cast<int>(PoleState::Leaning));
    CHECK_RANGE(o.tilt[2], 5.0, 20.0);
    for (int i : {0, 1, 3, 4, 5}) CHECK(at(o, i) <= static_cast<int>(PoleState::Cracked));
    CHECK(o.count(EventType::PoleSnapped) == 0);
}

TEST_CASE("street: 80 km/h snaps the pole and the line takes the load") {
    const Outcome o = street(80, 2, 8, "street_80kmh");
    CHECK(at(o, 2) >= static_cast<int>(PoleState::Snapped));
    CHECK(o.impact.broke);
    CHECK(o.count(EventType::PoleSnapped) == 1);
    // The snapped top hangs in (or falls out of) its wires: well past any lean.
    CHECK(o.maxTilt[2] > 20.0);
    // Its neighbours feel it: the line pulls them over (a domino needs weak or wet ground; see the next case).
    CHECK(o.maxTilt[1] > 0.05);
    CHECK(o.maxTilt[3] > 0.05);
}

TEST_CASE("street: 80 km/h on saturated ground can pull neighbours over (domino)") {
    const Outcome o = street(80, 1, 10, "street_80kmh_wet", -1, -1, 1.0);
    CHECK(at(o, 1) >= static_cast<int>(PoleState::Snapped));
    int damaged = 0;
    for (int i : {0, 2, 3, 4, 5}) damaged += at(o, i) >= static_cast<int>(PoleState::Cracked);
    std::printf("    neighbours damaged on wet ground: %d\n", damaged);
    // Not asserted to fall: whether the domino runs is the documented open point.
    CHECK(o.maxTilt[0] + o.maxTilt[2] > 0.1);
}

TEST_CASE("street: 60 km/h into a transformer pole detaches it and cuts power to its section only") {
    const Outcome o = street(60, 1, 8, "street_transformer_60kmh", 1, 4);
    CHECK(o.count(EventType::TransformerDetached) == 1);
    CHECK(o.count(EventType::TransformerLanded) == 1);
    CHECK(o.count(EventType::PowerOut) >= 1);
    // The section fed by the transformer on pole 1 is out; the one on pole 4 is not.
    const std::int32_t lostSection = o.ids[1];
    bool sawLost = false;
    for (const Event& e : o.all)
        if (e.type == static_cast<int>(EventType::PowerOut)) {
            CHECK(e.ids != o.ids[4]);
            sawLost |= e.ids == lostSection;
        }
    CHECK(sawLost);
    for (std::size_t i = 0; i < o.ids.size(); ++i) {
        if (o.section[i] == o.ids[4]) CHECK(o.powered[i] == 1);
        if (o.section[i] == lostSection) CHECK(o.powered[i] == 0);
    }
}

TEST_CASE("street: 30 km/h into a transformer pole keeps the transformer") {
    const Outcome o = street(30, 1, 4, "street_transformer_30kmh", 1, 4);
    CHECK(o.count(EventType::TransformerDetached) == 0);
    CHECK(o.count(EventType::PowerOut) == 0);
    for (int p : o.powered) CHECK(p == 1);
}

TEST_CASE("street: determinism — two identical runs give the identical state hash") {
    const Outcome a = street(80, 2, 5, nullptr, 1, 4);
    const Outcome b = street(80, 2, 5, nullptr, 1, 4);
    std::printf("    hash %016llx / %016llx\n", static_cast<unsigned long long>(a.hash), static_cast<unsigned long long>(b.hash));
    CHECK(a.hash == b.hash);
    CHECK(a.all.size() == b.all.size());
    const Outcome c = street(79, 2, 5, nullptr, 1, 4);
    CHECK(a.hash != c.hash);  // and the hash does see a change
}

TEST_CASE("street: an idle world costs nothing") {
    World w(1);
    w.loadTile(pdscn::streetTile(pdscn::StreetSpec{}));
    int internal = 0;
    for (int i = 0; i < 600; ++i) internal += w.step(1.0 / 60.0);
    CHECK(internal == 0);
    CHECK(w.stats().awakeSpans == 0);
    CHECK(w.stats().bodies == 0);
}

TEST_CASE("street: the world settles back to sleep after a snap") {
    pdscn::StreetSpec spec;
    spec.noTransformers = true;
    std::vector<std::int32_t> ids;
    World w(1);
    w.loadTile(pdscn::streetTile(spec, &ids));
    pdscn::hitPole(w, ids[2], 80);
    for (int i = 0; i < 60 * 20; ++i) w.step(1.0 / 60.0);
    const Stats s = w.stats();
    std::printf("    after 20 s: awake spans %d, bodies %d\n", s.awakeSpans, s.bodies);
    CHECK(s.awakeSpans <= 4);
    int internal = 0;
    for (int i = 0; i < 60; ++i) internal += w.step(1.0 / 60.0);
    CHECK(internal <= 60 * w.params.sim.substeps);
}

TEST_CASE("street: tile state saves, reloads on a fresh world, and a repair restores the pole") {
    pdscn::StreetSpec spec;
    spec.noTransformers = true;
    std::vector<std::int32_t> ids;
    World w(1);
    const std::string tile = pdscn::streetTile(spec, &ids);
    w.loadTile(tile);
    pdscn::hitPole(w, ids[2], 50);
    for (int i = 0; i < 60 * 6; ++i) w.step(1.0 / 60.0);
    PoleQuery before;
    w.queryPole(ids[2], before);
    CHECK(before.state == static_cast<int>(PoleState::Leaning));
    std::uint8_t small[16];
    const int need = w.saveTileState(0, 0, small, sizeof small);
    CHECK(need < 0);  // too small: it says how much it needs
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(-need));
    const int n = w.saveTileState(0, 0, buf.data(), static_cast<int>(buf.size()));
    CHECK(n == -need);
    std::printf("    tile diff: %d bytes\n", n);
    World w2(1);
    w2.loadTile(tile);
    CHECK(w2.loadTileState(0, 0, buf.data(), n) > 0);
    PoleQuery after;
    w2.queryPole(ids[2], after);
    CHECK(after.state == before.state);
    CHECK_NEAR(after.tiltDeg, before.tiltDeg, 0.01);
    // Unloading keeps the diff; loading the tile again brings it back.
    w2.unloadTile(0, 0);
    PoleQuery gone;
    CHECK(!w2.queryPole(ids[2], gone));
    w2.loadTile(tile);
    w2.queryPole(ids[2], after);
    CHECK(after.state == before.state);
    CHECK(w2.repairPole(ids[2]));
    w2.queryPole(ids[2], after);
    CHECK(after.state == 0);
    CHECK_NEAR(after.tiltDeg, 0.0, 1e-9);
}

TEST_CASE("street: 100 km/h brings the line down: neighbours dragged, live MV on the road arcs, lanes blocked, power out") {
    const Outcome o = street(100, 2, 10, "street_100kmh_domino", 4);
    CHECK(at(o, 2) >= static_cast<int>(PoleState::Snapped));
    CHECK(o.maxTilt[2] > 60.0);  // the top is on the ground
    // The falling top drags its neighbours: they are left permanently tilted in the soil.
    int pulled = 0;
    for (int i : {1, 3}) pulled += o.tilt[static_cast<std::size_t>(i)] > 0.4;
    CHECK(pulled >= 1);
    CHECK(std::max(o.tilt[0], o.tilt[1]) > 2.0);
    CHECK(o.count(EventType::WireSnapped) >= 3);
    CHECK(o.count(EventType::WireOnGroundLive) >= 1);
    CHECK(o.count(EventType::Arc) >= 1);
    CHECK(o.count(EventType::LaneBlocked) >= 1);
    CHECK(o.count(EventType::PowerOut) >= 1);
    std::printf("    pulled over %d, wires snapped %d, live on ground %d, arcs %d, lanes blocked %d\n", pulled,
                o.count(EventType::WireSnapped), o.count(EventType::WireOnGroundLive), o.count(EventType::Arc),
                o.count(EventType::LaneBlocked));
    // Every LANE_BLOCKED carries a lane of the street's lanes tile (T_p0_p0, edge 0).
    for (const Event& e : o.all)
        if (e.type == static_cast<int>(EventType::LaneBlocked)) {
            CHECK(e.ids == packLane(0, 0, 0, 0) || e.ids == packLane(0, 0, 0, 1));
        }
}
