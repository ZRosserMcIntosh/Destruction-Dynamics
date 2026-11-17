// Pole Dynamics — world data: the seed formula, tiles, spans across tiles, lanes.
// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstring>

#include "pd/c_api.h"
#include "pd/world.hpp"
#include "pd_test.hpp"
#include "scenarios.hpp"

using namespace pd;

TEST_CASE("data: Python round() rounds halves to even") {
    CHECK(pythonRound(0.5) == 0);
    CHECK(pythonRound(1.5) == 2);
    CHECK(pythonRound(2.5) == 2);
    CHECK(pythonRound(-0.5) == 0);
    CHECK(pythonRound(-1.5) == -2);
    CHECK(pythonRound(-2.6) == -3);
    CHECK(pythonRound(1234.4999) == 1234);
}

TEST_CASE("data: pole seeds and attributes match the world builder (Python)") {
    // Generated with the builder's own expressions in CPython 3 (class 2 = secondary).
    struct Ref { double x, y; std::int32_t seed; int transformer, light, telecom, drops; };
    const Ref refs[] = {
        {0, 0, 0, 1, 1, 8, 0},
        {12.5, -3.25, 2102574409, 1, 1, 10, 1},
        {-1234.56, 789.01, 958305920, 0, 1, 8, 3},
        {0.05, 0.15, 38699326, 0, 0, 12, 2},
        {0.25, -0.35, 1940854078, 0, 1, 8, 0},
        {-0.05, 2.45, 464391912, 1, 1, 12, 3},
        {4999.95, -4999.95, 1909294400, 1, 1, 12, 0},
        {-250.0, 1250.0, 1376448608, 1, 0, 12, 0},
        {31.45, -17.85, 1425189728, 0, 0, 11, 3},
        {100000.123, -98765.4321, 397364187, 0, 1, 9, 1},
    };
    for (const Ref& r : refs) {
        const PoleSpec p = makePole(r.x, r.y, 0, 2);
        CHECK(p.id == r.seed);
        CHECK(static_cast<int>(p.transformer) == r.transformer);
        CHECK(static_cast<int>(p.light) == r.light);
        CHECK(p.telecomCables == r.telecom);
        CHECK(p.serviceDrops == r.drops);
        CHECK(pd_pole_seed(r.x, r.y) == r.seed);
    }
}

TEST_CASE("data: a pole is 1,000-1,400 kg with its hardware, slender") {
    const PoleSpec s = makePole(10, 10, 0, 0);
    const MassProps m = massProperties(PoleGeometry{}, s, -1.5, 11.0, true);
    CHECK_RANGE(m.mass, 1000.0, 1700.0);
    const MassProps bare = massProperties(PoleGeometry{}, s, -1.5, 11.0, false);
    CHECK_RANGE(bare.mass, 1000.0, 1400.0);
    CHECK_RANGE(bare.comHeight, 2.5, 5.5);
    CHECK(bare.inertiaPerp > 100.0 * bare.inertiaAxial);
}

TEST_CASE("data: a tile loads, spans link poles by position, drops reach the houses") {
    World w(1);
    pdscn::StreetSpec spec;
    std::vector<std::int32_t> ids;
    CHECK(w.loadTile(pdscn::streetTile(spec, &ids)) == 6);
    int lineSpans = 0, drops = 0;
    for (int i = 0; i < w.spanCount(); ++i) {
        std::int32_t a, b;
        Real t;
        w.spanInfo(i, a, b, t);
        if (b >= 0) ++lineSpans; else ++drops;
        CHECK(w.spanMode(i) == SpanMode::Analytic);
    }
    CHECK(lineSpans == 5);
    for (int i = 0; i < w.poleCount(); ++i) {
        PoleQuery q;
        CHECK(w.queryPole(w.poleId(i), q));
        CHECK(q.state == 0);
        CHECK_NEAR(q.tiltDeg, 0.0, 1e-9);
    }
    // Each pole's drops come from its seed: ((s / 100000) % 4).
    int expectDrops = 0;
    for (auto id : ids) expectDrops += (id / 100000) % 4;
    CHECK(drops == expectDrops);
}

TEST_CASE("data: a span across a tile border links when the second tile loads") {
    World w(1);
    // Tile A has pole 1 and the span; tile B has pole 2 and the same span.
    const char* a = "{\"tile\":\"T_p0_p0\",\"v\":1,\"poles\":[[240.0,0.0,90,0]],\"spans\":[[240.0,0.0,275.0,0.0]]}";
    const char* b = "{\"tile\":\"T_p1_p0\",\"v\":1,\"poles\":[[275.0,0.0,90,0]],\"spans\":[[240.0,0.0,275.0,0.0]]}";
    CHECK(w.loadTile(a) == 1);
    std::int32_t pa, pb;
    Real t;
    w.spanInfo(0, pa, pb, t);
    CHECK(pa == poleSeed(240.0, 0.0));
    CHECK(pb == -1);  // the far pole is in a tile that is not loaded yet
    CHECK(w.loadTile(b) == 1);
    int line = 0;
    for (int i = 0; i < w.spanCount(); ++i) {
        w.spanInfo(i, pa, pb, t);
        if (pa >= 0 && pb >= 0) ++line;
    }
    CHECK(line == 1);  // one span, not two
}

TEST_CASE("data: rest catenary sag and tension (MV, 35 m)") {
    World w(1);
    pdscn::StreetSpec spec;
    w.loadTile(pdscn::streetTile(spec));
    float buf[4096];
    const int n = w.wireCurve(0, 33, buf, 4096);
    CHECK(n > 0);
    // Wire 0 (MV): kind, broken, samples, then points. Sag = w L^2 / (8 H).
    const int samples = static_cast<int>(buf[3]);
    const float* p = buf + 4;
    const double l = std::hypot(p[3 * (samples - 1)] - p[0], p[3 * (samples - 1) + 1] - p[1]);
    const double mid = 0.5 * (p[2] + p[3 * (samples - 1) + 2]) - p[3 * (samples / 2) + 2];
    CHECK_NEAR(mid, 0.216 * kGravity * l * l / (8 * 2400.0), 0.002);
    CHECK_NEAR(w.spanTension(0, 0), 2400.0, 1e-6);
}

TEST_CASE("data: lanes tiles map to world metres and lane IDs pack and unpack") {
    const std::int64_t id = packLane(-10, 3, 1234, 2);
    int tx, ty, e, l;
    pd_unpack_lane(id, &tx, &ty, &e, &l);
    CHECK(tx == -10);
    CHECK(ty == 3);
    CHECK(e == 1234);
    CHECK(l == 2);
    World w(1);
    CHECK(w.loadLanes(pdscn::streetLanes(pdscn::StreetSpec{})) == 1);
}

#include "pd/unity_frame.hpp"

TEST_CASE("data: Unity frame conversion commutes with rotation") {
    // A sim rotation applied then converted equals the converted rotation
    // applied to the converted vector (Unity's own quaternion maths).
    const Quat q = normalized(Quat{0.8, 0.1, -0.3, 0.5});
    const Vec3 v{1.5, -2.0, 0.7};
    const Vec3 a = unity::toUnity(rotate(q, v));
    const Vec3 b = rotate(unity::toUnity(q), unity::toUnity(v));
    CHECK_NEAR(a.x, b.x, 1e-12);
    CHECK_NEAR(a.y, b.y, 1e-12);
    CHECK_NEAR(a.z, b.z, 1e-12);
    const Vec3 r = unity::fromUnity(unity::toUnity(v));
    CHECK_NEAR(r.y, v.y, 0.0);
}
