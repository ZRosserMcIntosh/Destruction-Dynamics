// Street Dynamics — the world: tiles, persistence, repair, weather, allocation, cost.
// SPDX-License-Identifier: MIT
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#include "sd/world.hpp"
#include "sd_test.hpp"

namespace {
std::atomic<long> gAllocs{0};
bool gCounting = false;
}  // namespace

void* operator new(std::size_t n) {
    if (gCounting) gAllocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace sd;

std::string libraryJson();

namespace {

// A street: a row of each type every 6 m along y = 0, facing south (the road).
std::string streetTile(int perType) {
    const char* types[] = {"stop_sign_r1", "regulatory_r19", "warning_diamond", "street_name_double", "bollard_cast_iron",
                           "bollard_steel", "bollard_concrete", "bike_rack_u"};
    std::string s = "{\"tile\":\"T_p0_p0\",\"v\":1,\"objects\":[";
    int k = 0;
    for (int r = 0; r < perType; ++r)
        for (const char* t : types) {
            if (k) s += ",";
            s += "[\"" + std::string(t) + "\"," + std::to_string(k * 6.0 - 100) + ",0.0,180]";
            ++k;
        }
    return s + "]}";
}

// A box driving along y = -1.5 .. through the row (east), then turning in: it hits a few.
void drive(World& w, int steps, Real x0, Real speed) {
    HostBody hb;
    hb.id = 3;
    hb.half = {2.0, 0.9, 0.35};
    hb.centre = {x0, -0.6, 0.6};
    hb.velocity = {speed, 0, 0};
    for (int i = 0; i < steps; ++i) {
        w.setBodies(&hb, 1);
        w.step(1.0 / 60);
        hb.centre += hb.velocity * (1.0 / 60);
    }
}

}  // namespace

TEST_CASE("world: a tile loads every type; IDs are the builder's seeds; a second load adds nothing") {
    World w(1);
    CHECK(w.loadLibrary(libraryJson()).empty());
    CHECK(w.loadTile(streetTile(2)) == 16);
    CHECK(w.objectCount() == 16);
    CHECK(w.objectId(0) == objectSeed(-100, 0));
    CHECK(w.loadTile(streetTile(2)) == 16);
    CHECK(w.objectCount() == 16);
    CHECK(w.loadTile("{\"tile\":\"T_p0_p0\"}") == -1);
}

TEST_CASE("world: no heap allocation inside step() with a car ploughing through a street") {
    World w(1);
    w.loadLibrary(libraryJson());
    w.loadTile(streetTile(3));
    drive(w, 2, -110, 15);  // warm-up: the first contacts
    gAllocs = 0;
    gCounting = true;
    drive(w, 600, -105, 15);
    gCounting = false;
    std::printf("    allocations in 10 s of contacts: %ld\n", gAllocs.load());
    CHECK(gAllocs.load() == 0);
}

TEST_CASE("world: cost: a street of 160 objects, a car ploughing through them") {
    World w(1);
    w.loadLibrary(libraryJson());
    w.loadTile(streetTile(20));
    HostBody hb;
    hb.id = 3;
    hb.half = {2.0, 0.9, 0.35};
    hb.centre = {-110, -0.6, 0.6};
    hb.velocity = {15, 0, 0};
    double sum = 0, worst = 0;
    int n = 0, maxAwake = 0;
    for (int i = 0; i < 600; ++i) {
        w.setBodies(&hb, 1);
        const auto t0 = std::chrono::steady_clock::now();
        w.step(1.0 / 60);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        hb.centre += hb.velocity * (1.0 / 60);
        sum += ms, worst = std::max(worst, ms), ++n;
        maxAwake = std::max(maxAwake, w.stats().awake);
    }
    std::printf("    160 objects, up to %d awake: mean %.3f ms, worst %.3f ms per 60 Hz step\n", maxAwake, sum / n, worst);
#if defined(NDEBUG) && !defined(__SANITIZE_ADDRESS__)
    CHECK(sum / n < 2.0);  // the budget holds for optimised builds only
#endif
    // and idle: nothing awake, nothing run
    for (int i = 0; i < 600; ++i) w.setBodies(nullptr, 0), w.step(1.0 / 60);
    std::printf("    10 s after the car: %d awake\n", w.stats().awake);
    CHECK(w.stats().awake == 0);
}

TEST_CASE("world: damage persists through unload and a fresh world; repair puts it back") {
    World w(1);
    w.loadLibrary(libraryJson());
    const std::string tile = streetTile(1);
    w.loadTile(tile);
    drive(w, 240, -110, 12);
    int damaged = 0;
    for (int i = 0; i < w.objectCount(); ++i) damaged += w.objectState(i) != ObjectState::Intact;
    CHECK(damaged >= 3);
    std::vector<std::uint8_t> buf(64 * 1024);
    const int n = w.saveTileState(0, 0, buf.data(), static_cast<int>(buf.size()));
    CHECK(n > 16);
    World w2(1);
    w2.loadLibrary(libraryJson());
    w2.loadTile(tile);
    CHECK(w2.loadTileState(0, 0, buf.data(), n) == damaged);
    for (int i = 0; i < w.objectCount(); ++i) {
        CHECK(w2.objectState(i) == w.objectState(i));
        CHECK_NEAR(w2.objectTilt(i), w.objectTilt(i), 1e-9);
    }
    // unload keeps the diff; loading the tile again brings it back
    w2.unloadTile(0, 0);
    w2.loadTile(tile);
    int again = 0;
    for (int i = 0; i < w2.objectCount(); ++i) again += w2.objectState(i) != ObjectState::Intact;
    CHECK(again == damaged);
    // repair
    for (int i = 0; i < w2.objectCount(); ++i) CHECK(w2.repair(w2.objectId(i)));
    for (int i = 0; i < w2.objectCount(); ++i) {
        CHECK(w2.objectState(i) == ObjectState::Intact);
        CHECK(w2.objectTilt(i) < 1e-9);
    }
}

TEST_CASE("world: saturated ground: a footing in wet soil leans further") {
    // The shipped types all bend their post before the footing turns; here a stout steel bollard
    // on a shallow footing, so the soil governs (data, not code: a JSON entry added to the library).
    const char* shallow = R"({"types":{"bollard_shallow":{
        "name":"Steel bollard, shallow footing","family":"bollard","height":0.8,"segments":2,
        "section":{"kind":"solid_round","d":0.16},"material":"steel_s235",
        "base":{"kind":"footing","footing_stiffness":4.0e5,"footing_yield":6000,"footing_fail":0.5},
        "panels":[]}}})";
    auto lean = [&](Real wet) {
        World w(1);
        w.loadLibrary(libraryJson());
        CHECK(w.loadLibrary(shallow).empty());
        w.wetness = wet;
        const int oi = w.addObject("bollard_shallow", 0, 0, 180);
        // the same blow in both soils: 300 N s at 0.6 m (a slow bumper shove), along +y
        w.applyImpact(w.objectId(oi), {0, -0.08, 0.6}, {0, -1, 0}, 300);
        for (int i = 0; i < 240; ++i) w.setBodies(nullptr, 0), w.step(1.0 / 60);
        CHECK(w.objectState(oi) == ObjectState::Bent);
        return w.objectTilt(oi);
    };
    const Real dry = lean(0), wet = lean(1);
    std::printf("    shallow-footed bollard, one 300 N s blow, lean: dry %.1f deg, saturated %.1f deg\n",
                dry / kDeg, wet / kDeg);
    CHECK(dry > 1 * kDeg);
    CHECK(wet > dry + 0.3 * kDeg);
}

TEST_CASE("world: fuzz: random hits on every type never fault") {
    std::uint32_t r = 12345;
    auto rnd = [&] { r = r * 1664525u + 1013904223u; return static_cast<Real>(r >> 8) / 16777216.0; };
    World probe(1);
    probe.loadLibrary(libraryJson());
    for (int trial = 0; trial < 60; ++trial) {
        World w(1);
        w.loadLibrary(libraryJson());
        const int type = trial % probe.library.size();
        const int oi = w.addObject(probe.library.at(type).id, 0, 0, rnd() * 360);
        const Real speed = 1 + rnd() * 40, yaw = (rnd() - 0.5) * 2.4, lat = (rnd() - 0.5) * 1.6;
        HostBody hb;
        hb.id = 1;
        hb.half = {2.0, 0.9, 0.3 + 0.4 * rnd()};
        hb.orientation = {std::cos(0.5 * yaw), 0, 0, std::sin(0.5 * yaw)};
        const Vec3 dir{std::cos(yaw), std::sin(yaw), 0};
        hb.centre = Vec3{lat * -dir.y, lat * dir.x, 0.25 + hb.half.z} - dir * 3.5;
        hb.velocity = dir * speed;
        for (int i = 0; i < 150; ++i) {
            w.setBodies(&hb, 1);
            w.step(1.0 / 60);
            hb.centre += hb.velocity * (1.0 / 60);
        }
        float buf[512];
        const int n = w.parts(oi, buf, 512);
        bool finite = n > 0;
        for (int k = 0; k < n; ++k) finite = finite && std::isfinite(buf[k]);
        CHECK(finite);
        CHECK(w.objectTopHeight(oi) > -0.5);  // nothing under the ground
    }
}
