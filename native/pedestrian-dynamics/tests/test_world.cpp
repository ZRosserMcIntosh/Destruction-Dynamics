// Pedestrian Dynamics — the world: children and corners, blows, allocation, cost, sleep, fuzz.
// SPDX-License-Identifier: MIT
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>

#include "ped/rot.hpp"
#include "ped/world.hpp"
#include "ped_test.hpp"

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

using namespace ped;

std::string bodyJson();
std::string vehiclesJson();

namespace {

World makeWorld() {
    World w(1);
    w.loadBody(bodyJson());
    w.loadVehicles(vehiclesJson());
    return w;
}

// A car driving at constant speed along +x through `w` for `steps`, feeding back nothing (a
// heavy truck) unless `react`.
void drive(World& w, const char* profile, Real kmh, Real y, int steps, Real x0 = -3.0) {
    VehicleState vs;
    vs.id = 9;
    vs.profile = w.vehicles.index(profile);
    vs.position = {x0, y, 0};
    vs.velocity = {kmh / 3.6, 0, 0};
    for (int k = 0; k < steps; ++k) {
        w.setVehicles(&vs, 1);
        w.step(1.0 / 60);
        vs.position += vs.velocity * (1.0 / 60);
    }
}

Real lowest(const World& w, int i) {
    float buf[16 * 14];
    const int n = w.parts(i, buf, 16 * 14);
    Real lo = 1e9;
    for (int s = 0; s < n / 14; ++s) {
        const float* p = buf + s * 14;
        const Quat q{p[3], p[4], p[5], p[6]};
        const Vec3 x{p[0], p[1], p[2]};
        const Vec3 a = x + rotate(q, Vec3{p[7], p[8], p[9]}), b = x + rotate(q, Vec3{p[10], p[11], p[12]});
        lo = std::min(lo, std::min(a.z, b.z) - p[13]);
    }
    return lo;
}

}  // namespace

TEST_CASE("world: a child in front of a sedan is projected forward, an adult wraps") {
    for (Real stature : {1.20, 1.75}) {
        World w = makeWorld();
        const Real mass = stature < 1.5 ? 23.0 : 75.0;
        const int i = w.addPedestrian(0, 0, 0.5 * kPi, stature, mass);
        w.setAssist(i, 1);
        drive(w, "sedan", 35, 0, 60 * 6, -2.6);
        const Report& r = w.report(i);
        std::printf("    %.2f m, %2.0f kg: %-18s pelvis at impact %.2f m (bonnet edge %.2f), head %s\n", stature, mass,
                    trajectoryName(r.trajectory), r.pelvisHeightAtImpact, r.bleHeight,
                    r.headRegion >= 0 ? regionName(static_cast<Region>(r.headRegion)) : "-");
        CHECK(r.struck);
        if (stature < 1.5) CHECK(r.trajectory == Trajectory::ForwardProjection);
        else CHECK(r.trajectory == Trajectory::Wrap || r.trajectory == Trajectory::RoofVault);
    }
}

TEST_CASE("world: struck by the corner, a person goes over the wing (fender vault) or is pushed aside") {
    World w = makeWorld();
    const int i = w.addPedestrian(0, 0.78, 0.5 * kPi);  // at the car's left corner
    w.setAssist(i, 1);
    drive(w, "sedan", 40, 0, 60 * 6, -2.6);
    const Report& r = w.report(i);
    std::printf("    corner strike at 40 km/h: %s, head %s, thrown %.1f m\n", trajectoryName(r.trajectory),
                r.headRegion >= 0 ? regionName(static_cast<Region>(r.headRegion)) : "-", r.throwDistance);
    CHECK(r.struck);
    CHECK(r.trajectory == Trajectory::FenderVault || r.trajectory == Trajectory::Wrap ||
          r.trajectory == Trajectory::ForwardProjection);
}

TEST_CASE("world: a blow staggers; a hard one takes the legs away") {
    for (Real J : {15.0, 120.0}) {
        World w = makeWorld();
        const int i = w.addPedestrian(0, 0, 0);
        w.setAssist(i, 1);
        for (int k = 0; k < 30; ++k) w.step(1.0 / 60);
        const int chest = w.body.index("thorax");
        w.applyImpulse(i, chest, w.segmentPosition(i, chest), {J, 0, 0});
        // the host's reaction: the assist goes when the strength does
        w.setAssist(i, w.strength(i) > 0.7 ? 1.0 : 0.0);
        for (int k = 0; k < 180; ++k) w.step(1.0 / 60);
        const Real pelvis = w.segmentPosition(i, 0).z;
        std::printf("    %3.0f N s to the chest: strength %.2f, pelvis %.2f m after 3 s\n", J, w.strength(i), pelvis);
        if (J < 50) CHECK(pelvis > 0.85);
        else CHECK(pelvis < 0.5);
    }
}

TEST_CASE("world: no heap allocation in step(), with a car going through a crowd") {
    World w = makeWorld();
    for (int k = 0; k < 6; ++k) {
        const int i = w.addPedestrian(2.0 * k, 0.3 * (k % 3), 0.5 * kPi);
        w.setAssist(i, 1);
    }
    drive(w, "sedan", 30, 0, 2);
    gAllocs = 0;
    gCounting = true;
    drive(w, "sedan", 30, 0, 300, -2.5);
    gCounting = false;
    std::printf("    allocations in 5 s of a car ploughing through six people: %ld\n", gAllocs.load());
    CHECK(gAllocs.load() == 0);
}

TEST_CASE("world: cost: one person struck; forty more standing asleep cost nothing") {
    World w = makeWorld();
    for (int k = 0; k < 40; ++k) {
        const int i = w.addPedestrian(-20.0 + k % 8, 20.0 + k / 8, 0);
        w.setAssist(i, 1);
    }
    for (int k = 0; k < 120; ++k) w.step(1.0 / 60);  // they settle and sleep
    const int awakeIdle = w.stats().awake;
    const int victim = w.addPedestrian(0, 0, 0.5 * kPi);
    w.setAssist(victim, 1);
    VehicleState vs;
    vs.id = 1;
    vs.profile = w.vehicles.index("sedan");
    vs.position = {-2.6, 0, 0};
    vs.velocity = {40 / 3.6, 0, 0};
    double sum = 0, worst = 0;
    int n = 0;
    for (int k = 0; k < 120; ++k) {
        w.setVehicles(&vs, 1);
        const auto t0 = std::chrono::steady_clock::now();
        w.step(1.0 / 60);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        sum += ms, worst = std::max(worst, ms), ++n;
        vs.position += vs.velocity * (1.0 / 60);
    }
    std::printf("    40 asleep (%d awake after settling); one struck at 40 km/h: mean %.3f ms, worst %.3f ms per 60 Hz step\n",
                awakeIdle, sum / n, worst);
    CHECK(awakeIdle == 0);
#if defined(NDEBUG) && !defined(__SANITIZE_ADDRESS__)
    CHECK(sum / n < 0.5);
#endif
}

TEST_CASE("world: fuzz: random cars, speeds, angles and people never fault") {
    std::uint32_t r = 2024;
    auto rnd = [&] { r = r * 1664525u + 1013904223u; return static_cast<Real>(r >> 8) / 16777216.0; };
    const char* profiles[] = {"sedan", "hatch", "suv", "van"};
    for (int trial = 0; trial < 24; ++trial) {
        World w = makeWorld();
        const Real stature = 1.1 + 0.8 * rnd();
        const int i = w.addPedestrian(0, (rnd() - 0.5) * 1.6, rnd() * kTwoPi, stature, 20 + 80 * (stature - 1.0));
        w.setAssist(i, rnd() > 0.3 ? 1 : 0);
        w.setStrength(i, rnd());
        drive(w, profiles[trial % 4], 5 + 85 * rnd(), 0, 60 * 3, -2.4 - 2 * rnd());
        bool finite = true;
        for (int s = 0; s < w.segmentCount(); ++s) finite = finite && isFinite(w.segmentPosition(i, s));
        CHECK(finite);
        CHECK(lowest(w, i) > -0.08);
        CHECK(std::isfinite(w.report(i).hic15));
    }
}
