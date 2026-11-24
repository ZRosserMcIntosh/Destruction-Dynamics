// Street Dynamics — posts and signs against a moving box (no vehicle model).
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cstdio>

#include "sd/world.hpp"
#include "sd_test.hpp"

using namespace sd;

std::string libraryJson();

namespace {

struct Push {
    Real peakForce = 0, tilt = 0, plastic = 0;
    int state = 0;
    std::vector<int> events;
    bool panelOff = false;
    Real stub = 0;
};

// A 1.8 m wide box `tall` high (a car's front) driven at `speed` into object `type` at the
// origin from 5 cm away, its lower face `low` above the ground, for `seconds`.
Push push(const char* type, Real speed, Real seconds, Real low = 0.25, Real wet = 0, Real tall = 0.5) {
    World w(1);
    w.loadLibrary(libraryJson());
    w.wetness = wet;
    const int oi = w.addObject(type, 0, 0, 180);  // facing south, toward the box
    Push r;
    r.events.assign(16, 0);
    const Real dt = 1.0 / 60;
    // a car's front: a bumper-height box (the bonnet above it is the vehicle model's)
    const Vec3 half{0.9, 0.9, 0.5 * tall};
    const Real rad = 0.5 * w.objectType(oi).section.d;
    Vec3 c{0, -(half.y + rad + 0.05), low + half.z};
    for (Real t = 0; t < seconds; t += dt) {
        HostBody hb;
        hb.id = 7;
        hb.centre = c;
        hb.half = half;
        hb.velocity = {0, speed, 0};
        w.setBodies(&hb, 1);
        w.step(dt);
        const Vec3 j = w.bodyImpulse(7);
        r.peakForce = std::max(r.peakForce, length(j) / dt);
        c += hb.velocity * dt;
        Event ev[64];
        int n;
        while ((n = w.pollEvents(ev, 64)) > 0)
            for (int k = 0; k < n; ++k) ++r.events[static_cast<std::size_t>(ev[k].type)];
    }
    r.tilt = w.objectTilt(oi);
    r.plastic = w.objectMaxPlastic(oi);
    r.state = static_cast<int>(w.objectState(oi));
    r.panelOff = w.objectType(oi).panels.empty() ? false : !w.objectPanelAttached(oi, 0);
    r.stub = w.objectStubHeight(oi);
    return r;
}

}  // namespace

TEST_CASE("post: a sign left alone stands still and costs nothing") {
    World w(1);
    w.loadLibrary(libraryJson());
    const int oi = w.addObject("stop_sign_r1", 10, 10, 0);
    int sub = 0;
    for (int i = 0; i < 600; ++i) sub += w.step(1.0 / 60);
    CHECK(sub == 0);
    CHECK(w.objectTilt(oi) < 1e-9);
    CHECK(w.stats().awake == 0);
}

TEST_CASE("post: pushed slowly at bumper height, it yields at its plastic moment over the arm") {
    World probe(1);
    probe.loadLibrary(libraryJson());
    const Archetype& a = *probe.library.find("stop_sign_r1");
    const Push p = push("stop_sign_r1", 0.5, 1.5, 0.3);
    // the bumper (0.3-0.8 m) bears on the post at its top edge, 0.8 m up once the post leans:
    // the section at the ground governs (the footing is stronger), F = M / arm.
    const Real mLo = a.mech.yieldMoment * 0.9, mHi = a.mech.peakMoment * 1.1;
    std::printf("    slow push: peak %.2f kN (M_el / 0.8 m = %.2f, M_peak / 0.3 m = %.2f), tilt %.1f deg, plastic %.2f rad\n",
                p.peakForce / 1e3, mLo / 0.8 / 1e3, mHi / 0.3 / 1e3, p.tilt / kDeg, p.plastic);
    CHECK_RANGE(p.peakForce, mLo / 0.8, mHi / 0.3);
    CHECK(p.plastic > 0.2);
    CHECK(p.tilt > 15 * kDeg);
    CHECK(p.events[static_cast<int>(EventType::ObjectHit)] == 1);
    CHECK(p.events[static_cast<int>(EventType::PostBent)] == 1);
}

TEST_CASE("post: a car-sized box at 50 km/h folds a stop sign down and goes on") {
    const Push p = push("stop_sign_r1", 50 / 3.6, 2.0);
    std::printf("    50 km/h: peak %.1f kN, tilt %.0f deg, state %d, panel off %d\n", p.peakForce / 1e3, p.tilt / kDeg, p.state,
                p.panelOff);
    CHECK(p.tilt > 45 * kDeg);
    CHECK(p.state >= static_cast<int>(ObjectState::Bent));
    CHECK(p.events[static_cast<int>(EventType::PostDown)] == 1);
}

TEST_CASE("post: a cast-iron bollard snaps; a concrete-filled steel bollard holds a slow push") {
    const Push iron = push("bollard_cast_iron", 30 / 3.6, 1.0, 0.2);
    std::printf("    cast iron at 30 km/h: peak %.0f kN, state %d, events broken %d\n", iron.peakForce / 1e3, iron.state,
                iron.events[static_cast<int>(EventType::PostBroken)]);
    CHECK(iron.state == static_cast<int>(ObjectState::Broken));
    const Push steel = push("bollard_steel", 0.3, 1.0, 0.2);
    std::printf("    steel bollard, slow: peak %.0f kN, plastic %.3f rad\n", steel.peakForce / 1e3, steel.plastic);
    CHECK(steel.state != static_cast<int>(ObjectState::Broken));
}

TEST_CASE("post: a slip base lets go and leaves a stub under 102 mm (MASH)") {
    const Push p = push("sign_slip_base", 40 / 3.6, 1.5);
    std::printf("    slip base at 40 km/h: peak %.1f kN, state %d, stub %.3f m\n", p.peakForce / 1e3, p.state, p.stub);
    CHECK(p.events[static_cast<int>(EventType::BaseReleased)] == 1);
    CHECK(p.stub <= 0.102);
}

TEST_CASE("post: deterministic") {
    auto run = [] {
        World w(1);
        w.loadLibrary(libraryJson());
        w.addObject("street_name_double", 0, 0, 180);
        HostBody hb;
        hb.id = 1;
        hb.half = {0.9, 0.9, 0.5};
        hb.velocity = {0.3, 15, 0};
        hb.centre = {0, -2, 0.75};
        for (int i = 0; i < 90; ++i) {
            w.setBodies(&hb, 1);
            w.step(1.0 / 60);
            hb.centre += hb.velocity * (1.0 / 60);
        }
        return w.stateHash();
    };
    const auto a = run(), b = run();
    CHECK(a == b);
}
