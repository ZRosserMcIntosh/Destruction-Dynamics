// Pedestrian Dynamics — joints, muscles, standing, falling, momentum.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <string>

#include "ped/world.hpp"
#include "ped_test.hpp"

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

Real lowestPoint(const World& w, int i) {
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

TEST_CASE("motion: a person standing with full strength and the balance assist stays up") {
    World w = makeWorld();
    const int i = w.addPedestrian(0, 0, 0);
    w.setAssist(i, 1);
    const Real pelvis0 = w.segmentPosition(i, 0).z;
    for (int k = 0; k < 180; ++k) w.step(1.0 / 60);
    const Real pelvis = w.segmentPosition(i, 0).z;
    std::printf("    pelvis %.3f m -> %.3f m after 3 s; lowest point %.4f m\n", pelvis0, pelvis, lowestPoint(w, i));
    CHECK(pelvis > 0.97 * pelvis0);
    CHECK(lowestPoint(w, i) > -0.01);
}

TEST_CASE("motion: limp, a person crumples to the ground, rests on it, and sleeps") {
    World w = makeWorld();
    const int i = w.addPedestrian(0, 0, 0);
    w.setStrength(i, 0);
    Real lowest = 1;
    int k = 0;
    for (; k < 600 && w.awake(i); ++k) {
        w.step(1.0 / 60);
        lowest = std::min(lowest, lowestPoint(w, i));
    }
    const Real pelvis = w.segmentPosition(i, 0).z;
    std::printf("    pelvis %.3f m after %.1f s; deepest any part went into the ground %.4f m; asleep %d\n", pelvis, k / 60.0,
                -lowest, !w.awake(i));
    CHECK(pelvis < 0.35);
    CHECK(lowest > -0.03);
    CHECK(!w.awake(i));
}

TEST_CASE("motion: joints stop at their anatomical ranges") {
    // Drive each joint far past its range with the muscles at full strength (no gravity effect:
    // the person floats high above the ground for a moment) and read where it stops: within the
    // range plus what the muscle's torque cap strains the end-of-range stiffness.
    // (lowDeg: the hip folds until the thigh meets the belly, ~105 deg with hard capsules; a real
    // belly gives the last 15)
    struct Case { const char* seg; Real flex, abd, twist; int axis; Real limitDeg, lowDeg; };
    const Case cases[] = {
        {"thigh_l", 170, 0, 0, 0, 120, 100}, {"thigh_l", -90, 0, 0, 0, -20, -16}, {"thigh_l", 0, 90, 0, 1, 45, 38},
        {"shank_l", 179, 0, 0, 0, 140, 135},  {"shank_l", -60, 0, 0, 0, -3, -2},  {"forearm_r", 179, 0, 0, 0, 145, 140},
        {"head", 0, 0, 120, 2, 70, 66},      {"head", -110, 0, 0, 0, -55, -51},   {"foot_r", -100, 0, 0, 0, -50, -46},
    };
    for (const Case& c : cases) {
        World w = makeWorld();
        const int i = w.addPedestrian(0, 0, 0);
        w.setGroundHeight(i, -100);  // floating: the ground far below
        const int s = w.body.index(c.seg);
        w.setJointTarget(i, s, c.flex * kDeg, c.abd * kDeg, c.twist * kDeg);
        for (int k = 0; k < 30; ++k) w.step(1.0 / 60);
        const Vec3 a = w.jointAngles(i, s);
        const Real got = (c.axis == 0 ? a.x : c.axis == 1 ? a.y : a.z) / kDeg;
        std::printf("    %-9s driven to %6.0f: stops at %6.1f (range end %4.0f)\n", c.seg, c.axis == 0 ? c.flex : c.axis == 1 ? c.abd : c.twist,
                    got, c.limitDeg);
        CHECK(std::abs(got) <= std::abs(c.limitDeg) + 2.0);  // never past the range
        CHECK(std::abs(got) >= std::abs(c.lowDeg));          // and all the way to it
    }
}

TEST_CASE("motion: internal forces keep momentum; a blow adds exactly its impulse") {
    World w = makeWorld();
    const int i = w.addPedestrian(0, 0, 0.3);
    w.setGroundHeight(i, -100);  // in the air: only gravity acts from outside
    w.setStrength(i, 0.5);
    w.setJointTarget(i, w.body.index("thigh_l"), 60 * kDeg, 0, 0);
    w.setJointTarget(i, w.body.index("upperarm_r"), 90 * kDeg, 30 * kDeg, 0);
    const Real m = 75;
    const Real dt = 1.0 / 60;
    for (int k = 0; k < 20; ++k) w.step(dt);
    const Vec3 p0 = w.momentum(i);
    const Vec3 J{30, -10, 5};
    w.applyImpulse(i, w.body.index("thorax"), w.segmentPosition(i, w.body.index("thorax")) + Vec3{0, 0.1, 0.05}, J);
    for (int k = 0; k < 20; ++k) w.step(dt);
    const Vec3 p1 = w.momentum(i);
    const Vec3 expect = p0 + J + Vec3{0, 0, -kGravity * m * 20 * dt};
    std::printf("    momentum after a 30 N s blow and 1/3 s of flight: (%.4f %.4f %.4f) expected (%.4f %.4f %.4f)\n", p1.x, p1.y,
                p1.z, expect.x, expect.y, expect.z);
    CHECK(length(p1 - expect) < 1e-6 * m);
}
