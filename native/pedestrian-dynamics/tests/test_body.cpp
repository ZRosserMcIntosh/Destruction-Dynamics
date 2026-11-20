// Pedestrian Dynamics — the body model: anthropometry, scaling, joint data.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "ped/body.hpp"
#include "ped/vehicle.hpp"
#include "ped_test.hpp"

using namespace ped;

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}
std::string bodyJson() { return readFile(PED_DATA_DIR "/pedestrian_body.json"); }
std::string vehiclesJson() { return readFile(PED_DATA_DIR "/vehicle_profiles.json"); }

TEST_CASE("body: the model loads; 16 segments; left and right mirror each other") {
    BodyDef d;
    CHECK(d.load(bodyJson()).empty());
    CHECK(d.size() == 16);
    const int tl = d.index("thigh_l"), tr = d.index("thigh_r");
    CHECK(tl > 0 && tr > 0);
    CHECK(d.at(tr).parent == d.index("pelvis"));
    CHECK(d.at(d.index("shank_r")).parent == tr);
    CHECK_NEAR(d.at(tr).from.y, -d.at(tl).from.y, 1e-12);
    // abduction moves each leg outward: the axes mirror as pseudovectors
    CHECK_NEAR(d.at(tr).joint.abdAxis.x, -d.at(tl).joint.abdAxis.x, 1e-12);
    CHECK_NEAR(d.at(tr).joint.flexAxis.y, d.at(tl).joint.flexAxis.y, 1e-12);
    CHECK_NEAR(d.at(tl).joint.flexMax / kDeg, 120, 1e-9);
    CHECK_NEAR(d.at(tl).joint.flexMin / kDeg, -20, 1e-9);
    CHECK(!BodyDef().load("{\"segments\":[]}").empty());
}

TEST_CASE("body: de Leva masses, Drillis-Contini proportions, centre of mass") {
    BodyDef d;
    d.load(bodyJson());
    const BuiltBody b = buildBody(d, 1.75, 75);
    Real m = 0, top = 0;
    for (int i = 0; i < b.count; ++i) {
        m += b.seg[static_cast<std::size_t>(i)].mass;
        top = std::max(top, std::max(b.seg[static_cast<std::size_t>(i)].a.z, b.seg[static_cast<std::size_t>(i)].b.z) +
                                b.seg[static_cast<std::size_t>(i)].radius);
    }
    CHECK_NEAR(m, 75, 1e-9);
    CHECK_NEAR(top, 1.75, 0.01);                                         // the top of the head
    CHECK_NEAR(b.seg[static_cast<std::size_t>(d.index("head"))].mass, 75 * 0.0694, 1e-9);
    CHECK_NEAR(b.seg[static_cast<std::size_t>(d.index("thigh_l"))].mass, 75 * 0.1416, 1e-9);
    CHECK_NEAR(b.com.z / 1.75, 0.55, 0.015);                             // standing COM ~ 55 % of stature
    CHECK_NEAR(b.com.y, 0, 1e-12);
    // the feet are on the ground, the knee where Drillis and Contini put it
    const BuiltSegment& foot = b.seg[static_cast<std::size_t>(d.index("foot_l"))];
    CHECK_NEAR(foot.a.z - foot.radius, 0, 1e-9);
    CHECK_NEAR(b.seg[static_cast<std::size_t>(d.index("shank_l"))].jointAt.z, 0.285 * 1.75, 1e-9);
    // a smaller person: masses scale, gains scale with mass x stature
    const BuiltBody c = buildBody(d, 1.20, 25);
    CHECK_NEAR(c.seg[static_cast<std::size_t>(d.index("head"))].mass, 25 * 0.0694, 1e-9);
    CHECK_NEAR(c.seg[static_cast<std::size_t>(d.index("thigh_l"))].kp / b.seg[static_cast<std::size_t>(d.index("thigh_l"))].kp,
               (25 * 1.20) / (75 * 1.75), 1e-12);
    std::printf("    75 kg, 1.75 m: COM %.3f m (%.1f %% of stature); head %.2f kg, thigh %.2f kg, trunk %.2f kg\n", b.com.z,
                100 * b.com.z / 1.75, b.seg[3].mass, b.seg[static_cast<std::size_t>(d.index("thigh_l"))].mass,
                b.seg[0].mass + b.seg[1].mass + b.seg[2].mass);
}

TEST_CASE("vehicles: the profiles load; regions and key heights") {
    VehicleLibrary lib;
    CHECK(lib.load(vehiclesJson()).empty());
    CHECK(lib.size() == 4);
    const VehicleProfile& sedan = lib.at(lib.index("sedan"));
    CHECK_NEAR(sedan.bleHeight, 0.73, 1e-9);
    bool windscreen = false;
    for (const auto& p : sedan.parts) windscreen = windscreen || p.region == Region::Windscreen;
    CHECK(windscreen);
    // a slab's top face lies on its line: the windscreen's centre top is ~1.2 m up
    for (const auto& p : sedan.parts)
        if (p.region == Region::Windscreen) {
            const Vec3 top = p.centre + rotate(p.orientation, Vec3{0, 0, p.half.z});
            CHECK_NEAR(top.z, 0.5 * (1.42 + 0.97), 1e-9);
            CHECK_NEAR(top.x, 0.5 * (0.15 + 1.02), 1e-9);
        }
    CHECK(lib.index("suv") >= 0 && lib.index("van") >= 0 && lib.index("hatch") >= 0);
    CHECK(!VehicleLibrary().load("{\"profiles\":{\"x\":{\"parts\":[{\"region\":\"nope\",\"box\":{}}]}}}").empty());
}
