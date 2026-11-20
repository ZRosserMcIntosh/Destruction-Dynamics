// Street Dynamics — the library: section properties, hinge laws, the data.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <fstream>
#include <sstream>

#include "sd/library.hpp"
#include "sd/world.hpp"
#include "sd_test.hpp"

using namespace sd;

namespace {
std::string readFile(const char* path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
}  // namespace

std::string libraryJson() { return readFile(SD_DATA_DIR "/street_library.json"); }

TEST_CASE("library: a 2 1/2 in sign post's section, by hand") {
    Material steel;
    steel.yield = 250e6;
    steel.youngs = 205e9;
    Section s;
    s.kind = SectionKind::RoundTube;
    s.d = 0.0635;
    s.t = 0.00265;
    const PostMechanics m = deriveMechanics(s, steel);
    const Real di = 0.0635 - 2 * 0.00265;
    CHECK_NEAR(m.inertia, kPi / 64 * (std::pow(0.0635, 4) - std::pow(di, 4)), 1e-12);
    CHECK_NEAR(m.plasticModulus, (std::pow(0.0635, 3) - std::pow(di, 3)) / 6, 1e-12);
    CHECK_NEAR(m.massPerMetre, 7850 * kPi / 4 * (0.0635 * 0.0635 - di * di), 1e-9);
    // D/t = 24: a stocky (class 1) wall: hardens past M_p, buckles late
    CHECK(m.crossSectionClass == 1);
    CHECK_NEAR(m.plasticMoment, 250e6 * m.plasticModulus, 1e-6);
    CHECK_RANGE(m.plasticMoment, 2.3e3, 2.6e3);   // ~2.45 kN m
    CHECK(m.peakMoment > m.plasticMoment);
    CHECK(m.capacity(0) == m.yieldMoment);
    CHECK_NEAR(m.capacity(m.buckleRotation), m.peakMoment, 1e-9);
    CHECK(m.capacity(5.0) == m.residualMoment);
    std::printf("    2.5 in x 2.65 mm: I %.3g m^4, Mp %.2f kN m, %.2f kg/m\n", m.inertia, m.plasticMoment / 1e3, m.massPerMetre);
}

TEST_CASE("library: slender walls buckle early; brittle sections crack") {
    Material steel;
    Section thin;
    thin.d = 0.1;
    thin.t = 0.001;  // D/t = 100: class 4
    const PostMechanics m = deriveMechanics(thin, steel);
    CHECK(m.crossSectionClass == 4);
    CHECK(m.peakMoment < m.yieldMoment * 1.0001);
    Material iron;
    iron.brittle = true;
    iron.yield = 250e6;
    iron.residual = 0;
    Section bar;
    bar.kind = SectionKind::SolidRound;
    bar.d = 0.09;
    const PostMechanics b = deriveMechanics(bar, iron);
    CHECK_NEAR(b.peakMoment, 250e6 * kPi * 0.09 * 0.09 * 0.09 / 32, 1e-3);  // f_r x W
    CHECK(b.residualMoment == 0);
    CHECK(b.fractureRotation < 0.02);
}

TEST_CASE("library: the data loads, every type is physical") {
    Library lib;
    const std::string err = lib.load(libraryJson());
    CHECK(err.empty());
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(lib.size() >= 9);
    for (int i = 0; i < lib.size(); ++i) {
        const Archetype& a = lib.at(i);
        std::printf("    %-20s %5.2f m, %5.1f kg, Mp %6.2f kN m, class %d\n", a.id.c_str(), a.height, a.totalMass,
                    a.mech.plasticMoment / 1e3, a.mech.crossSectionClass);
        CHECK(a.height > 0.3);
        CHECK(a.totalMass > 1);
        CHECK(a.mech.plasticMoment > 100);
        CHECK(a.segments >= 2);
    }
    const Archetype* stop = lib.find("stop_sign_r1");
    CHECK(stop != nullptr);
    if (stop) {
        CHECK_NEAR(panelArea(stop->panels[0]), 2 * (std::sqrt(2.0) - 1) * 0.604 * 0.604, 1e-9);
        CHECK_RANGE(stop->panels[0].centre - 0.302, 2.09, 2.31);  // bottom edge 2.10-2.30 m (MBST, CET-SP)
    }
}

TEST_CASE("library: the world builder's seed gives the object IDs") {
    CHECK(objectSeed(0, 0) == 0);
    CHECK(objectSeed(12.5, -3.25) == 2102574409);
    CHECK(objectSeed(-1234.56, 789.01) == 958305920);
}
