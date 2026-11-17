// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Everything about a pole except its position, heading and class is a pure
// function of its position (the world data only stores [x, y, hdg, cls]):
//
//   seed(x, y) = ((int(round(x*10)) * 73856093) ^ (int(round(y*10)) * 19349663)) & 0x7fffffff
//   transformer    = s % 100 < 14
//   light          = (s // 100) % 100 < {55, 70, 85, 95}[cls]
//   telecom_cables = 4 + cls*2 + (s // 10000) % 5
//   service_drops  = (s // 100000) % 4
//
// as the world builder computes it in Python: round() rounds halves to even,
// and the products wrap like Python's unbounded integers masked to 31 bits
// (two's-complement multiplication modulo 2^64 gives the same low bits). The
// pole's ID is its seed.
//
// Frames: the pole's heading is a compass bearing (clockwise from north) of
// its line direction, which is also its flange (strong-axis) direction. The
// street side is to the right of the heading. Body axes: x along the line,
// y to the left, z up.
#pragma once

#include <cstdint>

#include "pd/math.hpp"
#include "pd/params.hpp"

namespace pd {

// Python's round() for a double (half to even), as an integer.
std::int64_t pythonRound(Real v);

// The pole ID / seed for a position (world metres from MASP).
std::int32_t poleSeed(Real x, Real y);

struct PoleSpec {
    std::int32_t id = 0;
    Real x = 0, y = 0;
    Real headingDeg = 0;
    int cls = 0;
    bool transformer = false;
    bool light = false;
    int telecomCables = 0;
    int serviceDrops = 0;
};

PoleSpec makePole(Real x, Real y, Real headingDeg, int cls);

// Unit vectors of a pole's frame in the world (ENU).
Vec3 headingVector(Real headingDeg);   // along the line (body +x)
Vec3 streetVector(Real headingDeg);    // toward the street (body -y)

// Mass properties of the part of a pole between heights z0 and z1 (metres
// above ground; negative is buried), with the hardware attached in that
// range (the transformer only if `withTransformer`). Positions are along the
// pole's own axis, so a tilted piece uses the same numbers.
struct MassProps {
    Real mass = 0;
    Real comHeight = 0;   // along the axis
    Vec3 comOffset;       // body-frame offset of the centre of mass from the axis (x, y; z = 0)
    Real inertiaPerp = 0; // about the centre of mass, perpendicular to the axis (kg m^2)
    Real inertiaAxial = 0;
};
MassProps massProperties(const PoleGeometry& g, const PoleSpec& s, Real z0, Real z1, bool withTransformer);

// Section half-width at height z (for collision radius), and the strong/weak
// nominal moments at the ground line for the pole's class.
Real sectionRadius(const PoleGeometry& g, Real z);

}  // namespace pd
