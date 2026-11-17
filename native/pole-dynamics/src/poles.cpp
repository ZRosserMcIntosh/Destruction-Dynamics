// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
#include "pd/poles.hpp"

#include <cmath>

#include "pd/dmath.hpp"

namespace pd {

std::int64_t pythonRound(Real v) {
    // Round half to even on the exact binary value, as Python's round() does.
    const Real f = std::floor(v);
    const Real d = v - f;  // exact for |v| < 2^52
    Real r;
    if (d > 0.5)
        r = f + 1.0;
    else if (d < 0.5)
        r = f;
    else
        r = (std::fmod(f, 2.0) == 0.0) ? f : f + 1.0;
    return static_cast<std::int64_t>(r);
}

std::int32_t poleSeed(Real x, Real y) {
    const auto a = static_cast<std::uint64_t>(pythonRound(x * 10.0));
    const auto b = static_cast<std::uint64_t>(pythonRound(y * 10.0));
    const std::uint64_t h = (a * 73856093ull) ^ (b * 19349663ull);
    return static_cast<std::int32_t>(h & 0x7fffffffull);
}

PoleSpec makePole(Real x, Real y, Real headingDeg, int cls) {
    static constexpr int kLightThreshold[kPoleClasses] = {55, 70, 85, 95};
    PoleSpec p;
    p.x = x;
    p.y = y;
    p.headingDeg = headingDeg;
    p.cls = cls < 0 ? 0 : (cls >= kPoleClasses ? kPoleClasses - 1 : cls);
    p.id = poleSeed(x, y);
    const std::int32_t s = p.id;
    p.transformer = s % 100 < 14;
    p.light = (s / 100) % 100 < kLightThreshold[p.cls];
    p.telecomCables = 4 + p.cls * 2 + (s / 10000) % 5;
    p.serviceDrops = (s / 100000) % 4;
    return p;
}

Vec3 headingVector(Real headingDeg) {
    Real s, c;
    dm::sincos(headingDeg * kDeg, s, c);
    return {s, c, 0};  // compass bearing: x east = sin, y north = cos
}

Vec3 streetVector(Real headingDeg) {
    const Vec3 f = headingVector(headingDeg);
    return {f.y, -f.x, 0};  // right of the heading
}

Real sectionRadius(const PoleGeometry& g, Real z) {
    const Real t = clamp(z / g.lengthAbove, 0.0, 1.0);
    const Real k = 1.0 + (g.topScale - 1.0) * t;
    return 0.5 * std::sqrt(sq(g.baseWidth * k) + sq(g.baseDepth * k)) * 0.8;
}

MassProps massProperties(const PoleGeometry& g, const PoleSpec& s, Real z0, Real z1, bool withTransformer) {
    // Concrete in slices along the axis (dimensions taper linearly from the
    // ground line to the top; the buried part keeps the base section).
    constexpr int kSlices = 48;
    Real m = 0, mz = 0, mzz = 0, my = 0, myy = 0, axial = 0;
    const Real dz = (z1 - z0) / kSlices;
    for (int i = 0; i < kSlices; ++i) {
        const Real z = z0 + (i + 0.5) * dz;
        const Real t = clamp(z / g.lengthAbove, 0.0, 1.0);
        const Real k = 1.0 + (g.topScale - 1.0) * t;
        const Real w = g.baseWidth * k, d = g.baseDepth * k;
        const Real dm_ = g.density * g.fill * w * d * dz;
        m += dm_;
        mz += dm_ * z;
        mzz += dm_ * z * z;
        axial += dm_ * (w * w + d * d) / 12.0;
    }
    // Hardware as point masses (z along the axis, y lateral in the body frame).
    auto add = [&](Real mass, Real z, Real yLat) {
        if (z < z0 || z > z1) return;
        m += mass;
        mz += mass * z;
        mzz += mass * z * z;
        my += mass * yLat;
        myy += mass * yLat * yLat;
    };
    // Body y is to the left, so a street-side offset o sits at y = -o. The
    // crossarm's mean conductor offset is -0.55 m (away from the street).
    add(g.crossarmMass, g.mvCrossarm, 0.55);
    add(g.lvRackMass, 0.5 * (g.lvHeight[0] + g.lvHeight[3]), -g.lvOffset);
    if (s.light) add(g.lightMass, 0.5 * (g.lightLow + g.lightHigh), -1.0);
    if (s.transformer && withTransformer) add(g.transformerMass, g.transformerHeight, -g.transformerOffset);
    MassProps r;
    r.mass = m;
    if (m <= 0) return r;
    r.comHeight = mz / m;
    r.comOffset = {0, my / m, 0};
    // Thin-body inertia about the centre of mass, perpendicular to the axis.
    r.inertiaPerp = (mzz - m * sq(r.comHeight)) + (myy - m * sq(r.comOffset.y)) + axial;
    r.inertiaAxial = axial + (myy - m * sq(r.comOffset.y)) + 1.0;
    return r;
}

}  // namespace pd
