// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
//
// Joint ranges: four quarter-ellipses of swing about the neutral pose, and a twist range.
#pragma once

#include <algorithm>
#include <cmath>

#include "ped/math.hpp"

namespace ped {

// The closest point on the ellipse x^2/a^2 + y^2/b^2 = 1 to (x, y) outside it (Eberly,
// "Distance from a point to an ellipse"): (a^2 x / (t + a^2), b^2 y / (t + b^2)) for the root t of
// F(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1, which is decreasing, positive at 0 and
// negative at a|x| + b|y|. A fixed number of bisections keeps it deterministic. A radial pull to
// the ellipse would be wrong here: on a narrow one (an elbow, 145 deg of flexion and 3 of
// sideways play) a little sideways error would drag the flexion half way back.
inline void closestOnEllipse(Real a, Real b, Real& x, Real& y) {
    const Real ax = a * std::abs(x), by = b * std::abs(y), a2 = a * a, b2 = b * b;
    Real lo = 0, hi = ax + by;
    for (int it = 0; it < 48; ++it) {
        const Real t = 0.5 * (lo + hi);
        const Real f = sq(ax / (t + a2)) + sq(by / (t + b2)) - 1.0;
        if (f > 0) lo = t;
        else hi = t;
    }
    const Real t = 0.5 * (lo + hi);
    x = a2 * x / (t + a2);
    y = b2 * y / (t + b2);
}

// Clamps (flex, abd) into the quarter-ellipse cone and twist into its range; true if it moved.
inline bool clampToRange(Real& flex, Real& abd, Real& twist, Real flexMin, Real flexMax, Real abdMin, Real abdMax,
                         Real twistMin, Real twistMax) {
    constexpr Real kMinHalf = 0.5 * kDeg;
    const Real ha = std::max(flex >= 0 ? flexMax : -flexMin, kMinHalf);
    const Real hb = std::max(abd >= 0 ? abdMax : -abdMin, kMinHalf);
    bool moved = false;
    if (sq(flex / ha) + sq(abd / hb) > 1) {
        closestOnEllipse(ha, hb, flex, abd);
        moved = true;
    }
    if (twist < twistMin) twist = twistMin, moved = true;
    else if (twist > twistMax) twist = twistMax, moved = true;
    return moved;
}

}  // namespace ped
