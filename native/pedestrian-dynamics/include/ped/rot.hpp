// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
//
// Rotation helpers built on ped/dmath.hpp (deterministic transcendentals).
#pragma once

#include "ped/dmath.hpp"
#include "ped/math.hpp"

namespace ped {

inline Quat axisAngle(const Vec3& axis, Real angle) {
    Real s, c;
    dm::sincos(0.5 * angle, s, c);
    return {c, axis.x * s, axis.y * s, axis.z * s};
}

// The rotation vector of a unit quaternion (shortest way round).
inline Vec3 rotationVector(Quat d) {
    if (d.w < 0) d = {-d.w, -d.x, -d.y, -d.z};
    const Real s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (s < 1e-12) return {2 * d.x, 2 * d.y, 2 * d.z};
    const Real angle = 2.0 * dm::atan2(s, d.w);
    return Vec3{d.x, d.y, d.z} * (angle / s);
}

// The unit quaternion of a rotation vector.
inline Quat fromRotationVector(const Vec3& v) {
    const Real a = length(v);
    if (a < 1e-12) return normalized(Quat{1, 0.5 * v.x, 0.5 * v.y, 0.5 * v.z});
    return axisAngle(v / a, a);
}

// Adds a small rotation (world vector) to q.
inline Quat addRotation(const Quat& q, const Vec3& v) {
    const Quat dq = Quat{0, v.x, v.y, v.z} * q;
    return normalized(Quat{q.w + 0.5 * dq.w, q.x + 0.5 * dq.x, q.y + 0.5 * dq.y, q.z + 0.5 * dq.z});
}

// Swing-twist decomposition q = swing * twist, twist about the unit axis t (in the frame q maps
// from). The swing's axis is perpendicular to t. Returns the twist angle in (-pi, pi].
inline Real swingTwist(const Quat& q0, const Vec3& t, Quat& swing) {
    Quat q = q0;
    if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
    const Real p = q.x * t.x + q.y * t.y + q.z * t.z;
    const Real n = std::sqrt(q.w * q.w + p * p);
    Quat twist = n > 1e-12 ? Quat{q.w / n, t.x * p / n, t.y * p / n, t.z * p / n} : Quat{};
    swing = q * conjugate(twist);
    return 2.0 * dm::atan2(p, q.w);
}

}  // namespace ped
