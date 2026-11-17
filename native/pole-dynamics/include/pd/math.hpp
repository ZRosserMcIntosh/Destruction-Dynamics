// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Minimal linear algebra. Double precision throughout. The world frame is ENU
// (x east, y north, z up, metres from MASP), as in Ballistic Dynamics and the
// São Paulo World tiles; Unity's Y-up frame is converted at the C ABI only
// (pd/c_api.h).
//
// Only +, -, *, / and sqrt appear here: IEEE 754 rounds each of them exactly,
// so these are bit-identical on every platform (determinism, as in Ballistic
// Dynamics' ADR 0004). Transcendental functions live in pd/dmath.hpp.
#pragma once

#include <cmath>
#include <cstdint>

namespace pd {

using Real = double;

inline constexpr Real kPi = 3.14159265358979323846;
inline constexpr Real kTwoPi = 2.0 * kPi;
inline constexpr Real kGravity = 9.80665;  // m/s^2, standard gravity
inline constexpr Real kDeg = kPi / 180.0;

struct Vec3 {
    Real x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
    constexpr bool operator==(const Vec3&) const = default;
};

constexpr Vec3 operator*(Real s, const Vec3& v) { return v * s; }
constexpr Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) {
    const Real l = length(v);
    return l > 1e-300 ? v / l : Vec3{};
}
inline bool isFinite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
constexpr Real clamp(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }
constexpr Real sq(Real v) { return v * v; }
constexpr Real minr(Real a, Real b) { return a < b ? a : b; }
constexpr Real maxr(Real a, Real b) { return a > b ? a : b; }

// Unit quaternion (w, x, y, z), rotating body vectors into the world.
struct Quat {
    Real w = 1, x = 0, y = 0, z = 0;
};

constexpr Quat operator*(const Quat& a, const Quat& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
constexpr Quat conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat normalized(const Quat& q) {
    const Real n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return n > 0 ? Quat{q.w / n, q.x / n, q.y / n, q.z / n} : Quat{};
}
constexpr Vec3 rotate(const Quat& q, const Vec3& v) {
    // v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2.0;
    return v + t * q.w + cross(u, t);
}
constexpr Vec3 rotateInv(const Quat& q, const Vec3& v) { return rotate(conjugate(q), v); }
// First-order integration of an angular velocity (world frame) over dt, renormalised.
inline Quat integrate(const Quat& q, const Vec3& w, Real dt) {
    const Quat dq{0, w.x * 0.5 * dt, w.y * 0.5 * dt, w.z * 0.5 * dt};
    const Quat r = dq * q;
    return normalized(Quat{q.w + r.w, q.x + r.x, q.y + r.y, q.z + r.z});
}

// Bit pattern of a double, for determinism hashes.
std::uint64_t bitsOf(Real v);

// FNV-1a over the bit patterns of a sequence of doubles.
struct Hash64 {
    std::uint64_t h = 1469598103934665603ull;
    void add(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (v >> (8 * i)) & 0xffu;
            h *= 1099511628211ull;
        }
    }
    void add(Real v) { add(bitsOf(v)); }
    void add(const Vec3& v) { add(v.x); add(v.y); add(v.z); }
    void add(const Quat& q) { add(q.w); add(q.x); add(q.y); add(q.z); }
};

}  // namespace pd
