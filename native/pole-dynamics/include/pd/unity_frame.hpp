// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// The sim <-> Unity frame conversion, for C++ hosts and as the reference the
// C# wrapper (unity/PoleDynamics.cs) mirrors.
//
// Sim: ENU, right-handed, x east, y north, z up (metres).
// Unity: left-handed, x east, y up, z north.
// A point or vector maps (x, y, z) -> (x, z, y): swapping two axes flips the
// handedness, which is exactly the difference between the two frames. A
// rotation by angle a about axis n becomes a rotation by -a about the swapped
// axis, so q = (w, x, y, z) -> (w, -x, -z, -y).
#pragma once

#include "pd/math.hpp"

namespace pd::unity {

inline Vec3 toUnity(const Vec3& v) { return {v.x, v.z, v.y}; }
inline Vec3 fromUnity(const Vec3& v) { return {v.x, v.z, v.y}; }
inline Quat toUnity(const Quat& q) { return {q.w, -q.x, -q.z, -q.y}; }
inline Quat fromUnity(const Quat& q) { return {q.w, -q.x, -q.z, -q.y}; }

}  // namespace pd::unity
