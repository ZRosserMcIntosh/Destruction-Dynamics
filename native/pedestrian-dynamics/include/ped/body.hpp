// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
//
// The body model is data (data/pedestrian_body.json): capsule segments in the standing neutral
// pose, given as fractions of stature, with de Leva mass fractions and the anatomical ranges of
// the joints between them. build() scales it to a person (stature, mass): positions, radii,
// masses, inertias, and the gains, which scale with mass x stature.
//
// Every segment's orientation is the identity in the neutral pose, so a joint's relative rotation
// (child relative to parent, in the parent's frame) is the identity when standing. Its three
// anatomical axes, in the parent's frame:
//   twist  along the child bone (rotation about it: internal/external rotation, pronation),
//   flex   flexion (+) / extension (-),
//   abd    abduction (+) / adduction (-), or lateral bend.
// The limit is a cone made of four quarter-ellipses around the neutral pose (each direction its
// own range, so hip flexion 120 and extension 20 are both exact), plus a twist range.
#pragma once

#include <array>
#include <string>
#include <string_view>
#include <vector>

#include "ped/math.hpp"

namespace ped {

inline constexpr int kMaxSegments = 20;

struct JointDef {
    std::string name;
    Vec3 at;                         // fraction of stature, neutral pose
    Vec3 twistAxis{0, 0, -1}, flexAxis{0, -1, 0}, abdAxis{1, 0, 0};
    Real flexMin = 0, flexMax = 0, abdMin = 0, abdMax = 0, twistMin = 0, twistMax = 0;  // rad
    Real kp = 0, torque = 0;         // N m / rad and N m (reference person)
    Real limitStiffness = 0;         // N m / rad past the range
    Real tearTorque = 0;             // N m past the range that tears the ligaments
};

struct SegmentDef {
    std::string name;
    int parent = -1;
    Real massPct = 0;
    Vec3 from, to;                   // capsule axis end centres, fraction of stature
    Real radius = 0;                 // fraction of stature
    Real tissueStiffness = 1e6;      // N/m
    JointDef joint;                  // (no joint for the root)
};

class BodyDef {
public:
    // Loads the body model; returns "" or an error.
    std::string load(std::string_view json);
    int size() const { return static_cast<int>(segments_.size()); }
    const SegmentDef& at(int i) const { return segments_[static_cast<std::size_t>(i)]; }
    int index(std::string_view name) const;
    Real refStature = 1.75, refMass = 75;

private:
    std::vector<SegmentDef> segments_;
};

// One person's body, scaled.
struct BuiltSegment {
    Vec3 a, b;                // capsule axis end centres (m), neutral pose, relative to the feet
    Vec3 centre;              // centre of mass (m)
    Real radius = 0, halfLength = 0, mass = 0;
    Vec3 inertia;             // principal moments about the centre (the capsules lie along the
                              // body axes in the neutral pose, so these are the body-frame axes)
    Real tissueStiffness = 0;
    int parent = -1;
    // joint to the parent
    Vec3 jointAt;             // world, neutral pose (m)
    Real kp = 0, torque = 0, limitStiffness = 0, tearTorque = 0;  // scaled
};

struct BuiltBody {
    int count = 0;
    std::array<BuiltSegment, kMaxSegments> seg{};
    Real mass = 0, stature = 0;
    Vec3 com;                 // standing
};

BuiltBody buildBody(const BodyDef& def, Real stature, Real mass);

}  // namespace ped
