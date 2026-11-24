// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
//
// Vehicle exteriors as a pedestrian meets them (data/vehicle_profiles.json): oriented boxes in
// the vehicle frame, each a region of the body (bumper, bonnet, windscreen...) with its own
// stiffness and damping. The host moves the vehicles; the people push back, and the impulses they
// put on each vehicle are handed back to the host, so momentum balances.
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "ped/math.hpp"

namespace ped {

enum class Region : int {
    Bumper = 0, Grille = 1, Bonnet = 2, Cowl = 3, Windscreen = 4, APillar = 5, RoofHeader = 6, Roof = 7, Body = 8,
    Ground = 9,  // (for events: a head striking the ground)
    Count = 10
};
const char* regionName(Region r);

struct VehiclePart {
    Region region = Region::Body;
    Vec3 centre, half;          // vehicle frame
    Quat orientation;           // vehicle frame (slabs are pitched)
    Real stiffness = 1e5, damping = 0.3, friction = 0.3;
};

struct VehicleProfile {
    std::string id, name;
    Real mass = 1300;
    Real front = 2.3, roofFront = 0, roofHeight = 1.45, halfWidth = 0.9, bleHeight = 0.73;
    std::vector<VehiclePart> parts;
    Vec3 boundCentre;           // vehicle frame
    Real boundRadius = 0;
};

class VehicleLibrary {
public:
    std::string load(std::string_view json);  // "" or an error
    int index(std::string_view id) const;
    const VehicleProfile& at(int i) const { return profiles_[static_cast<std::size_t>(i)]; }
    int size() const { return static_cast<int>(profiles_.size()); }

private:
    std::vector<VehicleProfile> profiles_;
};

// What the host tells the world about a vehicle each step.
struct VehicleState {
    std::int32_t id = 0;
    int profile = 0;            // index in the library
    Vec3 position;              // the profile's origin: on the ground under the middle of the car
    Quat orientation;
    Vec3 velocity, angularVelocity;  // world; the velocity is the origin's
};

}  // namespace ped
