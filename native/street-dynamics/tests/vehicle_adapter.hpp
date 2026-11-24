// Street Dynamics — the coupling with OpenRoad Dynamics' vehicle (tests; hosts copy it).
// SPDX-License-Identifier: MIT
//
// Each step:
//  1. capture the car's motion (StreetObstacles::capture);
//  2. the vehicle steps: its crash structure meets the objects as obstacles (StreetObstacles, an
//     ord::ObstacleQuery) that push back with at most what they can take (yieldForce), so a
//     sign post gives way and a bollard does not, and the car's crush is its own;
//  3. every ObstacleImpact on an object goes to sd::World::applyImpact: the objects take
//     exactly the impulse the car felt, so momentum balances by construction;
//  4. sd::World::step.
// Hosts without a crash model (traffic cars) pass boxes instead (carBoxes, setBodies).
#pragma once

#include "ord/damage.hpp"
#include "ord/vehicle.hpp"
#include "sd/world.hpp"

namespace sdtest {

class StreetObstacles final : public ord::ObstacleQuery {
public:
    explicit StreetObstacles(sd::World& w) : w_(w) {}
    // The car's motion at the start of the step (for the yield forces' inertia part).
    void capture(const ord::Vehicle& v, ord::Real dt);
    bool rigid = false;  // ignore the yield forces (everything rigid): for comparison
    bool anyNear(const ord::Vec3& c, ord::Real r) const override;
    ord::ObstacleContact point(const ord::Vec3& p, const ord::Vec3& ref, ord::Real margin) const override;
    ord::ObstacleContact edge(const ord::Vec3& a, const ord::Vec3& b, ord::Real margin, ord::Real& t) const override;

private:
    sd::Vec3 velocityAt(const ord::Vec3& p) const;
    sd::World& w_;
    ord::Vec3 com_, vel_, omega_;
    ord::Real dt_ = 1.0 / 360;
};

// The car as two boxes (the body up to the bonnet line, and the cabin above it) from its crash
// structure as built; ids `id` and `id + 1`. Returns the count written.
int carBoxes(const ord::Vehicle& v, std::int32_t id, sd::HostBody* out);

// An impulse on the car at a world point (a host's kick: debris, a blast).
void applyImpulse(ord::Vehicle& v, const ord::Vec3& point, const ord::Vec3& impulse);

}  // namespace sdtest
