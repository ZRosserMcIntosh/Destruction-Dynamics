// Pedestrian Dynamics — cars striking people, against what real collisions show.
// SPDX-License-Identifier: MIT
//
// The car here is a simple rigid body (mass, yaw inertia, brakes) standing in for a host's
// vehicle model: each step it tells the world where it is, and takes back the impulse the person
// put on it. Validation targets (see docs/README.md for sources and how sure each is):
//   throw distance between Searle's bounds for a launch at the car's speed, mu 0.66;
//   a medium sedan at 40 km/h wraps the person and the head lands on the bonnet, cowl or
//   windscreen ~100-150 ms after the bumper touches, at a speed near the car's;
//   a high-fronted SUV or a flat-fronted van projects the person forward;
//   the car loses about m_person / (m_car + m_person) of its speed;
//   momentum balances exactly between car and person.
#include <cmath>
#include <cstdio>
#include <string>

#include "ped/rot.hpp"
#include "ped/world.hpp"
#include "ped_test.hpp"

using namespace ped;

std::string bodyJson();
std::string vehiclesJson();

namespace {

constexpr Real kMu = 0.66;

struct TestCar {
    int profile = 0;
    Real mass = 1300, izz = 2000;
    Vec3 pos, vel;
    Real yaw = 0, yawRate = 0;
    Real brakeG = 0;  // deceleration once braking (g)
    bool braking = false;

    VehicleState state(std::int32_t id) const {
        VehicleState s;
        s.id = id;
        s.profile = profile;
        s.position = pos;
        s.orientation = axisAngle({0, 0, 1}, yaw);
        s.velocity = vel;
        s.angularVelocity = {0, 0, yawRate};
        return s;
    }
    void take(const Vec3& J, const Vec3& L) {
        // the car is on its wheels: it takes the horizontal impulse and the yaw moment
        vel.x += J.x / mass;
        vel.y += J.y / mass;
        yawRate += L.z / izz;
    }
    void advance(Real dt) {
        const Real sp = std::hypot(vel.x, vel.y);
        if (braking && sp > 0) {
            const Real dv = std::min(sp, brakeG * kGravity * dt);
            vel.x -= vel.x / sp * dv;
            vel.y -= vel.y / sp * dv;
        }
        pos += vel * dt;
        yaw += yawRate * dt;
        yawRate *= std::max(0.0, 1.0 - 3.0 * dt);  // tyres resist yaw
    }
};

struct Crash {
    Report r;
    Real carDv = 0;       // km/h lost to the person (braking excluded)
    Real seconds = 0;
    Real carSpeedAtHead = 0;
    std::uint64_t hash = 0;
    int sleeps = 0;
};

Crash crash(const char* profile, Real kmh, Real brakeG = 0.7, Real lateral = 0.0, Real heading = 0.5 * kPi,
            Real stature = 1.75, Real mass = 75) {
    World w(1);
    w.loadBody(bodyJson());
    w.loadVehicles(vehiclesJson());
    const int i = w.addPedestrian(0, lateral, heading, stature, mass);
    w.setAssist(i, 1);  // standing until struck
    TestCar car;
    car.profile = w.vehicles.index(profile);
    car.mass = w.vehicles.at(car.profile).mass;
    car.izz = car.mass * 1.6;  // ~ m (L^2 + W^2) / 12 for a 4.5 x 1.8 m car
    const Real front = w.vehicles.at(car.profile).front;
    const Real v0 = kmh / 3.6;
    car.pos = {-front - 0.25, 0, 0};
    car.vel = {v0, 0, 0};
    car.brakeG = brakeG;
    const Real dt = 1.0 / 60;
    Crash c;
    Real dvFromPerson = 0;
    for (int k = 0; k < 60 * 15; ++k) {
        const VehicleState vs = car.state(7);
        w.setVehicles(&vs, 1);
        w.step(dt);
        Vec3 J, L;
        w.vehicleImpulse(0, J, L);
        const Real before = car.vel.x;
        car.take(J, L);
        dvFromPerson += before - car.vel.x;
        const Report& r = w.report(i);
        if (r.struck) car.braking = true;
        if (r.headRegion >= 0 && c.carSpeedAtHead == 0) c.carSpeedAtHead = std::hypot(car.vel.x, car.vel.y);
        car.advance(dt);
        c.seconds = (k + 1) * dt;
        if (r.atRest) break;
    }
    c.r = w.report(i);
    c.carDv = dvFromPerson * 3.6;
    c.hash = w.stateHash();
    return c;
}

Real searleMin(Real v) { return v * v / (2 * kMu * kGravity); }
Real searleMax(Real v) { return v * v * (1 + kMu * kMu) / (2 * kMu * kGravity); }

void print(const char* label, Real kmh, const Crash& c) {
    const Report& r = c.r;
    const Real v = kmh / 3.6;
    std::printf("    %-15s throw %5.1f m (Searle %4.1f-%4.1f)  %-18s head %-10s %4.1f m/s at %3.0f ms  HIC car %5.0f ground %5.0f "
                "(peak %3.0f g, at %4.0f-%4.0f ms)  tibia %3.0f g  knee %4.1f deg  car -%.1f km/h  rest %.1f s\n",
                label, r.throwDistance, searleMin(v), searleMax(v), trajectoryName(r.trajectory),
                r.headRegion >= 0 ? regionName(static_cast<Region>(r.headRegion)) : "-", r.headImpactSpeed, r.headImpactTime * 1000,
                r.hicVehicle, r.hicGround, r.headPeakG, (r.hicT1 - r.firstContactTime) * 1000, (r.hicT2 - r.firstContactTime) * 1000,
                std::max(r.tibiaPeakG[0], r.tibiaPeakG[1]), std::max(r.kneeBendPeakDeg[0], r.kneeBendPeakDeg[1]), c.carDv, c.seconds);
}

}  // namespace

TEST_CASE("impact: a sedan at 20-60 km/h: wrap, head on the bonnet or glass, throw within Searle's bounds") {
    for (Real kmh : {20.0, 30.0, 40.0, 50.0, 60.0}) {
        const Crash c = crash("sedan", kmh);
        char label[64];
        std::snprintf(label, sizeof label, "sedan %2.0f km/h", kmh);
        print(label, kmh, c);
        const Real v = kmh / 3.6;
        CHECK(c.r.struck);
        CHECK(c.r.atRest);
        CHECK(c.r.throwDistance > 0.6 * searleMin(v));
        CHECK(c.r.throwDistance < 1.3 * searleMax(v));
        // the car loses about m / (M + m) of its speed to the person (they leave it at about its speed)
        const Real share = kmh * 75.0 / (1300.0 + 75.0);
        CHECK(c.carDv > 0.5 * share && c.carDv < 1.6 * share);
        if (kmh >= 30) {
            CHECK(c.r.trajectory == Trajectory::Wrap || c.r.trajectory == Trajectory::RoofVault);
            CHECK(c.r.headOnUpperSurface);
        }
        if (kmh == 40.0) {
            CHECK(c.r.headRegion == static_cast<int>(Region::Bonnet) || c.r.headRegion == static_cast<int>(Region::Cowl) ||
                  c.r.headRegion == static_cast<int>(Region::Windscreen) || c.r.headRegion == static_cast<int>(Region::APillar));
            CHECK(c.r.headImpactTime > 0.08 && c.r.headImpactTime < 0.2);
            CHECK(c.r.headImpactSpeed > 0.6 * v && c.r.headImpactSpeed < 1.5 * v);
        }
    }
}

TEST_CASE("impact: high and flat fronts project the person forward") {
    for (const char* prof : {"suv", "van"}) {
        const Crash c = crash(prof, 40);
        char label[64];
        std::snprintf(label, sizeof label, "%s 40 km/h", prof);
        print(label, 40, c);
        CHECK(c.r.struck && c.r.atRest);
        CHECK(c.r.trajectory == Trajectory::ForwardProjection);
        CHECK(c.r.throwDistance > 0.6 * searleMin(40 / 3.6) && c.r.throwDistance < 1.3 * searleMax(40 / 3.6));
    }
    const Crash h = crash("hatch", 40);
    print("hatch 40 km/h", 40, h);
    CHECK(h.r.trajectory == Trajectory::Wrap || h.r.trajectory == Trajectory::RoofVault);
}

TEST_CASE("impact: momentum balances exactly between car and person") {
    // the person floats (ground far below) so only the car and gravity act on them
    World w(1);
    w.loadBody(bodyJson());
    w.loadVehicles(vehiclesJson());
    const int i = w.addPedestrian(0, 0, 0.5 * kPi);
    w.setGroundHeight(i, -100);
    TestCar car;
    car.profile = w.vehicles.index("sedan");
    car.pos = {-2.55, 0, 0};
    car.vel = {11, 0, 0};
    const Real dt = 1.0 / 60;
    const Real m = 75;
    const Vec3 p0 = w.momentum(i) + Vec3{car.mass * car.vel.x, car.mass * car.vel.y, 0};
    for (int k = 0; k < 30; ++k) {
        const VehicleState vs = car.state(1);
        w.setVehicles(&vs, 1);
        w.step(dt);
        Vec3 J, L;
        w.vehicleImpulse(0, J, L);
        car.vel += Vec3{J.x, J.y, 0} / car.mass;
        car.vel.z += 0;
        car.advance(dt);
        // vertical: the car's suspension takes it; count it as the car's
        car.pos.z = 0;
        if (k == 29) {
            const Vec3 pp = w.momentum(i);
            const Real horizontal = std::hypot(pp.x + car.mass * car.vel.x - p0.x, pp.y + car.mass * car.vel.y - p0.y);
            std::printf("    after 0.5 s: person %.1f N s forward, car lost %.1f N s; imbalance %.2e N s\n", pp.x,
                        p0.x - car.mass * car.vel.x, horizontal);
            CHECK(horizontal < 1e-6 * car.mass);
            CHECK(pp.x > 0.5 * m * 11);  // and the person did get pushed
        }
    }
}

TEST_CASE("impact: the same crash twice gives the same bits") {
    const Crash a = crash("sedan", 45), b = crash("sedan", 45);
    CHECK(a.hash == b.hash);
    CHECK(a.r.hic15 == b.r.hic15);
}
