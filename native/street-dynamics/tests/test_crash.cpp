// Street Dynamics — crash tests: the real vehicle model (OpenRoad Dynamics) into street objects,
// scored as EN 12767 and AASHTO MASH score their tests.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "reference_vehicles.hpp"
#include "sd_test.hpp"
#include "vehicle_adapter.hpp"

using namespace sd;

std::string libraryJson();

namespace {

// A rigid post (radius r at x, y): what the vehicle had before obstacles could give way.
struct RigidPost final : ord::ObstacleQuery {
    Real cx, cy, r;
    RigidPost(Real x, Real y, Real rad) : cx(x), cy(y), r(rad) {}
    bool anyNear(const ord::Vec3& c, Real rad) const override { return std::hypot(c.x - cx, c.y - cy) < rad + r; }
    ord::ObstacleContact point(const ord::Vec3& p, const ord::Vec3&, Real margin) const override {
        ord::ObstacleContact c;
        const Real dx = p.x - cx, dy = p.y - cy, d = std::hypot(dx, dy);
        if (d < r + margin && d > 1e-9) { c.hit = true; c.depth = r - d; c.normal = {dx / d, dy / d, 0}; c.friction = 0.3; c.id = 9; }
        return c;
    }
    ord::ObstacleContact edge(const ord::Vec3& a, const ord::Vec3& b, Real margin, Real& t) const override {
        ord::ObstacleContact c;
        const Real ex = b.x - a.x, ey = b.y - a.y, l2 = ex * ex + ey * ey;
        if (l2 < 1e-12) return c;
        t = std::clamp(((cx - a.x) * ex + (cy - a.y) * ey) / l2, 0.0, 1.0);
        const Real dx = a.x + ex * t - cx, dy = a.y + ey * t - cy, d = std::hypot(dx, dy);
        if (d < r + margin && d > 1e-9) { c.hit = true; c.depth = r - d; c.normal = {dx / d, dy / d, 0}; c.friction = 0.3; c.id = 9; }
        return c;
    }
};

struct Crash {
    Real impactKmh = 0, exitKmh = 0, dvKmh = 0;
    Real asi = 0, thivKmh = 0, oivX = 0, oivY = 0, ridedownG = 0, peakG = 0;
    Real crush = 0;
    int state = 0;
    Real tiltDeg = 0, stub = 0;
    std::vector<int> events;
    bool faulted = false;
    std::uint64_t hash = 0;
    std::string trace;  // CSV: t, speed, ax, objects' tilt
};

ord::VehicleDefinition testCar(Real mass) {
    ord::VehicleDefinition d = ord::vehicles::referenceHatchFwd();
    // the standards' test masses: scale the hatch (its crash structure is built from the mass)
    const Real k = mass / d.mass;
    d.mass = mass;
    d.inertia = d.inertia * k;
    return d;
}

// The car at `kmh` into object `type`, 3 m ahead, `offset` m to the side of the car's centreline.
Crash crash(const char* type, Real kmh, Real mass, Real offset = 0, bool rigid = false, Real seconds = 2.0) {
    World w(1);
    w.loadLibrary(libraryJson());
    w.substepsPerStep = 3;  // the vehicle steps at 360 Hz: ~1 kHz here
    ord::FlatGround g;
    ord::Vehicle v(testCar(mass), ord::makePreset(ord::DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, g);
    v.enableDamage(ord::DamageConfig{});
    const Real speed = kmh / 3.6;
    v.placeOnGround(0, 0, 0, g, speed);
    // The same car, coasting the same way with nothing in its way: what the object took is the
    // difference (the car's own drag and rolling resistance are not the object's).
    ord::Vehicle base(testCar(mass), ord::makePreset(ord::DrivingPreset::Simulation));
    base.reset({0, 0, 0.6}, 0, g);
    base.enableDamage(ord::DamageConfig{});
    base.placeOnGround(0, 0, 0, g, speed);
    Real front = -1e9;
    for (int i = 0; i < v.damage().nodeCount(); ++i) front = std::max(front, v.damage().node(i).rest.x);
    const Real x0 = v.state().position.x + front + 3.0;
    const int oi = w.addObject(type, x0, v.state().position.y + offset, 270);  // facing the car
    sdtest::StreetObstacles obs(w);
    RigidPost post(x0, v.state().position.y + offset, 0.5 * w.objectType(oi).section.d);
    if (rigid) v.setObstacles(&post);
    else v.setObstacles(&obs);

    Crash r;
    r.impactKmh = kmh;
    r.events.assign(16, 0);
    const Real dt = 1.0 / 360;
    // occupant risk: the car's CoM acceleration in its own frame
    std::deque<ord::Vec3> window;  // 50 ms (ASI)
    ord::Vec3 sum{};
    ord::Vec3 vPrev = v.state().velocity;
    // THIV / OIV: a free head at the driver's seat (at the CoM here), moving on at the car's
    // velocity at the start of the impact, until it reaches the flail space (0.6 m ahead, 0.3 m
    // to either side, in the car's frame)
    bool headFree = false, headHit = false;
    ord::Vec3 headPos{}, headVel{};
    Real tHit = 0;
    std::vector<ord::Vec3> accels;
    Real tContact = -1;
    std::vector<Real> exitSpeeds;
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
        obs.capture(v, dt);
        ord::DriverInput in;
        in.clutch = 1;  // in neutral, as crash-test cars run (no engine braking)
        v.step(dt, in, g);
        base.step(dt, in, g);
        // what the car's structure put on each object goes onto the object: the same impulse
        // on both sides, so momentum balances by construction
        for (int k = 0; k < v.damage().impactCount(); ++k) {
            const ord::ObstacleImpact& im = v.damage().impact(k);
            w.applyImpact(im.id, {im.point.x, im.point.y, im.point.z}, {im.normal.x, im.normal.y, im.normal.z}, im.impulse);
        }
        const Real coastLoss = speed - std::hypot(base.state().velocity.x, base.state().velocity.y);
        w.step(dt);
        Event ev[64];
        int n;
        while ((n = w.pollEvents(ev, 64)) > 0)
            for (int k = 0; k < n; ++k) {
                ++r.events[static_cast<std::size_t>(ev[k].type)];
                if (ev[k].type == static_cast<int>(EventType::ObjectHit) && tContact < 0) tContact = i * dt;
            }
        const ord::Vec3 vel = v.state().velocity;
        const ord::Vec3 aWorld = (vel - vPrev) / dt;
        vPrev = vel;
        const ord::Vec3 aCar = v.state().orientation.inverseRotate(aWorld);
        accels.push_back(aCar);
        window.push_back(aCar);
        sum += aCar;
        if (window.size() > 18) {  // 50 ms at 360 Hz
            sum -= window.front();
            window.pop_front();
        }
        if (window.size() == 18) {
            const ord::Vec3 m = sum / 18.0;
            const Real asi = std::sqrt(std::pow(m.x / (12 * kGravity), 2) + std::pow(m.y / (9 * kGravity), 2) +
                                       std::pow(m.z / (10 * kGravity), 2));
            r.asi = std::max(r.asi, asi);
        }
        r.peakG = std::max(r.peakG, std::abs(aCar.x) / kGravity);
        if (tContact >= 0 && !headFree) {
            headFree = true;
            headPos = v.centerOfMassWorld();
            headVel = vel;
        }
        if (headFree && !headHit) {
            headPos += headVel * dt;
            const ord::Vec3 rel = v.state().orientation.inverseRotate(headPos - v.centerOfMassWorld());
            if (rel.x > 0.6 || std::abs(rel.y) > 0.3) {
                headHit = true;
                tHit = i * dt;
                const ord::Vec3 relV = v.state().orientation.inverseRotate(headVel - vel);
                r.thivKmh = std::hypot(relV.x, relV.y) * 3.6;
                r.oivX = std::abs(relV.x);
                r.oivY = std::abs(relV.y);
            }
        }
        if (headHit && i * dt > tHit && i * dt < tHit + 0.5) {
            // ridedown: the 10 ms average after the head strikes
            const std::size_t k = accels.size();
            if (k >= 4) {
                ord::Vec3 s{};
                for (std::size_t j = k - 4; j < k; ++j) s += accels[j];
                r.ridedownG = std::max(r.ridedownG, std::hypot(s.x, s.y) / 4 / kGravity);
            }
        }
        // EN 12767's exit speed: where the car is 12 m past the impact point (here, up to then)
        const Real past = v.state().position.x - x0;
        if (past > 0 && past < 12) exitSpeeds.push_back(std::hypot(vel.x, vel.y) + coastLoss);
        if (i % 9 == 0)
            r.trace += std::to_string(i * dt) + "," + std::to_string(std::hypot(vel.x, vel.y) * 3.6) + "," +
                       std::to_string(aCar.x / kGravity) + "," + std::to_string(w.objectTilt(oi) / kDeg) + "\n";
    }
    const ord::Vec3 vEnd = v.state().velocity;
    const Real coastEnd = speed - std::hypot(base.state().velocity.x, base.state().velocity.y);
    r.exitKmh = (exitSpeeds.empty() ? std::hypot(vEnd.x, vEnd.y) + coastEnd : exitSpeeds.back()) * 3.6;
    r.dvKmh = kmh - r.exitKmh;
    r.crush = v.damageState().crushFront;
    r.state = static_cast<int>(w.objectState(oi));
    r.tiltDeg = w.objectTilt(oi) / kDeg;
    r.stub = w.objectStubHeight(oi);
    r.faulted = v.faulted();
    r.hash = w.stateHash() ^ (static_cast<std::uint64_t>(std::llround(v.state().position.x * 1e9)) << 1);
    return r;
}

void report(const char* what, const Crash& c) {
    std::printf("    %-34s exit %5.1f km/h (dv %5.1f)  ASI %.2f  THIV %4.1f km/h  OIV %.1f/%.1f m/s  ridedown %4.1f g  "
                "peak %4.1f g  crush %.2f m  object: state %d tilt %3.0f deg stub %.2f m\n",
                what, c.exitKmh, c.dvKmh, c.asi, c.thivKmh, c.oivX, c.oivY, c.ridedownG, c.peakG, c.crush, c.state, c.tiltDeg,
                c.stub);
}

// EN 12767:2019 energy-absorption category from the exit speed (Table A.3).
const char* enCategory(Real impact, Real exit) {
    if (impact >= 99) return exit <= 50 ? "HE" : exit <= 70 ? "LE" : "NE";
    if (impact >= 69) return exit <= 5 ? "HE" : exit <= 30 ? "LE" : "NE";
    return exit <= 0.5 ? "HE" : exit <= 5 ? "LE" : "NE";
}

}  // namespace

TEST_CASE("crash: a stop sign (2 1/2 in post) is passively safe: the car goes through, EN 12767 NE") {
    // EN 12767: a ~900 kg car at 100 km/h and at 35 km/h.
    const Crash fast = crash("stop_sign_r1", 100, 900), slow = crash("stop_sign_r1", 35, 900);
    report("stop sign, 900 kg, 100 km/h", fast);
    report("stop sign, 900 kg, 35 km/h", slow);
    std::printf("    EN 12767 category at 100 km/h: %s\n", enCategory(100, fast.exitKmh));
    CHECK(!fast.faulted && !slow.faulted);
    CHECK(std::string(enCategory(100, fast.exitKmh)) == "NE");
    CHECK(fast.dvKmh < 10);
    CHECK(slow.dvKmh < 10);
    CHECK(fast.asi < 1.0);                           // class A/B territory
    CHECK(slow.asi < 1.0);                           // the 35 km/h limit (1.0 / 27 km/h)
    CHECK(slow.thivKmh < 27);
    CHECK(fast.state >= static_cast<int>(ObjectState::Bent));
    CHECK(fast.tiltDeg > 45);                        // the post is down
    CHECK(fast.events[static_cast<int>(EventType::ObjectHit)] == 1);
}

TEST_CASE("crash: the same post made rigid stops the car (the coupling is what lets it through)") {
    const Crash yielding = crash("stop_sign_r1", 50, 1100), rigid = crash("stop_sign_r1", 50, 1100, 0, true);
    report("stop sign, 1100 kg, 50 km/h", yielding);
    report("as a rigid post", rigid);
    CHECK(rigid.dvKmh > 35);   // stopped (the rest is the coasting twin's own loss)
    CHECK(yielding.dvKmh < 0.25 * rigid.dvKmh);
    CHECK(yielding.crush < rigid.crush);
}

TEST_CASE("crash: MASH 1100C at 100 km/h through a slip-base sign: OIV and the stub within the limits") {
    const Crash c = crash("sign_slip_base", 100, 1100, 0.3);  // the quarter point
    report("slip base, 1100 kg, 100 km/h, 1/4 pt", c);
    CHECK(c.events[static_cast<int>(EventType::BaseReleased)] == 1);
    CHECK(c.oivX < 5.0);       // MASH limit (preferred 3.0 m/s)
    CHECK(c.ridedownG < 20.49);
    CHECK(c.stub <= 0.102);    // stub height limit
}

TEST_CASE("crash: bollards: cast iron snaps, a concrete-filled steel bollard stops a slow car") {
    const Crash iron = crash("bollard_cast_iron", 50, 1100);
    const Crash steel = crash("bollard_steel", 20, 1100);
    report("cast-iron frade, 1100 kg, 50 km/h", iron);
    report("steel bollard, 1100 kg, 20 km/h", steel);
    CHECK(iron.state == static_cast<int>(ObjectState::Broken));
    CHECK(iron.dvKmh > 1);           // 32 kg of brittle iron: about its momentum (~1.4 km/h)
    CHECK(iron.crush > 0.02);
    CHECK(steel.dvKmh > 0.6 * 20);   // a slow car is stopped, or nearly
    CHECK(steel.crush > 0.05);
}

TEST_CASE("crash: a bike rack folds under a car at 30 km/h") {
    const Crash c = crash("bike_rack_u", 30, 1250, 0.0);
    report("bike rack, 1250 kg, 30 km/h", c);
    CHECK(c.dvKmh < 8);
    CHECK(c.state >= static_cast<int>(ObjectState::Bent));
}

TEST_CASE("crash: street-name plates at 60 km/h: the post folds, the plates come off and fly") {
    const Crash c = crash("street_name_double", 60, 1250);
    report("street-name post, 1250 kg, 60 km/h", c);
    std::printf("    plates detached %d, landed %d, struck the car %d\n", c.events[static_cast<int>(EventType::PanelDetached)],
                c.events[static_cast<int>(EventType::PanelLanded)], c.events[static_cast<int>(EventType::DebrisHitBody)]);
    CHECK(c.dvKmh < 12);
    CHECK(c.state >= static_cast<int>(ObjectState::Bent));
    CHECK(c.events[static_cast<int>(EventType::DebrisHitBody)] <= 10);  // each loose part (plates, post pieces) once at most
}

TEST_CASE("crash: deterministic with the vehicle in the loop") {
    const Crash a = crash("stop_sign_r1", 80, 1100, 0.2), b = crash("stop_sign_r1", 80, 1100, 0.2);
    CHECK(a.hash == b.hash);
    CHECK(a.exitKmh == b.exitKmh);
}

TEST_CASE("crash: traces") {
    for (const auto& [type, kmh] : std::vector<std::pair<const char*, Real>>{
             {"stop_sign_r1", 100}, {"stop_sign_r1", 35}, {"bollard_cast_iron", 50}, {"bollard_steel", 20}}) {
        const Crash c = crash(type, kmh, type[0] == 's' ? 900 : 1100);
        std::ofstream f(std::string(SD_TRACE_DIR) + "/crash_" + type + "_" + std::to_string(static_cast<int>(kmh)) + ".csv");
        f << "t,speed_kmh,ax_g,tilt_deg\n" << c.trace;
    }
    CHECK(true);
}

TEST_CASE("crash: baseline: the car coasting with nothing in its way") {
    const Crash c = crash("stop_sign_r1", 100, 900, 40.0);  // the post 40 m to the side
    report("coasting, 900 kg, 100 km/h", c);
    CHECK(c.events[static_cast<int>(EventType::ObjectHit)] == 0);
}
