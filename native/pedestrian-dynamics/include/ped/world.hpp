// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
//
// A world of pedestrians: each a chain of capsule segments (ped/body.hpp) joined by ball joints
// with anatomical limits, driven toward a pose by capped joint torques (an active ragdoll whose
// strength the host or a hit can take away), standing on the ground and struck by vehicles
// (ped/vehicle.hpp) that the host moves.
//
// The solver is XPBD on rigid bodies (Mueller et al. 2020), as in Street Dynamics: substeps,
// rigid ball joints, compliant limits and drives with vector multipliers, damped compliant
// contacts (the body's own tissue in series with what it hits). No allocation in step(); a
// pedestrian at rest sleeps; the same inputs give the same bits on every platform.
//
// Vehicles are moved by the host (the car's own model is the authority on the car). Every contact
// a person makes with a vehicle is an impulse the world sums per vehicle; the host applies it to
// the car after the step, so momentum balances between the two (a 75 kg person and a 1300 kg car:
// the car's lag of one step is small).
//
// Readouts for realism and validation: the head's HIC15 and peak acceleration, where and how fast
// the head met the vehicle, upper-tibia acceleration and knee bending (the EEVC legform's
// measures), ligament tears past the joint ranges, the throw distance and the kinematics class
// (wrap, forward projection, fender vault, roof vault).
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "ped/body.hpp"
#include "ped/math.hpp"
#include "ped/vehicle.hpp"

namespace ped {

enum class EventType : int {
    VehicleContact = 1,  // first contact with a vehicle: part = segment, other = region, value = closing speed
    HeadImpact = 2,      // the head strikes a vehicle region or the ground: value = normal speed (m/s)
    Landed = 3,          // back on the ground after being struck and airborne: value = airborne time (s)
    AtRest = 4,          // came to rest after being struck: value = throw distance (m)
    LigamentTear = 5,    // a joint forced past its range: part = segment (the joint's child), value = torque
    Hit = 6,             // applyImpulse: value = impulse (N s)
};

struct Event {
    EventType type = EventType::VehicleContact;
    std::int32_t pedestrian = 0, part = -1, other = -1;
    Vec3 at;
    Real value = 0, time = 0;
};

enum class Trajectory : int { None = 0, Wrap = 1, ForwardProjection = 2, FenderVault = 3, RoofVault = 4 };
const char* trajectoryName(Trajectory t);

struct Report {
    bool struck = false;
    std::int32_t vehicle = -1;          // id of the vehicle that struck first
    Real firstContactTime = 0;          // world time (s)
    Real impactSpeed = 0;               // closing speed at first contact (m/s)
    Vec3 throwOrigin;                   // pelvis at first contact
    bool atRest = false;
    Vec3 restPosition;                  // pelvis at rest
    Real throwDistance = 0;             // horizontal, origin to rest (m)
    // head
    int headRegion = -1;                // Region of the head's hardest strike on the vehicle
    Real headImpactSpeed = 0;           // its normal closing speed (m/s)
    Real headImpactTime = 0;            // after first contact (s)
    Vec3 headImpactPoint;               // vehicle frame
    Real groundHeadSpeed = 0;           // the head's hardest strike on the ground (m/s)
    Real hic15 = 0, headPeakG = 0;      // over the whole event
    Real hicT1 = 0, hicT2 = 0;          // its window (world time)
    Real hicVehicle = 0;                // windows ending while on the vehicle (the car's part)
    Real hicGround = 0;                 // windows ending after (the fall)
    Real groundHeadPeakG = 0;
    // legs (0 left, 1 right), in the bumper phase (the first 80 ms, as a legform test measures)
    Real tibiaPeakG[2] = {0, 0};
    Real kneeBendPeakDeg[2] = {0, 0};   // lateral bending (varus/valgus) of the knee
    std::uint32_t torn = 0;             // bit per segment whose joint tore
    // kinematics
    Real pelvisHeightAtImpact = 0;      // above the ground, at first contact
    Real bleHeight = 0;                 // the striking vehicle's bonnet leading edge
    bool vehiclePhase = false;          // still riding / touching the vehicle
    Real lastVehicleContact = 0;        // world time
    Real maxPelvisHeight = 0;           // above the ground
    Real carryTime = 0;                 // in contact with the vehicle (s)
    Real airTime = 0;                   // touching nothing (s)
    bool headOnUpperSurface = false;     // the head met the bonnet, cowl, windscreen, pillars or roof
    bool onTop = false;                 // the pelvis rode back over the bonnet (carried: wrap)
    bool overRoof = false;              // the pelvis went over the roof
    bool offSide = false;               // the pelvis left over a front wing (fender vault)
    Trajectory trajectory = Trajectory::None;
};

struct Stats {
    int pedestrians = 0, awake = 0;
    double lastStepMs = 0, maxStepMs = 0;
    std::int64_t steps = 0;
};

class World {
public:
    explicit World(std::uint32_t seed = 1);

    BodyDef body;
    VehicleLibrary vehicles;
    std::string loadBody(std::string_view json) { return error_ = body.load(json); }
    std::string loadVehicles(std::string_view json) { return error_ = vehicles.load(json); }
    const std::string& error() const { return error_; }

    // A person standing at (x, y) facing `heading` (rad from +x toward +y). Returns the index.
    int addPedestrian(Real x, Real y, Real heading, Real stature = 1.75, Real mass = 75, Real groundZ = 0);
    // Moves the ground under a person (kerbs, stairs; tests float people with it far below).
    void setGroundHeight(int i, Real z);
    int pedestrianCount() const { return static_cast<int>(peds_.size()); }
    std::int32_t pedestrianId(int i) const;
    void removeAll();

    // Muscle: 1 holds the pose at full strength, 0 is limp (a little tone, limpStrength, stays).
    void setStrength(int i, Real s);
    Real strength(int i) const;
    // Balance assist: a torque holding pelvis and chest upright (0..1), for standing in place.
    void setAssist(int i, Real a);
    // The pose the muscles drive toward: per joint (by its child segment), radians from neutral.
    void setJointTarget(int i, int segment, Real flex, Real abd, Real twist);

    // The vehicles this step (replaces last step's). Up to kMaxVehicles.
    static constexpr int kMaxVehicles = 32;
    void setVehicles(const VehicleState* v, int count);
    // The impulse (N s, world) the people put on vehicle `index` during the last step, and its
    // moment about the vehicle's origin (N m s). The host adds both to the car.
    void vehicleImpulse(int index, Vec3& linear, Vec3& angular) const;

    // A blow (a bullet, a shove): impulse (N s, world) at a world point on a segment.
    void applyImpulse(int i, int segment, const Vec3& point, const Vec3& impulse);

    // Advances by dt; returns substeps run (0 while everyone sleeps).
    int step(Real dt);

    // Queries
    int segmentCount() const { return body.size(); }
    const std::string& segmentName(int s) const { return body.at(s).name; }
    Vec3 segmentPosition(int i, int s) const;
    Quat segmentOrientation(int i, int s) const;
    Vec3 segmentVelocity(int i, int s) const;
    Real segmentMass(int i, int s) const;
    Vec3 centreOfMass(int i) const;
    Vec3 momentum(int i) const;
    // A joint's current angles (by its child segment): flexion, abduction, twist (rad).
    Vec3 jointAngles(int i, int segment) const;
    bool awake(int i) const;
    // Render parts: per segment 14 floats: x y z, qw qx qy qz, capsule end a (local xyz), end b,
    // radius. Returns floats written (negative: capacity too small).
    int parts(int i, float* out, int capacity) const;
    const Report& report(int i) const;
    int pollEvents(Event* out, int capacity);
    std::uint64_t stateHash() const;
    const Stats& stats() const { return stats_; }
    Real time() const { return time_; }

    // Tunables
    int substepsPerStep = 16;
    Real groundFriction = 0.66;     // a body sliding and tumbling on asphalt (Searle & Searle 1983)
    Real groundDamping = 0.6;       // damping ratio of body-on-ground contact
    Real limpStrength = 0.03;       // muscle tone that stays when limp
    Real limpOnContactSpeed = 1.5;  // m/s: a vehicle striking faster takes the muscles away
    Real dampingRatio = 1.0;        // of the joint drives
    Real jointFriction = 2.0;       // 1/s: relative spin across a joint decays at this rate
    Real assistStiffness = 3000;    // N m / rad, the balance assist on pelvis and chest
    Real assistTorque = 400;        // N m, its cap
    Real sleepSpeed = 0.03;         // m/s
    Real sleepTime = 0.5;           // s

private:
    struct Body {
        Vec3 x, xPrev, v, w;
        Quat q, qPrev;
        Real invMass = 0;
        Vec3 invInertia;            // body frame
        Vec3 la, lb;                // capsule axis ends, body frame
        Real radius = 0, halfLength = 0, tissueCompliance = 0;
        Vec3 vLast;                 // velocity at the end of the previous substep (accelerations)
        Vec3 settleX;
        Quat settleQ;
        bool grounded = false, wasGrounded = false;
        int vehicleRegion = -1, wasVehicleRegion = -1;  // the region touched this / last substep
    };
    struct Joint {
        int parent = -1, child = -1;
        Vec3 rp, rc;                // attachment in each body's frame
        Vec3 t, a, b;               // twist, flexion and abduction axes (parent frame)
        Real flexMin = 0, flexMax = 0, abdMin = 0, abdMax = 0, twistMin = 0, twistMax = 0;
        Real kp = 0, torque = 0, limitCompliance = 0, tearTorque = 0;
        Real inertia = 0;           // the child's largest moment (drive damping)
        Quat target;                // relative rotation the drive pulls toward
        Vec3 lambdaPos, lambdaLimit, lambdaDrive;
        bool torn = false;
    };
    static constexpr int kMaxPairs = kMaxSegments * (kMaxSegments - 1) / 2;
    static constexpr int kHicSamples = 64;
    struct Pedestrian {
        std::int32_t id = 0;
        int count = 0;
        std::array<Body, kMaxSegments> body{};
        std::array<Joint, kMaxSegments> joint{};  // joint[s] joins s to its parent (s > 0)
        std::array<std::array<std::uint8_t, 2>, kMaxPairs> pairs{};
        int pairCount = 0;
        Real strength = 1, assist = 0, heading = 0, groundZ = 0, stature = 1.75, mass = 75;
        Vec3 assistLambda[2];
        Quat assistTarget;
        bool awake = true;
        Real still = 0;
        std::uint32_t variation = 0;
        // impact bookkeeping
        Report report;
        int vehicleIndex = -1;      // this step's index of the vehicle that struck
        bool airborne = false;
        Real airborneSince = 0;
        bool landedReported = false;
        // HIC15: cumulative integral of the head's acceleration (g s) at each substep
        std::array<Real, kHicSamples> hicT{}, hicS{};
        int hicHead = 0, hicCount = 0;
        Real hicCum = 0;
    };

    // solver
    void predict(Pedestrian& p, Real h);
    void solveJoints(Pedestrian& p, Real h, bool drives);
    void solveGround(Pedestrian& p, Real h);
    void solveVehicles(Pedestrian& p, Real h, Real tFrac);
    void solveSelf(Pedestrian& p, Real h);
    void finishSubstep(Pedestrian& p, Real h);
    void vehiclePose(int v, Real tFrac, Vec3& c, Quat& q) const;
    void wake(Pedestrian& p);
    void emit(EventType type, const Pedestrian& p, int part, int other, const Vec3& at, Real value);
    void markStruck(Pedestrian& p, int vehicle, Real speed, int segment, int region, const Vec3& at);
    void classify(Pedestrian& p);
    Vec3 invInertiaWorld(const Body& b, const Vec3& v) const;
    Real genInvMass(const Body& b, const Vec3& r, const Vec3& n) const;
    Real applyPositional(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Real alpha, Real gamma,
                         Real approach);
    void applyPositionalVec(Body& a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Vec3& lambda);
    void applyAngular(Body& a, Body* b, const Vec3& dphi, Real alpha, Vec3& lambda, Real maxLambda);

    void resolveSegments();

    std::uint32_t seed_;
    std::string error_;
    int pelvisSeg_ = 0, thoraxSeg_ = -1, headSeg_ = -1, shankSeg_[2] = {-1, -1};
    std::vector<Pedestrian> peds_;
    std::int32_t nextId_ = 1;
    std::array<VehicleState, kMaxVehicles> veh_{};
    std::array<Vec3, kMaxVehicles> vehJ_{}, vehL_{};
    int vehCount_ = 0;
    Real stepDt_ = 1.0 / 60, time_ = 0, subTime_ = 0;
    std::array<Event, 1024> events_{};
    int evHead_ = 0, evCount_ = 0;
    Stats stats_;
};

}  // namespace ped
