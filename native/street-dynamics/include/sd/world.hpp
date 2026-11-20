// Street Dynamics — destructible street furniture.
// SPDX-License-Identifier: MIT
//
// The world: street objects (posts, signs, bollards, racks) from the tiles,
// each built from its library type as a chain of rigid segments joined by
// elastic-plastic hinges, with its sign panels clamped on. Solved with XPBD
// rigid bodies (Müller et al. 2020, "Detailed Rigid Body Simulation with
// Extended Position Based Dynamics"): positional and angular constraints,
// substeps, no iterations.
//
// The host's vehicles are kinematic boxes here (setBodies): they push, drag
// and wrap the posts. The other half of the coupling is the vehicle's
// obstacle query (obstaclePoint / obstacleEdge): the current shape of every
// object, with the force it can put on the car (yield), so a sign post gives
// way and a bollard does not.
//
// Frame: ENU, z up, metres. Time: the host's fixed step (1/60 s), 16 substeps
// inside while anything is moving. Idle objects sleep and cost nothing.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "sd/library.hpp"
#include "sd/math.hpp"

namespace sd {

enum class ObjectState : int { Intact = 0, Bent = 1, Down = 2, Broken = 3 };

enum class EventType : int {
    ObjectHit = 1,       // value: closing speed m/s; ids: host body id
    PostBent = 2,        // value: plastic rotation (rad) at the worst hinge
    PostDown = 3,        // the post lies over (top below half its height)
    PostBroken = 4,      // a hinge tore through; value: stub height (m) left standing
    BaseReleased = 5,    // slip base, bolts, socket or footing let go; value: stub height (m)
    PanelDetached = 6,   // span: panel index; value: clamp force (N)
    PanelLanded = 7,     // span: panel index; value: landing speed (m/s)
    DebrisHitBody = 8,   // a loose part hit a host body: ids = host body id, value = impulse (N s)
    Repaired = 9,
};

// Same layout as Pole Dynamics' event (4 x int32, 4 x double, int64).
struct Event {
    std::int32_t type = 0;
    std::int32_t object = -1;  // object ID
    std::int32_t part = -1;    // panel index or segment
    std::int32_t reserved = 0;
    double x = 0, y = 0, z = 0;
    double value = 0;
    std::int64_t ids = 0;
};

// A host vehicle (or any moving solid) as an oriented box, kinematic here.
struct HostBody {
    std::int32_t id = 0;
    Vec3 centre;
    Quat orientation;
    Vec3 half;          // half extents (body frame)
    Vec3 velocity;      // of the centre
    Vec3 angularVelocity;
};

// What the vehicle's obstacle query gets back.
struct ObstacleHit {
    bool hit = false;
    Real depth = 0;       // m (> 0 inside)
    Vec3 normal;          // out of the obstacle
    std::int32_t id = -1; // object ID
    Real yieldForce = 0;  // N: the most the object can push back right now (0: rigid)
    Real friction = 0.35;
};

struct Stats {
    int objects = 0, awake = 0, bodies = 0, contacts = 0;
    double lastStepMs = 0, maxStepMs = 0;
    std::int64_t steps = 0;
};

class World {
public:
    explicit World(std::uint32_t seed);

    Library library;
    std::string loadLibrary(std::string_view json) { return error_ = library.load(json); }

    // street_<tile>.json: {"tile":"T_i_j","v":1,"objects":[[type, x, y, heading_deg], ...]}.
    // Returns objects added, -1 on error (error()).
    int loadTile(std::string_view json);
    void unloadTile(int tileX, int tileY);
    // One object (lab, tests). Returns its index, -1 if the type is unknown.
    int addObject(std::string_view type, Real x, Real y, Real headingDeg, Real groundZ = 0);
    const std::string& error() const { return error_; }

    // Hosts without a crash model of their own (traffic cars, a sliding wreck): kinematic
    // boxes that push, drag and wrap the objects.
    void setBodies(const HostBody* bodies, int count);
    // The other way, for a vehicle with a crash model (OpenRoad Dynamics' ObstacleImpact): the
    // impulse (N s) the car put on object `id` this step, at `point`, along -`normal` (the
    // normal points out of the object). Momentum balances exactly between the two engines.
    // Applied at the start of the next step().
    void applyImpact(std::int32_t id, const Vec3& point, const Vec3& normal, Real impulse);
    // Advances by dt (the host's step); returns the substeps run (0 while all sleep).
    int step(Real dt);

    // ---- the vehicle's obstacle query ----
    // Is any object within `radius` of `centre`? (broadphase, conservative)
    bool anyNear(const Vec3& centre, Real radius) const;
    // The deepest object surface the point is in (or within `margin` of). `pointVelocity` is
    // the car's velocity at the point and `stepTime` its step: the object's yield force
    // includes what it takes to set its parts moving at the car's speed in one step.
    ObstacleHit obstaclePoint(const Vec3& p, Real margin, const Vec3& pointVelocity, Real stepTime) const;
    // A thin part poking between two of the car's nodes: the deepest along the edge a-b.
    ObstacleHit obstacleEdge(const Vec3& a, const Vec3& b, Real margin, const Vec3& pointVelocity, Real stepTime,
                             Real& t) const;
    // The impulse (N s, world) the objects put on host body `id` in the last step.
    Vec3 bodyImpulse(std::int32_t id) const;

    // ---- queries ----
    int objectCount() const { return static_cast<int>(objects_.size()); }
    std::int32_t objectId(int i) const { return objects_[static_cast<std::size_t>(i)].id; }
    int objectIndex(std::int32_t id) const;
    const Archetype& objectType(int i) const { return library.at(objects_[static_cast<std::size_t>(i)].type); }
    ObjectState objectState(int i) const { return objects_[static_cast<std::size_t>(i)].state; }
    bool objectAwake(int i) const { return objects_[static_cast<std::size_t>(i)].awake; }
    Real objectTilt(int i) const;          // rad: the top segment's axis from vertical
    Real objectTopHeight(int i) const;     // m above its ground
    Real objectMaxPlastic(int i) const;    // rad: the worst hinge's plastic rotation
    Real objectStubHeight(int i) const;    // m standing after a break (0 intact)
    Vec3 objectPanelCentre(int i, int panel) const;
    bool objectPanelAttached(int i, int panel) const;
    // Render parts: per part 13 floats: kind (0 segment, 1 panel, 2 bar), index, x y z (centre),
    // qw qx qy qz, then half extents (segment: radius, radius, half length; panel: half width,
    // half thickness, half height). Returns floats written (negative: capacity too small).
    int parts(int i, float* out, int capacity) const;
    int pollEvents(Event* out, int capacity);
    Stats stats() const { return stats_; }
    std::uint64_t stateHash() const;
    Real time() const { return static_cast<Real>(substeps_) * subDt(); }
    Real subDt() const { return hostStep / substepsPerStep; }

    // Persistence: the diff of a tile (damaged objects); -(size) when capacity is too small.
    int saveTileState(int tileX, int tileY, std::uint8_t* out, int capacity) const;
    int loadTileState(int tileX, int tileY, const std::uint8_t* data, int size);
    bool repair(std::int32_t id);

    // ---- tunables ----
    Real hostStep = 1.0 / 60.0;
    int substepsPerStep = 16;     // 960 Hz inside
    Real hostFriction = 0.45;     // between parts and host bodies (paint on steel)
    Real groundFriction = 0.55;   // parts on asphalt or paving
    Real airDensity = 1.2;        // kg/m^3
    Real wetness = 0;             // 0 dry .. 1 saturated: the footings' soil is weaker
    Real jointDamping = 30;       // 1/s: relative spin across a joint decays at this rate (structural damping)
    Real sleepSpeed = 0.03;       // m/s
    Real sleepTime = 0.5;         // s

private:
    struct Body {
        Vec3 x, xPrev, v;
        Quat q, qPrev;
        Vec3 w;
        Vec3 settleX;      // pose when the object was last in motion (sleep: how far it drifts)
        Quat settleQ;
        Real invMass = 0;
        Vec3 invInertia;   // body frame (principal)
        int kind = 0;      // 0 segment (capsule along local z), 1 panel (box), 2 bar (capsule)
        Vec3 half;         // segment/bar: (radius, radius, half length); panel: half extents
        int object = -1;
        int panel = -1;
        bool loose = false;   // not held by any joint to the structure
        bool grounded = false;
        bool struck = false;  // (loose parts) already reported striking a car
    };
    struct Joint {
        int a = -1, b = -1;        // bodies; b = -1: the ground (the base)
        Vec3 ra, rb;               // attachment in a's and b's frames (rb: world point when b < 0)
        Quat rest;                 // q_b = q_a * rest (b < 0: q_a's target is conj(rest))
        Real compliance = 0;       // rad per N*m (angular, elastic)
        Real posCompliance = 0;    // m/N
        Real plastic = 0;          // rad of plastic rotation in the post's section
        Real soilPlastic = 0;      // rad in the footing (base joints)
        Vec3 lambdaPos, lambdaAng;  // the substep's multipliers (vectors: see applyAngular)
        Real lastForce = 0, lastTorque = 0;  // N, N*m in the last substep
        int kind = 0;              // 0 post hinge, 1 base, 2 panel clamp, 3 frame corner
        int panel = -1;
        bool broken = false;
    };
    struct Object {
        std::int32_t id = 0;
        int type = 0;
        Real x = 0, y = 0, heading = 0, groundZ = 0;
        int tile = 0;
        int firstBody = 0, bodyCount = 0, firstJoint = 0, jointCount = 0;
        int segmentsPerLeg = 0;
        ObjectState state = ObjectState::Intact;
        bool awake = false, active = true, hitReported = false, bentReported = false, downReported = false,
             brokenReported = false;
        std::array<bool, 4> panelLanded{};
        Real still = 0;
        Real stub = 0;
        Real contactForce = 0, lastContactForce = 0;  // N, the most any host body felt (this / last step)
        std::uint32_t variation = 0;
        Real cellR = 0;
    };
    struct PendingImpact {
        int body = -1;
        Vec3 point, impulse;
    };
    struct HostContact {
        int body = -1;      // our body
        int host = -1;      // index into hosts_
        Vec3 normal;        // out of the host box, toward our body
        Vec3 ra;            // contact point on our body, body frame
        Vec3 hostLocal;     // the contact point in the host box's frame (for friction)
        Real lambda = 0, lambdaT = 0;
    };

    void build(Object& o, int index);
    void wake(int index);
    void substep(Real h, Real tFrac, const std::vector<int>& set);
    void solveJoint(Joint& j, Real h);
    void plasticity(Object& o, Real h);
    void solveGround(int bi, Real h);
    void solveHosts(int bi, Real h, Real tFrac);
    void updateStates(Object& o, int index);
    void emit(EventType t, const Object& o, int part, const Vec3& at, Real value, std::int64_t ids = 0);
    void hostPose(int hi, Real tFrac, Vec3& c, Quat& q) const;
    Real staticResistance(const Object& o, const Vec3& p) const;
    Real bodyEffectiveMass(const Body& b, const Vec3& p, const Vec3& n) const;
    // `approach`: the car's motion relative to the part (unit, or zero): a point that has gone
    // past a post's axis along it is still pushed back the way it came (no tunnelling).
    ObstacleHit hitBody(int bi, const Vec3& p, Real margin, const Vec3& approach = {}) const;
    // XPBD primitives
    Vec3 invInertiaWorld(const Body& b, const Vec3& v) const;
    Real genInvMass(const Body& b, const Vec3& r, const Vec3& n) const;
    void applyPositional(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Real alpha, Real& lambda);
    void applyAngular(Body* a, Body* b, const Vec3& dphi, Real alpha, Real& lambda);
    void applyPositional(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Real alpha, Vec3& lambda);
    void applyAngular(Body* a, Body* b, const Vec3& dphi, Real alpha, Vec3& lambda);
    void index(int objectIndex);

    std::uint32_t seed_;
    std::vector<Object> objects_;
    std::vector<Body> bodies_;
    std::vector<Joint> joints_;
    std::vector<HostBody> hosts_;
    std::vector<Vec3> hostImpulse_;
    std::vector<int> awake_, plain_, frames_;  // awake objects; this step's split of them
    std::vector<HostContact> contacts_;   // scratch, reserved
    std::vector<PendingImpact> impacts_;  // reserved
    std::unordered_map<std::int64_t, std::vector<int>> grid_;  // 8 m cells: objects
    std::unordered_map<std::int32_t, int> byId_;
    std::unordered_map<std::int64_t, std::vector<std::uint8_t>> tileStore_;
    static constexpr int kMaxEvents = 1024;
    std::array<Event, kMaxEvents> events_{};
    int eventHead_ = 0, eventCount_ = 0;
    std::int64_t substeps_ = 0;
    Stats stats_;
    std::string error_;
};

std::int32_t objectSeed(Real x, Real y);  // the world builder's seed (as the poles)

}  // namespace sd
