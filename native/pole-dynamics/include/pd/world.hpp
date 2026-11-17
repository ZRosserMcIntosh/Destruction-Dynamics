// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// The world: São Paulo World tiles of poles and spans, the wires on them, the
// poles that break, and the events the host reacts to.
//
// Cheap when nothing happens. A standing pole is a few numbers; a span at rest
// is an analytic catenary (sag from tension) the host draws itself. Only an
// impact, or a pole that moves, wakes things up: the pole becomes a rigid
// body on the soil (a ball joint with an elastic-plastic rotational spring at
// the embedment pivot), its spans and the spans up to `wakeNeighbours` poles
// away become XPBD ropes, and everything goes back to sleep when it settles.
// A span that settled where it was (unbroken, poles back in place) returns to
// its catenary; one that did not (broken, off its insulators, on the street)
// stays frozen as nodes for the session.
//
// Fixed step. The host calls step(dt) at its fixed rate (60 Hz); the world
// runs `substeps` internal steps per host step (240 Hz). The internal clock is
// an integer step count, so the result depends on the inputs, not on how the
// host's time was summed.
//
// No allocation in step(). Pools are sized when tiles load (poles, spans,
// lanes) and when the world is created (rope slots, rigid bodies, events).
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "pd/math.hpp"
#include "pd/params.hpp"
#include "pd/poles.hpp"

namespace pd {

enum class PoleState : int { Intact = 0, Cracked = 1, Leaning = 2, Snapped = 3, Fallen = 4 };

enum class EventType : int {
    PoleCracked = 1,
    PoleLeaning = 2,
    PoleSnapped = 3,
    PoleFell = 4,
    WireSnapped = 5,
    WireOnGroundLive = 6,
    Arc = 7,
    PowerOut = 8,
    LaneBlocked = 9,
    TransformerDetached = 10,
    TransformerLanded = 11,   // value: impact speed; ids: 1 = oil leak hook, 2 = fire hook
    ClampFailed = 12,
    PoleHitBody = 13,         // the falling pole hit a host body: span = body id, value = impulse
    LightChanged = 14,        // value 1 lit, 0 dark
    PoleRepaired = 15,
    LaneCleared = 16,
    PowerRestored = 17,
};

// Event record (also the C ABI layout: 4 x int32, 4 x double, 1 x int64).
struct Event {
    std::int32_t type = 0;
    std::int32_t pole = -1;  // pole ID (seed)
    std::int32_t span = -1;  // span index, or host body id for PoleHitBody
    std::int32_t wire = -1;  // wire index in the span
    double x = 0, y = 0, z = 0;
    double value = 0;
    std::int64_t ids = 0;    // packed lane ID, section ID, or flags
};

// A host body the wires and falling poles collide with (a car): a capsule.
struct Capsule {
    std::int32_t id = 0;
    Vec3 a, b;
    Real radius = 0;
    Vec3 velocity;
};

// What the vehicle module reports for a hit.
struct ImpactInput {
    Vec3 point;          // contact point (world)
    Vec3 normal;         // direction the car pushes the pole (into the pole)
    Vec3 relVelocity;    // car velocity relative to the pole
    Real vehicleMass = 1300;
    Real crushStiffness = 0;  // N/m (0: SimParams::carCrushStiffness)
    Real crushPlateau = 0;    // N, force at which the front stops stiffening (0: none)
};

struct ImpactResult {
    std::int32_t pole = -1;   // pole ID hit, -1 if none within reach
    int state = 0;            // PoleState after the hit
    Real impulse = 0;         // N s the pole took (the car loses this along the normal)
    Real vehicleDeltaV = 0;   // m/s the car loses along the normal
    Real peakForce = 0;       // N
    Real duration = 0;        // s
    int broke = 0;            // 1 if the pole snapped (the car may carry on)
};

struct PieceTransform {
    Vec3 position;   // of the piece's bottom end on its axis
    Quat rotation;   // body axes (x along the line, y left, z up) to world
    Real z0 = 0, z1 = 0;  // heights along the pole the piece spans
};

struct PoleQuery {
    std::int32_t id = 0;
    int state = 0;
    int pieces = 0;          // 1 standing / leaning, 2 snapped (stub + upper)
    PieceTransform piece[2];
    int transformer = 0;     // 0 none, 1 attached, 2 detached body
    Vec3 transformerPosition;
    Quat transformerRotation;
    int lit = 0;
    int powered = 0;
    Real tiltDeg = 0;
    std::int32_t section = -1;
};

struct Stats {
    int poles = 0, spans = 0, awakeSpans = 0, frozenSpans = 0, nodes = 0, bodies = 0;
    double lastStepMs = 0, maxStepMs = 0;
    std::int64_t steps = 0;
};

enum class SpanMode : int { Analytic = 0, Awake = 1, Frozen = 2 };

class World {
public:
    explicit World(std::uint32_t seed, int ropeSlots = 96, int bodySlots = 24);

    Params params;

    // ---- Tiles ----
    // Parses poles_<tile>.json ({tile, v, poles:[[x,y,hdg,cls]], spans:[[ax,ay,bx,by]]}).
    // Returns the number of poles added, or -1 on a parse error (error()).
    int loadTile(std::string_view json);
    // Parses lanes T_<i>_<j>.json (sao-paulo/lanes): {t, e:[[oneway, fwd, back, speed, w, x0,y0,...]], ...}.
    int loadLanes(std::string_view json);
    void unloadTile(int tileX, int tileY);
    const std::string& error() const { return error_; }

    // ---- Simulation ----
    void setBodies(const Capsule* caps, int n);
    int step(Real dt);  // returns the internal steps that did work (0 while everything sleeps)
    ImpactResult reportImpact(const ImpactInput& in);

    // ---- Queries ----
    int poleCount() const { return static_cast<int>(poles_.size()); }
    int poleIndex(std::int32_t id) const;
    std::int32_t poleId(int index) const { return poles_[static_cast<std::size_t>(index)].spec.id; }
    bool queryPole(std::int32_t id, PoleQuery& out) const;
    int spanCount() const { return static_cast<int>(spans_.size()); }
    SpanMode spanMode(int span) const;
    int spanWireCount(int span) const;
    // Node positions of every wire in a span (woken or frozen spans only):
    // [wireCount, then per wire: kind, broken-segment count, n, x0,y0,z0, ...].
    // Analytic spans return 0 (the host draws its catenary); curveOf() samples them.
    int wireNodes(int span, float* out, int capacity) const;
    // Samples any span (analytic or not) in the same layout (rendering in the lab).
    int wireCurve(int span, int samples, float* out, int capacity) const;
    // A span's end poles (IDs, -1 for a house or an unloaded tile) and wire tension (N, max over wires).
    void spanInfo(int span, std::int32_t& poleA, std::int32_t& poleB, Real& maxTension) const;
    int pollEvents(Event* out, int capacity);
    Stats stats() const { return stats_; }
    std::uint64_t stateHash() const;

    // ---- Persistence ----
    int saveTileState(int tileX, int tileY, std::uint8_t* out, int capacity) const;
    int loadTileState(int tileX, int tileY, const std::uint8_t* data, int size);
    bool repairPole(std::int32_t id);

    // ---- Introspection for tests and traces ----
    Real poleTilt(int index) const;
    PoleState poleState(int index) const { return poles_[static_cast<std::size_t>(index)].state; }
    Real spanTension(int span, int wire) const;  // N at the A end (analytic H/cos at rest)
    Real time() const { return static_cast<Real>(steps_) * subDt(); }
    Real subDt() const { return params.sim.hostStep / params.sim.substeps; }

private:
    // A rigid body: a falling pole piece or a detached transformer.
    struct Body {
        bool active = false;
        int pole = -1;          // index
        int kind = 0;           // 0 pole piece, 1 transformer
        Vec3 x;                 // centre of mass
        Quat q;
        Vec3 v, w;
        Real mass = 0, invMass = 0;
        Real iPerp = 0, iAxial = 0;
        Real z0 = 0, z1 = 0;    // piece extent along the pole (heights)
        Real comHeight = 0;     // along the axis
        Vec3 comOffset;         // body frame
        // Joint at the bottom: 0 none (free), 1 soil (embedment pivot), 2 rebar hinge.
        int joint = 0;
        Vec3 anchor;            // world
        Vec3 anchorLocal;       // body frame, from the centre of mass
        Vec3 plastic;           // soil plastic tilt (world rotation vector, horizontal)
        Vec3 restUp;            // pole axis at rest (soil) or at the snap (hinge)
        Real yield = 0;         // N m: soil yield, or hinge residual moment
        Real stiffness = 0;     // N m / rad
        Real still = 0;         // s at rest
        int calmWindows = 0;    // consecutive sleep windows below creep speed
        Vec3 windowX;           // pose at the start of the sleep window
        Quat windowQ;
        bool asleep = false;
        bool onGround = false;
        bool landedReported = false;
        Vec3 jointMoment;       // N m the joint carried this substep (world)
        Vec3 jointImpulse;      // N s, accumulated this substep
        Real tankAccel = 0;     // m/s^2 at the transformer this substep
        Vec3 tankVel;           // m/s at the transformer
        std::int32_t lastHitBody = -1;
    };

    struct Pole {
        std::uint64_t laneMask = 0;  // lanes its fallen piece was reported blocking
        PoleSpec spec;
        int tile = -1;
        Vec3 base;              // ground point
        Vec3 fwd, street;
        Quat rest;              // body axes to world when standing straight
        Quat tilt;              // static tilt applied on top of rest (cracks, settled lean)
        PoleState state = PoleState::Intact;
        Real strength = 1;      // from the seed
        Real breakHeight = 0;   // where it snapped (m), 0 if not
        Quat stubTilt;          // orientation of the stub after a snap
        int body = -1;          // main piece
        int transformerBody = -1;
        bool transformerAttached = false;
        bool lit = false, powered = true;
        std::int32_t section = -1;
        int span[12];           // attached span indices
        int spanCount = 0;
        std::array<std::uint8_t, 12> clampBroken{};  // per conductor slot: 0-2 MV, 3-6 LV, 7 telecom, 8-11 drops
        bool fellReported = false, snappedReported = false, leanReported = false, crackedReported = false;
        bool active = true;
        Vec3 restForce[12];     // wire load at each attachment at rest
        Vec3 guyDir;            // unit horizontal direction of the guy's pull (dead ends, corners), zero if none
        Real guyHeight = 0;     // m where the guy is attached
        Real guyCapacity = 0;   // N the guy takes before it slips (twice its design load)
        Vec3 wireMoment;        // N m the wires put on it about its pivot (this substep)
        Vec3 settledMoment;     // the same, when it last came to rest (a change from this wakes it)
    };

    struct Span {
        int a = -1, b = -1;     // pole indices; b = -1 for a drop or an unloaded neighbour
        Vec3 anchorA, anchorB;  // fixed ends when a pole is missing (house, other tile)
        bool drop = false;
        int dropSlot = 0;       // conductor slot at pole a for a drop (8..11)
        int wires = 0;
        std::array<int, 8> kind{};
        std::array<int, 8> slotA{}, slotB{};  // conductor slot at each end
        Real telecomMass = 0;
        SpanMode mode = SpanMode::Analytic;
        int rope = -1;          // rope slot
        Real still = 0;
        bool active = true;
        std::uint64_t laneMask = 0;  // lanes already reported blocked (bit: (edge * 8 + lane) % 64)
        std::uint64_t key = 0;
        std::array<Vec3, 8> restEnd[2];  // analytic pull of each wire on each end pole at rest
    };

    // A rope slot: all the wires of one span as XPBD chains.
    static constexpr int kWiresPerSlot = 8;
    static constexpr int kNodesPerWire = 24;
    static constexpr int kNodesPerSlot = kWiresPerSlot * kNodesPerWire;
    struct Wire {
        int n = 0;
        int kind = 0;
        std::uint32_t broken = 0;     // bit i: segment i (between node i and i+1) is broken
        std::uint8_t pinnedA = 1, pinnedB = 1;
        bool onGroundReported = false;
        bool live = false;
        Real arcTimer = 0;
        Real groundTime = 0;
        Real tension = 0;       // N this substep
        int quiet = 0;          // substeps it has not moved (it rests past kWireRest)
        Vec3 endA, endB;        // pinned end positions when it came to rest
        Real restLength = 0;    // m, natural length of the whole wire
        Real maxTension = 0;
    };
    struct Rope {
        bool used = false;
        int span = -1;
        std::array<Wire, kWiresPerSlot> wire{};
    };

    struct Join { int ropeA, nodeA, ropeB, nodeB; int pole, slot; bool active; };

    struct LaneEdge {
        int tileX = 0, tileY = 0, edge = 0, lanes = 0;
        int first = 0, count = 0;  // points in lanePts_
        Real minX = 0, minY = 0, maxX = 0, maxY = 0;
    };

    // ---- helpers (world.cpp) ----
    void emit(EventType t, int pole, int span, int wire, const Vec3& at, Real value, std::int64_t ids);
    int findPoleNear(const Vec3& p, Real radius) const;
    Vec3 attachmentPoint(int pole, int slot, Real* heightOut = nullptr) const;  // world
    Vec3 attachmentLocal(int pole, int slot) const;                            // height & lateral
    Vec3 spanEnd(const Span& s, int end, int wire) const;
    void restCatenary(const Span& s, int wire, const Vec3& pa, const Vec3& pb, int n, Vec3* out) const;
    const WireType& wireType(const Span& s, int wire) const;
    Real wireMassPerMetre(const Span& s, int wire) const;
    void computeRestForces(int pole);
    // Horizontal wire load on a standing pole beyond what its guy takes (tension only).
    Vec3 guyReaction(const Pole& p, const Vec3& horizontalPull) const;
    void recomputeSections();
    void recomputePower(bool emitEvents);
    void checkLanes(int pole, int span);
    void linkSpans(int firstNewSpan);
    // ---- bodies (bodies.cpp) ----
    int allocBody();
    void makeDynamic(int pole, int wakeDepth = -1);  // -1: SimParams::wakeNeighbours
    void snapPole(int pole, Real height, const Vec3& kickImpulse, const Vec3& kickPoint);
    void detachTransformer(int pole);
    void stepBodies(Real h);
    void bodyContacts(Body& b, Real h);
    Vec3 bodyPoint(const Body& b, const Vec3& local) const;
    Vec3 pieceLocal(const Body& b, int pole, int slot) const;
    Vec3 transformerPoint(int pole) const;
    void applyImpulse(Body& b, const Vec3& j, const Vec3& at);
    Vec3 invInertiaWorld(const Body& b, const Vec3& v) const;
    void updatePoleStates();
    Real momentCapacity(const Pole& p, const Vec3& momentDir, Real factor) const;
    // ---- ropes (wires.cpp) ----
    void wakeSpan(int span);
    void wakeAround(int pole, int depth);
    void sleepSpan(int span, bool toAnalytic);
    void stepRopes(Real h);
    void ropeForces();
    void checkClamps();
    void checkRopeSleep(Real dt);
    void hazards(Real dt);
    Real segTension(int rope, int wire, int seg) const;

    std::uint32_t seed_;
    std::string error_;
    std::vector<Pole> poles_;
    std::vector<Span> spans_;
    std::unordered_map<std::int32_t, int> poleById_;
    std::unordered_map<std::int64_t, int> poleByCell_;  // quantised position (0.1 m)
    std::unordered_map<std::uint64_t, int> spanByKey_;
    std::vector<Body> bodies_;
    std::vector<Rope> ropes_;
    // Rope state, struct-of-arrays per slot (kNodesPerSlot each).
    std::vector<Vec3> x_, prev_;
    std::vector<Real> invMass_, rest_, lambda_;
    std::vector<Join> joins_;
    // Per-attachment wire force on the poles this substep (pole index * 12 + slot).
    std::vector<Vec3> clampForce_;
    std::vector<Event> events_;
    int eventHead_ = 0, eventCount_ = 0;
    std::vector<Capsule> caps_;
    int capCount_ = 0;
    std::vector<LaneEdge> lanes_;
    std::vector<Vec3> lanePts_;
    std::unordered_map<std::int64_t, std::vector<std::uint8_t>> tileStore_;
    std::int64_t steps_ = 0;
    Real acc_ = 0;
    std::uint64_t rng_;
    bool powerDirty_ = false;
    Real lastStructural_ = -1e9;  // time of the last crack, lean, snap, fall, clamp failure or broken wire
    bool statsDirty_ = true;      // tiles or poles changed outside step(): recount the stats
    Stats stats_;
    // Scratch for power and lane checks (sized at load: step() never allocates).
    std::vector<std::uint8_t> reachedScratch_;
    std::vector<int> queueScratch_;
    std::vector<std::int32_t> sectionScratch_;
    std::vector<Vec3> footprintScratch_;
    // Poles with wire forces on them this substep (cleared and scanned instead of every pole).
    std::vector<int> touched_;
    std::vector<std::uint8_t> touchedFlag_;
    int touchedCount_ = 0;
    // Active lists (indices), fixed capacity.
    std::vector<int> awake_;
    int awakeCount_ = 0;
};

// Packs a lane ID: tile x (16 bits, signed), tile y (16 bits, signed), edge (24 bits), lane (8 bits).
inline std::int64_t packLane(int tileX, int tileY, int edge, int lane) {
    return static_cast<std::int64_t>((static_cast<std::uint64_t>(static_cast<std::uint16_t>(tileX)) << 48) |
                                     (static_cast<std::uint64_t>(static_cast<std::uint16_t>(tileY)) << 32) |
                                     (static_cast<std::uint64_t>(edge & 0xffffff) << 8) |
                                     static_cast<std::uint64_t>(lane & 0xff));
}

}  // namespace pd
