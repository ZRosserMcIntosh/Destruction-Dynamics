// São Paulo World: OpenRoad Dynamics + Street Dynamics in one library (Unity, Windows).
// The car is a kinematic box to the street objects (sd::World::setBodies); what they push back
// with (bodyImpulse) goes on the car. 60 Hz for the objects, the car's own 360 Hz kept.
#include "../zrossermcintosh/vehicle-dynamics/web/ord_web.cpp"
#include "sd/world.hpp"
#include "sd/c_api.h"
#include "vehicle_adapter.hpp"

namespace {
sd::World* g_street = nullptr;
double g_streetImpulse = 0;   // N s on the car in the last advance (for the host's feedback)

struct Composite final : ObstacleQuery {
    static constexpr int kStreetId = 1000000;
    sdtest::StreetObstacles* street = nullptr;
    bool anyNear(const Vec3& c, Real r) const override { return g_obstacles.anyNear(c, r) || (street && street->anyNear(c, r)); }
    ObstacleContact point(const Vec3& p, const Vec3& ref, Real margin) const override {
        ObstacleContact a = g_obstacles.point(p, ref, margin);
        if (!street) return a;
        ObstacleContact b = street->point(p, ref, margin);
        if (b.hit) b.id = kStreetId + g_street->objectIndex(b.id);
        return (b.hit && (!a.hit || b.depth > a.depth)) ? b : a;
    }
    ObstacleContact edge(const Vec3& p, const Vec3& q, Real margin, Real& t) const override {
        Real ta = 0, tb = 0;
        ObstacleContact a = g_obstacles.edge(p, q, margin, ta);
        ObstacleContact b = street ? street->edge(p, q, margin, tb) : ObstacleContact{};
        if (b.hit) b.id = kStreetId + g_street->objectIndex(b.id);
        if (b.hit && (!a.hit || b.depth > a.depth)) { t = tb; return b; }
        t = ta; return a;
    }
};
Composite g_composite;
sdtest::StreetObstacles* g_streetObs = nullptr;
bool g_streetWork = false;
bool g_carDamage = true;
int g_restSteps = 0;
void streetStep() {
    Vehicle& v = *g.vehicle;
    const bool damaged = g.damageMode == 1 && v.damageEnabled();
    if (damaged) { v.setObstacles(&g_composite); g_streetObs->capture(v, kStep); }
    stepOnce();
    if (damaged) {
        const DamageModel& dm = v.damage();
        g_street->impactSpeedCap = 1.3 * length(v.state().velocity) + 2.0;
        for (int i = 0; i < dm.impactCount(); ++i) {
            const ObstacleImpact& im = dm.impact(i);
            if (im.id < Composite::kStreetId) continue;
            // A hit very low on a post has almost no lever on its base; in the ground the top ~0.3 m of soil gives
            // instead. The resistance is floored at that lever (kMinArm in the solver), so the push is applied there too,
            // or the base would never see enough moment to let go.
            const double pz = std::max(im.point.z, groundHeight(im.point.x, im.point.y) + 0.32);
            g_street->applyImpact(g_street->objectId(im.id - Composite::kStreetId), {im.point.x, im.point.y, pz}, {im.normal.x, im.normal.y, im.normal.z}, im.impulse);
            g_streetImpulse += im.impulse;
            g_streetWork = true;
        }
    }
    g_street->step(kStep);
}
}  // namespace

extern "C" {
ORD_EXPORT int sp_street_init(const char* json, int length) {
    delete g_street;
    delete g_streetObs;
    g_street = new sd::World(20261126u);
    g_street->substepsPerStep = 3;   // 1080 Hz inside, stepped with the car's 360 Hz
    g_streetObs = new sdtest::StreetObstacles(*g_street);
    g_composite.street = g_streetObs;
    const std::string err = g_street->loadLibrary(std::string_view(json, static_cast<std::size_t>(length)));
    if (!err.empty()) { g_composite.street = nullptr; delete g_streetObs; g_streetObs = nullptr; delete g_street; g_street = nullptr; return -1; }
    return static_cast<int>(g_street->library.size());
}
ORD_EXPORT void* sp_street_world() { return g_street; }
// Like ord_advance, with the street objects stepped alongside.
ORD_EXPORT int sp_advance(double dt) {
    if (!g.vehicle || g.vehicle->faulted()) return 0;
    g.accumulator += std::min(dt, 0.1);
    g_streetImpulse = 0;
    int n = 0;
    while (g.accumulator >= kStep) {
        if (g_street) streetStep(); else stepOnce();
        g.accumulator -= kStep;
        ++n;
        if (g.vehicle->faulted()) break;
    }
    if (!g_carDamage && g.damageMode == 1 && g.vehicle->damageEnabled()) {
        const DamageState& d = g.vehicle->damageState();
        const bool marked = d.crushFront > 0.005 || d.crushRear > 0.005 || d.crushLeft > 0.005 || d.crushRight > 0.005 || d.crushRoof > 0.005 || d.brokenBeams > 0;
        const bool slow = length(g.vehicle->state().velocity) < 1.5;
        g_restSteps = (marked && (!d.awake || slow)) ? g_restSteps + n : 0;
        if (g_restSteps > 270) { g.vehicle->repairDamage(); g_restSteps = 0; }   // three quarters of a second at rest or nearly stopped: as built again
    }
    publish();
    return n;
}
ORD_EXPORT double sp_street_impulse() { return g_streetImpulse; }
// Car damage on (1) or off (0). Off: the crash structure still takes the contacts (posts and walls behave the same),
// but nothing breaks on the car (no alignment, wheel, tyre, steering or engine damage), and once the structure has
// come to rest after a hit it is put back as built.
ORD_EXPORT void sp_set_car_damage(int on) {
    g_carDamage = on != 0;
    if (!g.vehicle || g.damageMode != 1) return;
    DamageConfig c; c.mechanical = g_carDamage;
    g.vehicle->enableDamage(c);
    g.vehicle->setObstacles(&g_composite);
}
}
