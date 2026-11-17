// Pole Dynamics — run a scenario and print what happened.
// SPDX-License-Identifier: MIT
//
// Usage: pdtrace <kmh> [hit pole index] [seconds] [wetness] [transformer pole index] [out prefix] [yaw deg]
#include <cstdio>
#include <cstdlib>
#include <string>

#include "scenarios.hpp"

using namespace pd;

int main(int argc, char** argv) {
    const Real kmh = argc > 1 ? std::atof(argv[1]) : 50.0;
    const int hit = argc > 2 ? std::atoi(argv[2]) : 2;
    const Real seconds = argc > 3 ? std::atof(argv[3]) : 8.0;
    const Real wet = argc > 4 ? std::atof(argv[4]) : 0.0;
    const int tr = argc > 5 ? std::atoi(argv[5]) : -1;
    const std::string out = argc > 6 ? argv[6] : "";
    const Real yaw = argc > 7 ? std::atof(argv[7]) : 0.0;
    pdscn::StreetSpec spec;
    spec.transformerAt = tr;
    spec.noTransformers = true;
    std::vector<std::int32_t> ids;
    World w(1);
    w.params.sim.wetness = wet;
    if (w.loadTile(pdscn::streetTile(spec, &ids)) < 0) {
        std::fprintf(stderr, "load: %s\n", w.error().c_str());
        return 1;
    }
    w.loadLanes(pdscn::streetLanes(spec));
    pdscn::Trace t;
    t.poles = ids;
    for (int i = 0; i < w.spanCount() && i < 5; ++i) t.spans.push_back(i);
    for (int i = 0; i < 30; ++i) w.step(1.0 / 60.0);
    const ImpactResult r = pdscn::hitPole(w, ids[static_cast<std::size_t>(hit)], kmh, 0.5, yaw);
    std::printf("impact %.0f km/h: pole %d state %d broke %d impulse %.0f N s dv %.2f m/s peak %.0f kN dur %.3f s\n", kmh, r.pole,
                r.state, r.broke, r.impulse, r.vehicleDeltaV, r.peakForce / 1000, r.duration);
    pdscn::run(w, t, seconds);
    for (const auto& e : t.events) std::printf("  %s\n", e.c_str());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        PoleQuery q;
        w.queryPole(ids[i], q);
        Real maxTilt = 0;
        for (Real v : t.tilt[i]) maxTilt = v > maxTilt ? v : maxTilt;
        std::printf("pole %zu id %d: state %d tilt %.2f deg (max %.2f) lit %d powered %d transformer %d\n", i, ids[i], q.state, q.tiltDeg,
                    maxTilt, q.lit, q.powered, q.transformer);
    }
    const Stats s = w.stats();
    std::printf("awake spans %d frozen %d nodes %d bodies %d max step %.3f ms\n", s.awakeSpans, s.frozenSpans, s.nodes, s.bodies, s.maxStepMs);
    if (!out.empty()) {
        t.writeCsv(out + ".csv");
        t.writeJson(out + ".json", "pdtrace " + std::to_string(static_cast<int>(kmh)) + " km/h");
    }
    return 0;
}
