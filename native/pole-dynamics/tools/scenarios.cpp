// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
#include "scenarios.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

#include "pd/dmath.hpp"

namespace pdscn {

using namespace pd;

std::string streetTile(const StreetSpec& s, std::vector<std::int32_t>* ids) {
    std::ostringstream o;
    o.precision(10);
    o << "{\"tile\":\"T_p0_p0\",\"v\":1,\"poles\":[";
    std::vector<Real> xs, ys;
    for (int i = 0; i < s.poles; ++i) {
        Real x = s.x0 + i * s.spacing, y = s.y0;
        // Nudge along the line (decimetres) until the seed gives what we want.
        for (int k = 0; k < 400; ++k) {
            const Real xx = x + 0.1 * (k % 20) - 1.0, yy = y + 0.1 * (k / 20);
            const PoleSpec p = makePole(xx, yy, 90.0, s.cls);
            const bool wantT = i == s.transformerAt || i == s.transformerAt2;
            const bool ok = wantT ? p.transformer : (!s.noTransformers || !p.transformer);
            if (ok) {
                x = xx;
                y = yy;
                break;
            }
        }
        xs.push_back(x);
        ys.push_back(y);
        if (ids) ids->push_back(poleSeed(x, y));
        o << (i ? "," : "") << "[" << x << "," << y << ",90," << s.cls << "]";
    }
    o << "],\"spans\":[";
    for (int i = 0; i + 1 < s.poles; ++i)
        o << (i ? "," : "") << "[" << xs[static_cast<std::size_t>(i)] << "," << ys[static_cast<std::size_t>(i)] << ","
          << xs[static_cast<std::size_t>(i + 1)] << "," << ys[static_cast<std::size_t>(i + 1)] << "]";
    o << "]}";
    return o.str();
}

std::string streetLanes(const StreetSpec& s) {
    // Tile T_p0_p0: x from -250, y from +250 (north-west corner), 0.5 m pixels, y down.
    auto px = [](Real x) { return (x + 250.0) * 2.0; };
    auto py = [](Real y) { return (250.0 - y) * 2.0; };
    // Poles stand at the kerb: the near lane edge is 0.5 m from the pole line,
    // so the road's centre line is 0.5 + 3.2 m south of it.
    const Real y = s.y0 - 3.7;
    std::ostringstream o;
    o << "{\"t\":\"T_p0_p0\",\"e\":[[0,1,1,50,3," << px(s.x0 - 40) << "," << py(y) << "," << px(s.x0 + s.spacing * s.poles + 40) << ","
      << py(y) << "]],\"n\":[],\"k\":{}}";
    return o.str();
}

ImpactResult hitPole(World& w, std::int32_t id, Real kmh, Real height, Real yawDeg, Real mass) {
    PoleQuery q;
    w.queryPole(id, q);
    Real s, c;
    dm::sincos(yawDeg * kDeg, s, c);
    const Vec3 n{s, c, 0};  // north, turned toward east by yaw
    const Real v = kmh / 3.6;
    ImpactInput in;
    in.point = q.piece[0].position + Vec3{-n.x * 0.15, -n.y * 0.15, height};
    in.normal = n;
    in.relVelocity = n * v;
    in.vehicleMass = mass;
    return w.reportImpact(in);
}

const char* eventName(int t) {
    switch (t) {
        case 1: return "POLE_CRACKED";
        case 2: return "POLE_LEANING";
        case 3: return "POLE_SNAPPED";
        case 4: return "POLE_FELL";
        case 5: return "WIRE_SNAPPED";
        case 6: return "WIRE_ON_GROUND_LIVE";
        case 7: return "ARC";
        case 8: return "POWER_OUT";
        case 9: return "LANE_BLOCKED";
        case 10: return "TRANSFORMER_DETACHED";
        case 11: return "TRANSFORMER_LANDED";
        case 12: return "CLAMP_FAILED";
        case 13: return "POLE_HIT_BODY";
        case 14: return "LIGHT_CHANGED";
        case 15: return "POLE_REPAIRED";
        case 16: return "LANE_CLEARED";
        case 17: return "POWER_RESTORED";
        default: return "?";
    }
}

void Trace::record(World& w) {
    if (tilt.empty()) {
        tilt.resize(poles.size());
        state.resize(poles.size());
        tension.resize(spans.size());
    }
    time.push_back(w.time());
    for (std::size_t i = 0; i < poles.size(); ++i) {
        PoleQuery q;
        w.queryPole(poles[i], q);
        tilt[i].push_back(q.tiltDeg);
        state[i].push_back(q.state);
    }
    for (std::size_t i = 0; i < spans.size(); ++i) {
        std::int32_t a, b;
        Real t;
        w.spanInfo(spans[i], a, b, t);
        tension[i].push_back(t / 1000.0);
    }
}

void Trace::drain(World& w) {
    Event ev[256];
    int n;
    while ((n = w.pollEvents(ev, 256)) > 0)
        for (int i = 0; i < n; ++i) {
            char buf[256];
            std::snprintf(buf, sizeof buf, "%.3f %s pole=%d span=%d wire=%d value=%.1f ids=%lld at=(%.1f,%.1f,%.1f)", w.time(),
                          eventName(ev[i].type), ev[i].pole, ev[i].span, ev[i].wire, ev[i].value,
                          static_cast<long long>(ev[i].ids), ev[i].x, ev[i].y, ev[i].z);
            events.emplace_back(buf);
        }
}

void run(World& w, Trace& t, Real seconds) {
    const int steps = static_cast<int>(seconds * 60.0 + 0.5);
    t.drain(w);
    t.record(w);
    for (int i = 0; i < steps; ++i) {
        w.step(1.0 / 60.0);
        t.drain(w);
        t.record(w);
    }
}

bool Trace::writeCsv(const std::string& path) const {
    std::ofstream o(path);
    if (!o) return false;
    o << "t";
    for (std::size_t i = 0; i < poles.size(); ++i) o << ",tilt" << i << "_deg,state" << i;
    for (std::size_t i = 0; i < spans.size(); ++i) o << ",span" << spans[i] << "_kN";
    o << "\n";
    for (std::size_t k = 0; k < time.size(); ++k) {
        o << time[k];
        for (std::size_t i = 0; i < poles.size(); ++i) o << "," << tilt[i][k] << "," << state[i][k];
        for (std::size_t i = 0; i < spans.size(); ++i) o << "," << tension[i][k];
        o << "\n";
    }
    return true;
}

bool Trace::writeJson(const std::string& path, const std::string& title) const {
    std::ofstream o(path);
    if (!o) return false;
    auto arr = [&](const std::vector<Real>& v) {
        o << "[";
        for (std::size_t i = 0; i < v.size(); i += 2) o << (i ? "," : "") << v[i];  // every other step (30 Hz)
        o << "]";
    };
    o << "{\"title\":\"" << title << "\",\"t\":";
    arr(time);
    o << ",\"tilt\":[";
    for (std::size_t i = 0; i < tilt.size(); ++i) {
        o << (i ? "," : "");
        arr(tilt[i]);
    }
    o << "],\"tension\":[";
    for (std::size_t i = 0; i < tension.size(); ++i) {
        o << (i ? "," : "");
        arr(tension[i]);
    }
    o << "],\"events\":[";
    for (std::size_t i = 0; i < events.size(); ++i) o << (i ? "," : "") << "\"" << events[i] << "\"";
    o << "]}\n";
    return true;
}

}  // namespace pdscn
