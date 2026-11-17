// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Scenarios shared by the tests, pdtrace and the benchmark: a residential
// street of poles with its wires (and a lanes tile for it), a car hitting a
// pole at bumper height, and a trace recorder (CSV + JSON).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pd/world.hpp"

namespace pdscn {

using pd::Real;

struct StreetSpec {
    int poles = 6;
    Real spacing = 35.0;    // m between poles
    int cls = 0;            // residential
    Real x0 = 0, y0 = 0;    // first pole
    int transformerAt = -1; // index that must carry a transformer (-1: whatever the seeds say)
    int transformerAt2 = -1; // a second one
    bool noTransformers = false;  // move poles until none has one (except transformerAt)
};

// A tile JSON for a straight street of poles running east (heading 90°):
// the street is to the south of the poles. Positions are nudged by
// decimetres until the seeds give the requested transformers.
std::string streetTile(const StreetSpec& s, std::vector<std::int32_t>* ids = nullptr);

// A lanes tile (T_p0_p0) with a two-lane road along the street, its kerb 0.5 m south of the poles.
std::string streetLanes(const StreetSpec& s);

// The car's hit: a 1,300 kg car at `kmh` into pole `id` at bumper height, pushing north
// (away from the street) with a `yawDeg` component along the street.
pd::ImpactResult hitPole(pd::World& w, std::int32_t id, Real kmh, Real height = 0.5, Real yawDeg = 0.0,
                         Real mass = 1300.0);

// Records pole tilts, states and wire tensions every host step.
struct Trace {
    std::vector<std::int32_t> poles;
    std::vector<int> spans;         // spans whose max tension is traced
    std::vector<Real> time;
    std::vector<std::vector<Real>> tilt;     // per pole, degrees
    std::vector<std::vector<int>> state;
    std::vector<std::vector<Real>> tension;  // per span, kN
    std::vector<std::string> events;
    void record(pd::World& w);
    void drain(pd::World& w);
    bool writeCsv(const std::string& path) const;
    bool writeJson(const std::string& path, const std::string& title) const;
};

// Runs `seconds` of host steps at 60 Hz, recording.
void run(pd::World& w, Trace& t, Real seconds);

const char* eventName(int type);

}  // namespace pdscn
