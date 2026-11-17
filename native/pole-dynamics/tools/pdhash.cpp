// Pole Dynamics — the WebAssembly smoke test's scenario, natively, for comparing
// state hashes across builds: pdhash tile.json  (web/smoke_test.mjs writes it).
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <fstream>
#include <sstream>

#include "scenarios.hpp"

using namespace pd;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: pdhash tile.json\n");
        return 2;
    }
    std::ifstream f(argv[1]);
    std::stringstream ss;
    ss << f.rdbuf();
    World w(1);
    if (w.loadTile(ss.str()) != 6) {
        std::fprintf(stderr, "load: %s\n", w.error().c_str());
        return 1;
    }
    for (int i = 0; i < 30; ++i) w.step(1.0 / 60.0);
    const ImpactResult r = pdscn::hitPole(w, w.poleId(2), 80.0);
    for (int i = 0; i < 60 * 8; ++i) w.step(1.0 / 60.0);
    std::printf("impact: peak %.0f kN, broke %d\nhash %016llx\n", r.peakForce / 1000, r.broke ? 1 : 0,
                static_cast<unsigned long long>(w.stateHash()));
    return 0;
}
