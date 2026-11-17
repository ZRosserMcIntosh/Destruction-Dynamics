// Pole Dynamics — step() must not allocate, and the step budget.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "pd/world.hpp"
#include "pd_test.hpp"
#include "scenarios.hpp"

namespace {
std::atomic<long> gAllocs{0};
bool gCounting = false;
}  // namespace

void* operator new(std::size_t n) {
    if (gCounting) gAllocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace pd;

TEST_CASE("alloc: no heap allocation inside step(), reportImpact() or pollEvents()") {
    pdscn::StreetSpec spec;
    spec.transformerAt = 1;
    spec.transformerAt2 = 4;
    spec.noTransformers = true;
    std::vector<std::int32_t> ids;
    World w(1);
    w.loadTile(pdscn::streetTile(spec, &ids));
    w.loadLanes(pdscn::streetLanes(spec));
    Event ev[256];
    gAllocs = 0;
    gCounting = true;
    for (int i = 0; i < 30; ++i) w.step(1.0 / 60.0);
    pdscn::hitPole(w, ids[1], 80, 0.5, 180.0);
    pdscn::hitPole(w, ids[4], 60);
    for (int i = 0; i < 60 * 10; ++i) {
        w.step(1.0 / 60.0);
        while (w.pollEvents(ev, 256) > 0) {}
    }
    gCounting = false;
    std::printf("    allocations during 10 s of a double hit: %ld\n", gAllocs.load());
    CHECK(gAllocs.load() == 0);
}

TEST_CASE("perf: step budget with 3 broken poles and ~40 woken spans") {
    // Two parallel streets of 12 poles; three poles hit at 80 km/h. The budget
    // window is the brief's case: >= 3 bodies moving and <= 45 woken spans.
    World w(1);
    std::vector<std::int32_t> a, b;
    pdscn::StreetSpec s1;
    s1.poles = 12;
    s1.noTransformers = true;
    w.loadTile(pdscn::streetTile(s1, &a));
    pdscn::StreetSpec s2 = s1;
    s2.y0 = -60;
    std::string t2 = pdscn::streetTile(s2, &b);
    w.loadTile(t2);
    for (int i = 0; i < 30; ++i) w.step(1.0 / 60.0);
    pdscn::hitPole(w, a[3], 80);
    pdscn::hitPole(w, a[8], 80);
    pdscn::hitPole(w, b[5], 80);
    std::vector<double> window, all;
    Event ev[256];
    for (int i = 0; i < 60 * 6; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        w.step(1.0 / 60.0);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        while (w.pollEvents(ev, 256) > 0) {}
        const Stats s = w.stats();
        all.push_back(ms);
        if (s.bodies >= 3 && s.awakeSpans <= 45) window.push_back(ms);
    }
    auto report = [](const char* name, std::vector<double> v) {
        if (v.empty()) return 0.0;
        double sum = 0;
        for (double x : v) sum += x;
        std::sort(v.begin(), v.end());
        const double mean = sum / static_cast<double>(v.size());
        std::printf("    %s: %zu steps, mean %.3f ms, p95 %.3f ms, max %.3f ms\n", name, v.size(), mean,
                    v[v.size() * 95 / 100], v.back());
        return mean;
    };
    const double mean = report("budget window (>=3 bodies, <=45 awake spans)", window);
    report("whole run", all);
    CHECK(!window.empty());
    // The brief's target is 0.3 ms on a laptop; this asserts a loose ceiling so
    // CI machines do not flake. README lists the measured numbers.
    CHECK(mean < 1.5);
}
