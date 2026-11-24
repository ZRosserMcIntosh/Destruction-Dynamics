// Pedestrian Dynamics — minimal dependency-free test harness (as Ballistic Dynamics' bd_test.hpp,
// OpenRoad Dynamics' ord_test.hpp and Aviation Dynamics' avd_test.hpp).
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace pedtest {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
inline int& failures() {
    static int f = 0;
    return f;
}
struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

// Long scenarios run at full length in optimised builds and shortened in
// Debug/sanitizer builds (same invariants, fewer simulated seconds).
inline constexpr bool kOptimisedBuild =
#ifdef NDEBUG
    true;
#else
    false;
#endif

}  // namespace pedtest

#define PED_CAT2(a, b) a##b
#define PED_CAT(a, b) PED_CAT2(a, b)
#define TEST_CASE(name)                                                                                 \
    static void PED_CAT(ped_test_fn_, __LINE__)();                                                      \
    static pedtest::Registrar PED_CAT(ped_test_reg_, __LINE__)(name, &PED_CAT(ped_test_fn_, __LINE__)); \
    static void PED_CAT(ped_test_fn_, __LINE__)()

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            ++pedtest::failures();                                                         \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #cond << "\n"; \
        }                                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                  \
    do {                                                                                                       \
        const double pd_a_ = (a), pd_b_ = (b), pd_t_ = (tol);                                               \
        if (!(std::abs(pd_a_ - pd_b_) <= pd_t_)) {                                                          \
            ++pedtest::failures();                                                                             \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| <= " #tol << "  (" \
                      << pd_a_ << " vs " << pd_b_ << ")\n";                                                  \
        }                                                                                                      \
    } while (0)

#define CHECK_RANGE(v, lo, hi)                                                                                     \
    do {                                                                                                           \
        const double pd_v_ = (v);                                                                                 \
        if (!(pd_v_ >= (lo) && pd_v_ <= (hi))) {                                                                 \
            ++pedtest::failures();                                                                                 \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #v " = " << pd_v_ << " not in [" << (lo) \
                      << ", " << (hi) << "]\n";                                                                    \
        }                                                                                                          \
    } while (0)

// ---- Shared fixtures ----
#include <fstream>
#include <sstream>


namespace pedtest {

inline std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace pedtest
