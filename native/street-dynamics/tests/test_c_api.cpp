// Street Dynamics — the C ABI the engines bind (Unity P/Invoke, the browser's wasm).
// SPDX-License-Identifier: MIT
#include <string>
#include <vector>

#include "sd/c_api.h"
#include "sd_test.hpp"

std::string libraryJson();

TEST_CASE("c api: library, an object, an impact, events, state, persistence") {
    void* w = sd_create_world(7);
    const std::string lib = libraryJson();
    CHECK(sd_load_library(w, lib.c_str(), static_cast<int32_t>(lib.size())) >= 9);  // the type count
    CHECK(sd_type_count(w) >= 9);
    CHECK(sd_load_library(w, "{", 1) < 0);
    CHECK(std::string(sd_error(w)).size() > 0);
    const int32_t id = sd_add_object(w, "stop_sign_r1", 10, 20, 90, 0);  // its ID
    CHECK(id == sd_object_id(w, 0));
    CHECK(sd_add_object(w, "no_such_type", 0, 0, 0, 0) < 0);
    // a blow at bumper height: 1.5 kN s (a small car at ~10 km/h, all of it into the post)
    sd_apply_impact(w, id, 10.0, 19.97, 0.5, 0, -1, 0, 1500);
    for (int i = 0; i < 120; ++i) sd_step(w, 1.0 / 60);
    SdObjectState st{};
    CHECK(sd_query_object(w, id, &st) == 1);
    CHECK(st.state >= SD_BENT);
    CHECK(st.tilt_deg > 5);
    SdEvent ev[32];
    const int32_t n = sd_poll_events(w, ev, 32);
    bool hit = false, bent = false;
    for (int i = 0; i < n; ++i) hit |= ev[i].type == SD_OBJECT_HIT, bent |= ev[i].type == SD_POST_BENT;
    CHECK(hit);
    CHECK(bent);
    float parts[13 * 16];
    const int32_t nf = sd_object_parts(w, id, parts, 13 * 16);
    CHECK(nf > 0 && nf % 13 == 0);
    CHECK(sd_object_parts(w, id, parts, 13) < 0);
    // persistence round trip into a fresh world
    std::vector<uint8_t> buf(16384);
    const int32_t size = sd_save_tile_state(w, 0, 0, buf.data(), static_cast<int32_t>(buf.size()));
    CHECK(size > 0);
    void* w2 = sd_create_world(7);
    sd_load_library(w2, lib.c_str(), static_cast<int32_t>(lib.size()));
    sd_add_object(w2, "stop_sign_r1", 10, 20, 90, 0);
    CHECK(sd_load_tile_state(w2, 0, 0, buf.data(), size) == 1);
    SdObjectState st2{};
    sd_query_object(w2, id, &st2);
    CHECK(st2.state == st.state);
    CHECK_NEAR(st2.tilt_deg, st.tilt_deg, 1e-6);
    CHECK(sd_repair(w2, id) == 1);
    sd_query_object(w2, id, &st2);
    CHECK(st2.state == SD_INTACT);
    SdStats stats{};
    sd_stats(w, &stats);
    CHECK(stats.objects == 1);
    CHECK(sd_state_hash(w) != sd_state_hash(w2));
    sd_destroy_world(w2);
    sd_destroy_world(w);
}
