// Pedestrian Dynamics — the C ABI the engines bind (Unity P/Invoke, the browser's wasm).
// SPDX-License-Identifier: MIT
#include <string>

#include "ped/c_api.h"
#include "ped_test.hpp"

std::string bodyJson();
std::string vehiclesJson();

TEST_CASE("c api: load, add, a car strikes, report, events, impulses") {
    void* w = ped_create_world(3);
    const std::string body = bodyJson(), veh = vehiclesJson();
    CHECK(ped_load_body(w, body.c_str(), static_cast<int32_t>(body.size())) == 16);
    CHECK(ped_load_vehicles(w, veh.c_str(), static_cast<int32_t>(veh.size())) == 4);
    CHECK(ped_load_body(w, "{", 1) < 0);
    CHECK(std::string(ped_error(w)).size() > 0);
    CHECK(std::string(ped_segment_name(w, 3)) == "head");
    const int32_t i = ped_add(w, 0, 0, 1.5707963267948966, 1.75, 75, 0);
    CHECK(i == 0);
    ped_set_assist(w, i, 1);
    PedVehicle car{};
    car.id = 5;
    car.profile = ped_profile_index(w, "sedan");
    car.px = -2.6;
    car.qw = 1;
    car.vx = 40 / 3.6;
    double total = 0;
    for (int k = 0; k < 600; ++k) {
        ped_set_vehicles(w, &car, 1);
        ped_step(w, 1.0 / 60);
        double J[3], L[3];
        ped_vehicle_impulse(w, 0, J, L);
        total += J[0];
        car.px += car.vx / 60;
    }
    PedReport r{};
    CHECK(ped_report(w, i, &r) == 1);
    CHECK(r.struck == 1);
    CHECK(r.vehicle == 5);
    CHECK(r.trajectory == PED_WRAP || r.trajectory == PED_ROOF_VAULT);
    CHECK(r.hic_vehicle > 0);
    CHECK(total < -100);  // the person pushed back on the car
    PedEvent ev[256];
    const int32_t n = ped_poll_events(w, ev, 256);
    bool contact = false, head = false;
    for (int k = 0; k < n; ++k) contact |= ev[k].type == PED_VEHICLE_CONTACT, head |= ev[k].type == PED_HEAD_IMPACT;
    CHECK(contact && head);
    float parts[16 * 14];
    CHECK(ped_parts(w, i, parts, 16 * 14) == 16 * 14);
    CHECK(ped_parts(w, i, parts, 10) < 0);
    PedStats st{};
    ped_stats(w, &st);
    CHECK(st.pedestrians == 1);
    ped_destroy_world(w);
}
