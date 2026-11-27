# Street Dynamics in the São Paulo World Unity project

`sp_street.cpp` is the glue that builds OpenRoad Dynamics (`Vehicle-Dynamics/web/ord_web.cpp`), Street Dynamics and the
vehicle adapter (`native/street-dynamics/tests/vehicle_adapter.*`) into one Windows library (`spworld3.dll`) for the
Unity project `SPDemoURP`. It includes `ord_web.cpp` by a relative path: adjust the `#include` to where
Vehicle-Dynamics is cloned.

Exports added on top of `ord_*` and `sd_*`: `sp_street_init(json, len)`, `sp_street_world()`, `sp_advance(dt)`
(replaces `ord_advance`), `sp_street_impulse()`, `sp_set_car_damage(on)`.

Build (Zig as the cross-compiler; copy `tests/vehicle_adapter.*` next to the sources or add `-Itests` and the .cpp):

    python3 -m ziglang c++ -target x86_64-windows-gnu -shared -std=c++20 -O2 -fno-exceptions -fno-rtti -DSD_EXPORT= \
      -I<Vehicle-Dynamics>/src/core/include -I<Vehicle-Dynamics>/src/vehicles \
      -Inative/street-dynamics/include -Inative/street-dynamics/tests \
      integrations/unity-saopauloworld/sp_street.cpp native/street-dynamics/tests/vehicle_adapter.cpp \
      native/street-dynamics/src/*.cpp <Vehicle-Dynamics>/src/core/src/*.cpp \
      <Vehicle-Dynamics>/src/vehicles/reference_vehicles.cpp <Vehicle-Dynamics>/src/vehicles/production_vehicles.cpp \
      -Wl,--export-all-symbols -s -o spworld3.dll

`street_library_sp.json` is `data/street_library.json` plus types used in the Unity scene. Everything added is an
ESTIMATE and says so in its `source` text: `lamp_mast_steel`, `signal_post_steel`, `signal_mast_arm_steel`,
`tree_young_05/10/15` and the material `wood_green_tipuana` (tree strengths carry a 0.5 tuning factor asked for by Rosser).
