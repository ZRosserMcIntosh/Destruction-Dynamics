cmake_minimum_required(VERSION 3.20)
project(PoleDynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(PD_BUILD_TESTS "Build Pole Dynamics tests" ON)
option(PD_BUILD_TOOLS "Build the pdtrace and pdhash tools" ON)
option(PD_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(MSVC)
  set(PD_WARNINGS /W4 /permissive-)
  if(PD_WARNINGS_AS_ERRORS)
    list(APPEND PD_WARNINGS /WX)
  endif()
  # Determinism (as Ballistic Dynamics' ADR 0004): strict IEEE, no FMA contraction.
  set(PD_FP_FLAGS /fp:precise)
else()
  set(PD_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(PD_WARNINGS_AS_ERRORS)
    list(APPEND PD_WARNINGS -Werror)
  endif()
  set(PD_FP_FLAGS -ffp-contract=off -fno-fast-math)
endif()

# ---- Engine-independent core (no engine headers) ----
add_library(pd_core
  src/dmath.cpp
  src/json.cpp
  src/poles.cpp
  src/world.cpp
  src/bodies.cpp
  src/wires.cpp
)
add_library(pd::core ALIAS pd_core)
target_include_directories(pd_core PUBLIC include)
target_compile_options(pd_core PRIVATE ${PD_WARNINGS} PUBLIC ${PD_FP_FLAGS})

# ---- The native plugin Unity loads (C ABI, include/pd/c_api.h) ----
add_library(PoleDynamics SHARED src/c_api.cpp)
target_link_libraries(PoleDynamics PRIVATE pd_core)
target_compile_options(PoleDynamics PRIVATE ${PD_WARNINGS})
set_target_properties(PoleDynamics PROPERTIES CXX_VISIBILITY_PRESET hidden)

if(PD_BUILD_TOOLS)
  add_library(pd_scenarios STATIC tools/scenarios.cpp)
  target_include_directories(pd_scenarios PUBLIC tools)
  target_link_libraries(pd_scenarios PUBLIC pd_core)
  target_compile_options(pd_scenarios PRIVATE ${PD_WARNINGS})
  add_executable(pdtrace tools/pdtrace.cpp)
  target_link_libraries(pdtrace PRIVATE pd_scenarios)
  target_compile_options(pdtrace PRIVATE ${PD_WARNINGS})
  add_executable(pdhash tools/pdhash.cpp)
  target_link_libraries(pdhash PRIVATE pd_scenarios)
  target_compile_options(pdhash PRIVATE ${PD_WARNINGS})
endif()

if(PD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
