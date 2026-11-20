cmake_minimum_required(VERSION 3.20)
project(PedestrianDynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(PED_BUILD_TESTS "Build Pedestrian Dynamics tests" ON)
option(PED_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(MSVC)
  set(PED_WARNINGS /W4 /permissive-)
  if(PED_WARNINGS_AS_ERRORS)
    list(APPEND PED_WARNINGS /WX)
  endif()
  set(PED_FP_FLAGS /fp:precise)
else()
  set(PED_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(PED_WARNINGS_AS_ERRORS)
    list(APPEND PED_WARNINGS -Werror)
  endif()
  # Determinism (as Ballistic Dynamics' ADR 0004): strict IEEE, no FMA contraction.
  set(PED_FP_FLAGS -ffp-contract=off -fno-fast-math)
endif()

add_library(ped_core
  src/dmath.cpp
  src/json.cpp
  src/body.cpp
  src/vehicle.cpp
  src/world.cpp
  src/solver.cpp
)
add_library(ped::core ALIAS ped_core)
target_include_directories(ped_core PUBLIC include)
target_compile_options(ped_core PRIVATE ${PED_WARNINGS} PUBLIC ${PED_FP_FLAGS})
target_compile_definitions(ped_core PUBLIC PED_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_library(PedestrianDynamics SHARED src/c_api.cpp)
target_link_libraries(PedestrianDynamics PRIVATE ped_core)
target_compile_options(PedestrianDynamics PRIVATE ${PED_WARNINGS})
set_target_properties(PedestrianDynamics PROPERTIES CXX_VISIBILITY_PRESET hidden)

if(PED_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
