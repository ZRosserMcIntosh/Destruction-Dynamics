cmake_minimum_required(VERSION 3.20)
project(StreetDynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(SD_BUILD_TESTS "Build Street Dynamics tests" ON)
option(SD_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
# The crash tests drive the real vehicle model (OpenRoad Dynamics): point this at a
# Vehicle-Dynamics checkout to build them.
set(SD_VEHICLE_DYNAMICS "" CACHE PATH "Vehicle-Dynamics source tree (for the crash tests)")

if(MSVC)
  set(SD_WARNINGS /W4 /permissive-)
  if(SD_WARNINGS_AS_ERRORS)
    list(APPEND SD_WARNINGS /WX)
  endif()
  set(SD_FP_FLAGS /fp:precise)
else()
  set(SD_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(SD_WARNINGS_AS_ERRORS)
    list(APPEND SD_WARNINGS -Werror)
  endif()
  # Determinism (as Ballistic Dynamics' ADR 0004): strict IEEE, no FMA contraction.
  set(SD_FP_FLAGS -ffp-contract=off -fno-fast-math)
endif()

add_library(sd_core
  src/dmath.cpp
  src/json.cpp
  src/library.cpp
  src/world.cpp
  src/solver.cpp
)
add_library(sd::core ALIAS sd_core)
target_include_directories(sd_core PUBLIC include)
target_compile_options(sd_core PRIVATE ${SD_WARNINGS} PUBLIC ${SD_FP_FLAGS})
target_compile_definitions(sd_core PUBLIC SD_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_library(StreetDynamics SHARED src/c_api.cpp)
target_link_libraries(StreetDynamics PRIVATE sd_core)
target_compile_options(StreetDynamics PRIVATE ${SD_WARNINGS})
set_target_properties(StreetDynamics PROPERTIES CXX_VISIBILITY_PRESET hidden)

if(SD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
