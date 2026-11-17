set(PD_TEST_SOURCES
  test_main.cpp
  test_data.cpp
  test_street.cpp
  test_alloc.cpp
  ${PROJECT_SOURCE_DIR}/src/c_api.cpp  # the C ABI, tested directly
)
add_executable(pd_tests ${PD_TEST_SOURCES})
target_link_libraries(pd_tests PRIVATE pd_core pd_scenarios)
target_compile_options(pd_tests PRIVATE ${PD_WARNINGS})
target_compile_definitions(pd_tests PRIVATE PD_TRACE_DIR="${CMAKE_BINARY_DIR}/traces")

# The runner prints per-case results; pass a substring to run a subset: pd_tests "street:"
add_test(NAME pd_tests COMMAND pd_tests)
