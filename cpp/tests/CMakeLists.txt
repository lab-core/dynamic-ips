# ---------------------------------------------------------------------------
# Tests
#
#   * unit_tests   — C++ unit/regression tests for the self-contained pieces
#                    (argument parsing, file readers). Links only the Gurobi/
#                    CPLEX-free `utilities` and `data` libraries, so it builds
#                    and runs without a solver license.
#   * toy_regression — end-to-end check that the bundled toy example still
#                    solves correctly. Needs the realtime_DARP binary, hence a
#                    solver backend; only registered when one is enabled.
#
# Run all tests from the build directory with: ctest --output-on-failure
# ---------------------------------------------------------------------------

add_executable(unit_tests
        test_main.cpp
        test_config_parser.cpp
        test_read_write.cpp)

target_link_libraries(unit_tests PRIVATE utilities data Threads::Threads)

add_test(NAME unit_tests COMMAND unit_tests)

# Repository root (one level above cpp/), where data/ToyExample lives.
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# The unit tests' toy-loading defaults probe for data/ToyExample relative to the
# working directory, so run them from the repository root.
set_tests_properties(unit_tests PROPERTIES WORKING_DIRECTORY "${REPO_ROOT}")

if(USE_GUROBI OR USE_CPLEX)
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME toy_regression
                COMMAND ${Python3_EXECUTABLE}
                        ${CMAKE_CURRENT_SOURCE_DIR}/regression_toy.py
                        $<TARGET_FILE:realtime_DARP>)
        set_tests_properties(toy_regression PROPERTIES
                WORKING_DIRECTORY "${REPO_ROOT}"
                LABELS "regression")
    endif()
endif()
