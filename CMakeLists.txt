# ---------------------------------------------------------------------------
# Top-level CMake entry point.
#
# The actual C++ solver lives in cpp/. This thin wrapper exists so that IDEs
# (e.g. CLion) and `cmake -S . -B build` invocations launched from the
# repository root automatically pick up the solver build. All build options
# (SOLVER, GUROBI_ROOT, EIGEN_DIR, ENABLE_ASAN, ...) are defined in
# cpp/CMakeLists.txt and are forwarded transparently.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(dynamic_ips)

# Enable CTest at the top level so `ctest` works from a root-level build tree
# (the tests themselves are defined under cpp/tests).
enable_testing()

add_subdirectory(cpp)
