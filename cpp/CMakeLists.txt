cmake_minimum_required(VERSION 3.16)
project(DARP_IPS)

# ---------------------------------------------------------------------------
# Build option: which commercial solver backend to compile in.
# Exactly one of CPLEX or GUROBI must be selected.
# Override on the command line:
#     cmake -DSOLVER=CPLEX  ..
#     cmake -DSOLVER=GUROBI ..
# Defaults to GUROBI to match the previous default build configuration.
# ---------------------------------------------------------------------------
# Set Solver backend to use (CPLEX, GUROBI, or NONE). Defaults to GUROBI;
# override on the command line with -DSOLVER=CPLEX / -DSOLVER=GUROBI / -DSOLVER=NONE.
# NONE builds the solver-free libraries and tests only (no commercial backend,
# no license) and is what CI uses. Using a cache variable (rather than a plain
# set) ensures the command-line value is honored.
set(SOLVER "GUROBI" CACHE STRING "Solver backend to use (CPLEX, GUROBI, or NONE)")
set_property(CACHE SOLVER PROPERTY STRINGS CPLEX GUROBI NONE)

if(SOLVER STREQUAL "CPLEX")
    set(USE_CPLEX  ON)
    set(USE_GUROBI OFF)
elseif(SOLVER STREQUAL "GUROBI")
    set(USE_CPLEX  OFF)
    set(USE_GUROBI ON)
elseif(SOLVER STREQUAL "NONE")
    set(USE_CPLEX  OFF)
    set(USE_GUROBI OFF)
else()
    message(FATAL_ERROR
        "SOLVER must be 'CPLEX', 'GUROBI', or 'NONE' (got '${SOLVER}'). "
        "Configure with -DSOLVER=CPLEX, -DSOLVER=GUROBI, or -DSOLVER=NONE.")
endif()


# Dynamically detect and set architecture for macOS
if (APPLE)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        set(CMAKE_OSX_ARCHITECTURES "arm64")
    else()
        set(CMAKE_OSX_ARCHITECTURES "x86_64")
    endif()
endif()

# Ensure the CMake module path includes the directory with FindCPLEX.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Check C++11 support
include(CheckCXXCompilerFlag)

# Suppress developer warnings if not defined
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

######## JSON Library - Local header
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

######## Cplex Dependency (optional)
if(USE_CPLEX)
    find_package(CPLEX)
    if(CPLEX_FOUND)
        message("-----CPLEX IS FOUND-----")
        include_directories(${CPLEX_INCLUDE_DIR})
        link_directories(${CPLEX_LIBRARY})
    else()
        message(WARNING
            "USE_CPLEX=ON but CPLEX was not found. "
            "CPLEX backend will be disabled for this build.")
        set(USE_CPLEX OFF)
    endif()

    if(CPLEX_CONCERT_FOUND)
        message("-----CPLEX CONCERT IS FOUND-----")
        include_directories(${CPLEX_CONCERT_INCLUDE_DIR})
        link_directories(${CPLEX_CONCERT_LIBRARY})
    endif()

    if(CPLEX_ILOCPLEX_FOUND)
        message("-----CPLEX ILOCPLEX IS FOUND-----")
        include_directories(${CPLEX_ILOCPLEX_INCLUDE_DIR})
        link_directories(${CPLEX_ILOCPLEX_LIBRARY})
    endif()
endif()

find_package(Boost REQUIRED)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    message(STATUS "Boost found at: ${Boost_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Boost not found")
endif()


######## GUROBI Dependency (optional)
# Search order:
#   1. -DGUROBI_ROOT=/path/to/gurobi passed at configure time
#   2. GUROBI_ROOT environment variable
#   3. Standard macOS install location (version-independent glob)
if(USE_GUROBI)
    if(NOT GUROBI_ROOT AND DEFINED ENV{GUROBI_ROOT})
        set(GUROBI_ROOT $ENV{GUROBI_ROOT})
    endif()
    if(NOT GUROBI_ROOT AND APPLE)
        file(GLOB _gurobi_candidates
            "/Library/gurobi*/macos_universal2"
            "/Library/gurobi*/macos_arm64"
            "/Library/gurobi*/mac64")
        if(_gurobi_candidates)
            list(SORT _gurobi_candidates ORDER DESCENDING)
            list(GET _gurobi_candidates 0 GUROBI_ROOT)
            message(STATUS "Gurobi auto-detected at: ${GUROBI_ROOT}")
        endif()
    endif()
    if(GUROBI_ROOT)
        set(GUROBI_INCLUDE_DIR "${GUROBI_ROOT}/include")
        file(GLOB _gurobi_lib "${GUROBI_ROOT}/lib/libgurobi[0-9]*.dylib"
                               "${GUROBI_ROOT}/lib/libgurobi[0-9]*.so")
        if(_gurobi_lib)
            list(GET _gurobi_lib 0 GUROBI_LIBRARY)
        endif()
        set(GUROBI_CXX_LIBRARY "${GUROBI_ROOT}/lib/libgurobi_c++.a")
    endif()
    find_package(GUROBI)
    if(GUROBI_FOUND)
        message("-----GUROBI IS FOUND-----")
        include_directories(${GUROBI_INCLUDE_DIRS})
        link_directories(${GUROBI_LIBRARIES})
    else()
        message(WARNING
            "USE_GUROBI=ON but Gurobi was not found. "
            "Gurobi backend will be disabled for this build.")
        set(USE_GUROBI OFF)
    endif()
endif()

######## Eigen Dependency
# Search order:
#   1. System-installed Eigen (e.g. apt install libeigen3-dev, brew install eigen)
#   2. -DEIGEN_DIR=/path/to/eigen passed at configure time
#   3. EIGEN_DIR environment variable
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found via find_package: ${EIGEN3_INCLUDE_DIR}")
    set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
else()
    if(NOT EIGEN_DIR AND DEFINED ENV{EIGEN_DIR})
        set(EIGEN_DIR $ENV{EIGEN_DIR})
    endif()
    if(EIGEN_DIR)
        message(STATUS "Eigen3 found via EIGEN_DIR: ${EIGEN_DIR}")
        set(EIGEN_INCLUDE_DIR ${EIGEN_DIR})
    else()
        message(FATAL_ERROR
            "Eigen3 not found. Install it system-wide or provide the path:\n"
            "  cmake -DEIGEN_DIR=/path/to/eigen-3.x.x ..\n"
            "  or: export EIGEN_DIR=/path/to/eigen-3.x.x")
    endif()
endif()
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

# Threads for portability
find_package(Threads REQUIRED)

######## Compiler message
#set(CMAKE_BUILD_TYPE Release)
message(STATUS "System: ${CMAKE_SYSTEM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler version: ${CMAKE_CXX_COMPILER_VERSION}")


SET(CMAKE_CXX_STANDARD 17)

# ---------------------------------------------------------------------------
# AddressSanitizer / LeakSanitizer
#   cmake -DENABLE_ASAN=ON -DCMAKE_BUILD_TYPE=Debug ..
# Note: LSan (leak detection) is Linux-only. On macOS, ASan catches
# use-after-free, overflows, and double-free but NOT memory leaks.
# ---------------------------------------------------------------------------
option(ENABLE_ASAN "Build with AddressSanitizer (+ LSan on Linux)" OFF)
if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address)
    message(STATUS "AddressSanitizer enabled")
endif()

######## Set output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(DARP_IPS_LIB DARP_IPS)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Assemble shared LIBS based on which solvers are enabled.
# Each backend's imported targets are only appended if its option is ON and
# the package was located. DARP_USE_CPLEX / DARP_USE_GUROBI are exposed as
# compile definitions so that source code can #ifdef around the relevant
# headers and code paths.
# ---------------------------------------------------------------------------
set(LIBS Threads::Threads)

if(USE_CPLEX)
    list(APPEND LIBS cplex-library cplex-concert ilocplex)
    add_compile_definitions(DARP_USE_CPLEX)
endif()

if(USE_GUROBI)
    list(APPEND LIBS gurobi::gurobi_c++)
    add_compile_definitions(DARP_USE_GUROBI)
endif()


INCLUDE_DIRECTORIES(src)
add_subdirectory(src)

# ---------------------------------------------------------------------------
# Tests (unit + regression). Disable with -DBUILD_TESTING=OFF.
# Run them from the build directory with: ctest --output-on-failure
# ---------------------------------------------------------------------------
option(BUILD_TESTING "Build the unit and regression tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Build configuration summary
# ---------------------------------------------------------------------------
message(STATUS "----------------------------------------------------------------")
message(STATUS "DARP build config: SOLVER=${SOLVER}")
message(STATUS "----------------------------------------------------------------")
