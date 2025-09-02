cmake_minimum_required(VERSION 3.16)
project(DARP_IPS)

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

# Check Solver
set(HAVE_CPLEX 0)
set(HAVE_GUROBI 0)

# Suppress developer warnings if not defined
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

######## JSON Library - Local header
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

######## Cplex Dependency
find_package(CPLEX REQUIRED)
if(CPLEX_FOUND)
    message("-----CPLEX IS FOUND-----")
    include_directories(${CPLEX_INCLUDE_DIR})
    link_directories(${CPLEX_LIBRARY})
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
    set(HAVE_CPLEX 1)
else()
    message(STATUS "CPLEX not found. Set CPLEX_HOME or disable ENABLE_CPLEX.")
endif()

find_package(Boost REQUIRED)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    message(STATUS "Boost found at: ${Boost_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Boost not found")
endif()

# Enable AddressSanitizer for debug builds
option(ENABLE_SANITIZERS "Enable address sanitizer for debug builds" ON)
if(ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Address sanitizer enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g -O1")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif()



######## GUROBI Dependency
if (APPLE)
    set(GUROBI_ROOT "/Library/gurobi1200/macos_universal2")
    set(GUROBI_INCLUDE_DIR "${GUROBI_ROOT}/include")
    set(GUROBI_LIBRARY "${GUROBI_ROOT}/lib/libgurobi120.dylib")
    set(GUROBI_CXX_LIBRARY "${GUROBI_ROOT}/lib/libgurobi_c++.a")
endif ()
find_package(GUROBI REQUIRED)
if(GUROBI_FOUND)
    message("-----GUROBI IS FOUND-----")
    include_directories(${GUROBI_INCLUDE_DIRS})
    link_directories(${GUROBI_LIBRARIES})
    set(HAVE_GUROBI 1)
else()
    message(STATUS "Gurobi not found. Set GUROBI_HOME or disable ENABLE_GUROBI.")
endif()

######## Eigen Dependency
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found at: ${Eigen3_INCLUDE_DIRS}")
    if(Eigen3_VERSION)
        message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
    endif()
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

# Threads for portability
find_package(Threads REQUIRED)

######## Compiler message
#set(CMAKE_BUILD_TYPE Release)
message(STATUS "System: ${CMAKE_SYSTEM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler version: ${CMAKE_CXX_COMPILER_VERSION}")


SET(CMAKE_CXX_STANDARD 17)

######## Set output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(DARP_IPS_LIB DARP_IPS)
SET(LIBS cplex-library cplex-concert ilocplex gurobi::gurobi_c++ -lpthread Eigen3::Eigen)


INCLUDE_DIRECTORIES(src)
add_subdirectory(src)