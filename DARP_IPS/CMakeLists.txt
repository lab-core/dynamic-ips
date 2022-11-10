cmake_minimum_required(VERSION 3.8.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
#set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE INTERNAL "" FORCE)
project(DARP_IPS)

# this code is to solve the following warning
# CMake Warning (dev) at The package name passed to `find_package_handle_standard_args`
# (CPLEX CONCERT) does not match the name of the calling package (CPLEX).
# This can lead to problems in calling code that expects `find_package` result variables (e.g., `_FOUND`)
# to follow a certain pattern. Call Stack (most recent call first):
# This warning is for project developers.  Use -Wno-dev to suppress it.

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

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
endif()

######## GUROBI Dependency
#find_package(GUROBI REQUIRED)
#include_directories(${GUROBI_INCLUDE_DIRS})

######## Eigen Dependency
if (UNIX)
    if (APPLE)
        set (EIGEN_DIR "/Applications/eigen-3.4.0")
    else()
        set (EIGEN_DIR /home/elamib/Documents/eigen-3.4.0 /home/ella/eigen-3.4.0)
    endif()
else()
    set (EIGEN_DIR "C:/Program Files/eigen-3.4.0")
endif()
set(EIGEN_INCLUDE_DIR ${EIGEN_DIR})
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

######## Compiler message
set(CMAKE_BUILD_TYPE Debug)
message(STATUS "System: ${CMAKE_SYSTEM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler version: ${CMAKE_CXX_COMPILER_VERSION}")


SET(CMAKE_CXX_STANDARD 14)

######## Set output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(DARP_IPS_LIB DARP_IPS)


INCLUDE_DIRECTORIES(src)
add_subdirectory(src)

#add_executable(static_DARP src/staticMain.cpp)
#add_executable(dynamic_DARP src/dynamicMain.cpp)
#add_executable(anytime_DARP src/anyTimeMain.cpp)
add_executable(realtime_DARP src/realTimeMain.cpp)

#target_link_libraries(dynamic_DARP ${DARP_IPS_LIB} cplex-library cplex-concert ilocplex)
#target_link_libraries(static_DARP ${DARP_IPS_LIB} cplex-library cplex-concert ilocplex)
#target_link_libraries(anytime_DARP ${DARP_IPS_LIB} cplex-library cplex-concert ilocplex)
target_link_libraries(realtime_DARP ${DARP_IPS_LIB} cplex-library cplex-concert ilocplex)