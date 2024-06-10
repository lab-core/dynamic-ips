cmake_minimum_required(VERSION 3.28.1)
project(DARP_IPS)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

# CPLEX Dependency
find_package(CPLEX REQUIRED)
if(CPLEX_FOUND)
    message("----- CPLEX IS FOUND -----")
    include_directories(${CPLEX_INCLUDE_DIR})
    link_directories(${CPLEX_LIBRARY})
endif()

if(CPLEX_CONCERT_FOUND)
    message("----- CPLEX CONCERT IS FOUND -----")
    include_directories(${CPLEX_CONCERT_INCLUDE_DIR})
    link_directories(${CPLEX_CONCERT_LIBRARY})
endif()

if(CPLEX_ILOCPLEX_FOUND)
    message("----- CPLEX ILOCPLEX IS FOUND -----")
    include_directories(${CPLEX_ILOCPLEX_INCLUDE_DIR})
    link_directories(${CPLEX_ILOCPLEX_LIBRARY})
endif()

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development.Module Development.Embed REQUIRED)
message(STATUS "Python3 executable: ${Python3_EXECUTABLE}")

find_package(pybind11 CONFIG REQUIRED)

# Compiler message
message(STATUS "System: ${CMAKE_SYSTEM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler version: ${CMAKE_CXX_COMPILER_VERSION}")

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(DARP_IPS_LIB DARP_IPS)
set(LIBS cplex-library cplex-concert ilocplex -lpthread pybind11::embed pybind11::module)

include_directories(src)
add_subdirectory(src)
#add_subdirectory(src/pybind11)
include_directories(src ${Python_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS})

# Add the pybind11 module
pybind11_add_module(CG_binding src/CG_binding.cpp)
target_link_libraries(CG_binding PRIVATE ${DARP_IPS_LIB} ${LIBS})

# Add the executable for real-time DARP
add_executable(realtime_DARP src/realTimeMain.cpp)
target_link_libraries(realtime_DARP PRIVATE ${DARP_IPS_LIB} ${LIBS})

