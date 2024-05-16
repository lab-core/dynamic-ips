cmake_minimum_required(VERSION 3.5)
project(DARP_IPS)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_CXX_STANDARD 14)

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


# Find Python
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(CG_binding src/CG_binding.cpp)


#include_directories(${Python3_INCLUDE_DIRS})
#link_directories(${Python3_INCLUDE_DIRS})


include_directories(${pybind11_INCLUDE_DIRS})
link_directories(${pybind11_INCLUDE_DIRS})


######## Compiler message
message(STATUS "System: ${CMAKE_SYSTEM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler version: ${CMAKE_CXX_COMPILER_VERSION}")

######## Set output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(DARP_IPS_LIB DARP_IPS)
SET(LIBS cplex-library cplex-concert ilocplex -lpthread pybind11::embed)


# Include pybind11
#INCLUDE_DIRECTORIES(src src/pybind11/include)
INCLUDE_DIRECTORIES(src)
add_subdirectory(src)

add_library(CGlib MODULE src/CG_binding.cpp)
target_link_libraries(CGlib PUBLIC pybind11::module ${DARP_IPS_LIB} ${LIBS})

add_executable(realtime_DARP src/realTimeMain.cpp)
target_link_libraries(realtime_DARP ${DARP_IPS_LIB} ${LIBS})
