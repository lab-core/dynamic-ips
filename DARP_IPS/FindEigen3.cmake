# FindEigen3.cmake
# Simple Eigen3 finder

# Try official config first
find_package(Eigen3 CONFIG QUIET)
if(Eigen3_FOUND)
    return()
endif()

# Manual search
find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Core
        PATHS
        ${EIGEN3_ROOT_DIR}
        $ENV{EIGEN3_ROOT_DIR}
        /home/ella/eigen-3.4.0
        /Applications/eigen-3.4.0          # macOS
        "C:/Program Files/eigen-3.4.0"    # Windows
        /home/elamib/Documents/eigen-3.4.0
        PATH_SUFFIXES include eigen3
)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Get version
if(Eigen3_INCLUDE_DIRS AND EXISTS "${Eigen3_INCLUDE_DIRS}/Eigen/src/Core/util/Macros.h")
    file(READ "${Eigen3_INCLUDE_DIRS}/Eigen/src/Core/util/Macros.h" _eigen_macros)
    string(REGEX MATCH "#define EIGEN_WORLD_VERSION ([0-9]+)" _ "${_eigen_macros}")
    set(_world ${CMAKE_MATCH_1})
    string(REGEX MATCH "#define EIGEN_MAJOR_VERSION ([0-9]+)" _ "${_eigen_macros}")
    set(_major ${CMAKE_MATCH_1})
    string(REGEX MATCH "#define EIGEN_MINOR_VERSION ([0-9]+)" _ "${_eigen_macros}")
    set(_minor ${CMAKE_MATCH_1})
    set(Eigen3_VERSION "${_world}.${_major}.${_minor}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
        REQUIRED_VARS Eigen3_INCLUDE_DIRS
        VERSION_VAR Eigen3_VERSION
)

# Create target
if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Eigen3_INCLUDE_DIRS}"
    )
endif()
