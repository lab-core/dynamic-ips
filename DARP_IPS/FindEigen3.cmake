# FindEigen3.cmake
# Simple Eigen3 finder

# Try official config first
find_package(Eigen3 CONFIG QUIET)

if(Eigen3_FOUND)
    if(TARGET Eigen3::Eigen)
        get_target_property(_eigen_include_dirs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

        set(EIGEN3_INCLUDE_DIR "${_eigen_include_dirs}")
        set(Eigen3_INCLUDE_DIRS "${_eigen_include_dirs}")
        set(EIGEN3_INCLUDE_DIRS "${_eigen_include_dirs}")
    endif()
    return()
endif()

# Manual search
find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Core
        PATHS
        ${EIGEN3_ROOT_DIR}
        $ENV{EIGEN3_ROOT_DIR}
        $ENV{EIGEN_DIR}
        /usr/include/eigen3
        /usr/local/include/eigen3
        /opt/homebrew/include/eigen3
        /opt/homebrew/opt/eigen/include/eigen3
        /usr/local/opt/eigen/include/eigen3
        /Applications/eigen-3.4.0
        "C:/Program Files/eigen-3.4.0"
        PATH_SUFFIXES include eigen3
)

set(Eigen3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")

# Get version
if(EIGEN3_INCLUDE_DIR AND EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
    file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen_macros)

    string(REGEX MATCH "#define EIGEN_WORLD_VERSION ([0-9]+)" _ "${_eigen_macros}")
    set(_world "${CMAKE_MATCH_1}")

    string(REGEX MATCH "#define EIGEN_MAJOR_VERSION ([0-9]+)" _ "${_eigen_macros}")
    set(_major "${CMAKE_MATCH_1}")

    string(REGEX MATCH "#define EIGEN_MINOR_VERSION ([0-9]+)" _ "${_eigen_macros}")
    set(_minor "${CMAKE_MATCH_1}")

    set(Eigen3_VERSION "${_world}.${_major}.${_minor}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
        REQUIRED_VARS EIGEN3_INCLUDE_DIR
        VERSION_VAR Eigen3_VERSION
)

# Create target
if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
    )
endif()