# FindEigen3.cmake
# Priority:
#   1. Eigen3's own config package (CMAKE_PREFIX_PATH / installed configs).
#   2. Manual header search, honouring:
#        - user-provided EIGEN_DIR / EIGEN3_ROOT_DIR
#        - module env vars (EBROOTEIGEN, EIGEN_ROOT, ...)
#        - compiler include paths exported by `module load` (CPATH & friends)

# --- 1. Official config package (CONFIG forces config mode -> no recursion) ---
find_package(Eigen3 CONFIG QUIET)
if(Eigen3_FOUND AND TARGET Eigen3::Eigen)
    get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
    set(Eigen3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
    return()
endif()

# --- collect include dirs exported by environment modules via CPATH-style vars ---
set(_eigen_env_includes "")
foreach(_var CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH)
    if(DEFINED ENV{${_var}})
        string(REPLACE ":" ";" _paths "$ENV{${_var}}")
        list(APPEND _eigen_env_includes ${_paths})
    endif()
endforeach()

# --- 2. Manual header search ---
find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Core
        HINTS
        ${EIGEN3_ROOT_DIR}
        ${EIGEN_DIR}
        ENV EIGEN3_ROOT_DIR
        ENV EIGEN_DIR
        ENV EIGEN_ROOT
        ENV EBROOTEIGEN
        ENV Eigen3_ROOT
        ${_eigen_env_includes}      # <-- picks up the CPATH entry from `module load eigen`
        PATHS
        /usr/include
        /usr/local/include
        /opt/homebrew/include
        /opt/homebrew/opt/eigen/include
        /usr/local/opt/eigen/include
        PATH_SUFFIXES
        eigen3
        include
        include/eigen3
)

set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
set(Eigen3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")

# --- version ---
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

if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()