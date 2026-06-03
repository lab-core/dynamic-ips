# FindGUROBI.cmake
# Finds the Gurobi optimization library
# Works with both local installations and HPC module systems
# This module defines:
#  GUROBI_FOUND - whether the system has Gurobi
#  GUROBI_INCLUDE_DIRS - the Gurobi include directory
#  GUROBI_LIBRARIES - the Gurobi libraries
#  GUROBI_CXX_LIBRARY - the Gurobi C++ library
#  GUROBI_LIBRARY - the Gurobi C library
#  GUROBI_VERSION - the Gurobi version (if detectable)

# Determine platform
if(APPLE)
  # Check for Apple Silicon vs Intel
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    set(GUROBI_PLATFORM "macos_universal2")
  else()
    set(GUROBI_PLATFORM "mac64")
  endif()
elseif(UNIX)
  set(GUROBI_PLATFORM "linux64")
elseif(WIN32)
  set(GUROBI_PLATFORM "win64")
endif()

# Priority order for finding Gurobi:
# 1. Environment variables (GUROBI_HOME, GUROBI_DIR)
# 2. Module system paths (common HPC locations)
# 3. Standard system locations
# 4. Local installation paths

set(GUROBI_SEARCH_PATHS)

# 1. Check environment variables first
if(DEFINED ENV{GUROBI_HOME})
  list(APPEND GUROBI_SEARCH_PATHS $ENV{GUROBI_HOME})
endif()

if(DEFINED ENV{GUROBI_DIR})
  list(APPEND GUROBI_SEARCH_PATHS $ENV{GUROBI_DIR})
endif()

if(DEFINED ENV{GUROBI_ROOT})
  list(APPEND GUROBI_SEARCH_PATHS $ENV{GUROBI_ROOT})
endif()

# 2. Common HPC/module system paths
list(APPEND GUROBI_SEARCH_PATHS
        # Common module system paths
        /opt/gurobi*
        /usr/local/gurobi*
        /sw/gurobi*
        /apps/gurobi*
        /shared/gurobi*
        # SLURM/LSF common paths
        ${EBROOTGUROBI}  # EasyBuild
        ${GUROBI_ROOT}   # Some module systems set this
)

# 3. Platform-specific paths
if(APPLE)
  list(APPEND GUROBI_SEARCH_PATHS
          /Library/gurobi*/macos_universal2
          /Library/gurobi*/mac64
          /Library/gurobi*
          /Applications/gurobi*/macos_universal2
          /Applications/gurobi*/mac64
          /Applications/gurobi*
          ~/gurobi*/macos_universal2
          ~/gurobi*/mac64
          ~/gurobi*
  )
elseif(UNIX)
  list(APPEND GUROBI_SEARCH_PATHS
          /usr/local/gurobi*/linux64
          /opt/gurobi*/linux64
          /usr/local/gurobi*
          /opt/gurobi*
          ~/gurobi*/linux64
          ~/gurobi*
  )
endif()

# Find Gurobi include directory
find_path(GUROBI_INCLUDE_DIRS
        NAMES gurobi_c++.h gurobi_c.h
        PATHS ${GUROBI_SEARCH_PATHS}
        PATH_SUFFIXES include
        DOC "Gurobi include directory"
        NO_DEFAULT_PATH  # Only search our specified paths first
)

# If not found, try system paths
if(NOT GUROBI_INCLUDE_DIRS)
  find_path(GUROBI_INCLUDE_DIRS
          NAMES gurobi_c++.h gurobi_c.h
          PATHS /usr/include /usr/local/include
          DOC "Gurobi include directory"
  )
endif()

# Construct library search paths
set(GUROBI_LIB_SEARCH_PATHS)
foreach(base_path ${GUROBI_SEARCH_PATHS})
  list(APPEND GUROBI_LIB_SEARCH_PATHS
          ${base_path}/lib
          ${base_path}/lib64
          ${base_path}/lib/${GUROBI_PLATFORM}
          ${base_path}/${GUROBI_PLATFORM}/lib
  )
endforeach()

# Add platform-specific paths for macOS (version-independent globs)
if(APPLE)
  list(APPEND GUROBI_LIB_SEARCH_PATHS
          /Library/gurobi*/macos_universal2/lib
          /Library/gurobi*/mac64/lib
  )
endif()

# Try to determine Gurobi version from include directory
if(GUROBI_INCLUDE_DIRS)
  # Extract version from path if possible
  string(REGEX MATCH "gurobi([0-9]+)" GUROBI_VERSION_MATCH ${GUROBI_INCLUDE_DIRS})
  if(GUROBI_VERSION_MATCH)
    string(REGEX REPLACE "gurobi([0-9])([0-9])([0-9])" "\\1.\\2.\\3" GUROBI_VERSION ${CMAKE_MATCH_1})
    set(GUROBI_VERSION_MAJOR ${CMAKE_MATCH_1})
    set(GUROBI_VERSION_MINOR ${CMAKE_MATCH_2})
    set(GUROBI_LIB_SUFFIX "${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
  endif()
endif()

# Common Gurobi library name patterns
set(GUROBI_LIB_NAMES
        gurobi${GUROBI_LIB_SUFFIX}
        gurobi130   # Version 13.0
        gurobi120   # Version 12.0
        gurobi110   # Version 11.0
        gurobi100   # Version 10.0
        gurobi95
        gurobi91
        gurobi90
        gurobi
)

# Find the main Gurobi library
find_library(GUROBI_LIBRARY
        NAMES ${GUROBI_LIB_NAMES}
        PATHS ${GUROBI_LIB_SEARCH_PATHS}
        DOC "Gurobi library"
        NO_DEFAULT_PATH
)

# If not found, try system paths
if(NOT GUROBI_LIBRARY)
  find_library(GUROBI_LIBRARY
          NAMES ${GUROBI_LIB_NAMES}
          PATHS /usr/lib /usr/local/lib /usr/lib64 /usr/local/lib64
          DOC "Gurobi library"
  )
endif()

# Find the Gurobi C++ library
# On some systems, the C++ library might have different names or be missing
set(GUROBI_CXX_LIB_NAMES
        libgurobi_c++.a      # Static library (common on macOS)
        gurobi_c++.a
        libgurobi_c++.dylib  # Dynamic library
        gurobi_c++.dylib
        libgurobi_c++.so     # Linux dynamic library
        gurobi_c++.so
        libgurobi_c++        # Without extension
        gurobi_c++
)

find_library(GUROBI_CXX_LIBRARY
        NAMES ${GUROBI_CXX_LIB_NAMES}
        PATHS ${GUROBI_LIB_SEARCH_PATHS}
        DOC "Gurobi C++ library"
        NO_DEFAULT_PATH
)

# If not found, try system paths
if(NOT GUROBI_CXX_LIBRARY)
  find_library(GUROBI_CXX_LIBRARY
          NAMES ${GUROBI_CXX_LIB_NAMES}
          PATHS /usr/lib /usr/local/lib /usr/lib64 /usr/local/lib64
          DOC "Gurobi C++ library"
  )
endif()

# Debug output to help troubleshoot
if(NOT GUROBI_CXX_LIBRARY AND GUROBI_LIBRARY)
  message(STATUS "Gurobi main library found at: ${GUROBI_LIBRARY}")
  message(STATUS "Searching for C++ library in: ${GUROBI_LIB_SEARCH_PATHS}")
  get_filename_component(GUROBI_LIB_DIR ${GUROBI_LIBRARY} DIRECTORY)
  file(GLOB GUROBI_LIB_FILES "${GUROBI_LIB_DIR}/*gurobi*")
  message(STATUS "Available Gurobi files in lib directory:")
  foreach(lib_file ${GUROBI_LIB_FILES})
    message(STATUS "  ${lib_file}")
  endforeach()
endif()

# Handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND
include(FindPackageHandleStandardArgs)

# Check if we have at least the main library and includes
if(GUROBI_LIBRARY AND GUROBI_INCLUDE_DIRS)
  if(GUROBI_CXX_LIBRARY)
    # Full installation with C++ library
    find_package_handle_standard_args(GUROBI
            FOUND_VAR GUROBI_FOUND
            REQUIRED_VARS GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIRS
            VERSION_VAR GUROBI_VERSION
    )
    set(GUROBI_LIBRARIES ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})
  else()
    # C library only (some installations might not have C++ wrapper)
    find_package_handle_standard_args(GUROBI
            FOUND_VAR GUROBI_FOUND
            REQUIRED_VARS GUROBI_LIBRARY GUROBI_INCLUDE_DIRS
            VERSION_VAR GUROBI_VERSION
    )
    set(GUROBI_LIBRARIES ${GUROBI_LIBRARY})
    message(WARNING "Gurobi C++ library not found. Only C library available. You may need to use the C API instead of C++.")
  endif()
else()
  find_package_handle_standard_args(GUROBI
          FOUND_VAR GUROBI_FOUND
          REQUIRED_VARS GUROBI_LIBRARY GUROBI_INCLUDE_DIRS
          VERSION_VAR GUROBI_VERSION
  )
endif()

if(GUROBI_FOUND)
  set(GUROBI_LIBRARIES ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})

  # Determine library types
  get_filename_component(GUROBI_LIB_EXT ${GUROBI_LIBRARY} EXT)
  get_filename_component(GUROBI_CXX_LIB_EXT ${GUROBI_CXX_LIBRARY} EXT)

  # Create imported targets for modern CMake
  if(NOT TARGET gurobi::gurobi)
    if(GUROBI_LIB_EXT STREQUAL ".a")
      add_library(gurobi::gurobi STATIC IMPORTED)
    else()
      add_library(gurobi::gurobi SHARED IMPORTED)
    endif()
    set_target_properties(gurobi::gurobi PROPERTIES
            IMPORTED_LOCATION ${GUROBI_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRS}
    )
  endif()

  if(NOT TARGET gurobi::gurobi_c++)
    if(GUROBI_CXX_LIB_EXT STREQUAL ".a")
      add_library(gurobi::gurobi_c++ STATIC IMPORTED)
    else()
      add_library(gurobi::gurobi_c++ SHARED IMPORTED)
    endif()
    set_target_properties(gurobi::gurobi_c++ PROPERTIES
            IMPORTED_LOCATION ${GUROBI_CXX_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRS}
            INTERFACE_LINK_LIBRARIES gurobi::gurobi
    )
  endif()

  message(STATUS "Found Gurobi: ${GUROBI_LIBRARY}")
  message(STATUS "Gurobi include dir: ${GUROBI_INCLUDE_DIRS}")
  message(STATUS "Gurobi libraries: ${GUROBI_LIBRARIES}")
  if(GUROBI_VERSION)
    message(STATUS "Gurobi version: ${GUROBI_VERSION}")
  endif()
else()
  if(GUROBI_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find Gurobi. Please ensure Gurobi is installed and GUROBI_HOME is set, or load the Gurobi module.")
  else()
    message(STATUS "Gurobi not found. If using an HPC system, make sure to load the Gurobi module first.")
  endif()
endif()

# Mark variables as advanced
mark_as_advanced(
        GUROBI_INCLUDE_DIRS
        GUROBI_LIBRARY
        GUROBI_CXX_LIBRARY
        GUROBI_VERSION
)