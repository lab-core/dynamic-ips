# Try to find the CPLEX, Concert, IloCplex and CP Optimizer libraries.
#
# Once done this will add the following imported targets:
#
#  cplex-library - the CPLEX library
#  cplex-concert - the Concert library
#  ilocplex - the IloCplex library
#  cplex-cp - the CP Optimizer library


# Find the path to CPLEX Studio.
# CPLEX Studio 12.4 can be installed in the following default locations:
#   /opt/ibm/ILOG/CPLEX_Studio<edition>124 - Linux
#   /opt/IBM/ILOG/CPLEX_Studio<edition>124 - UNIX
#   ~/Applications/IBM/ILOG/CPLEX_Studio<edition>124 - Mac OS X
#   C:\Program Files\IBM\ILOG\CPLEX_Studio<edition>124 - Windows
#     set(CPLEX_STUDIO_DIR /opt/ibm/ILOG/CPLEX_Studio221 /opt/IBM/ILOG/CPLEX_Studio221 /home/elamib/Documents/ibm/ILOG/CPLEX_Studio221 /home/ibm/cplex-studio/22.1)


if (UNIX)
    # Determine if the system is macOS or Linux and set the architecture accordingly
    if (APPLE)
        if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            set(CPLEX_ARCH arm64)
            set(CPLEX_STUDIO_DIR /Applications/CPLEX_Studio2211)
        else()
            set(CPLEX_ARCH x86-64)
            set(CPLEX_STUDIO_DIR /Applications/CPLEX_Studio221)
        endif()
        set(CPLEX_LIB_PATH_SUFFIXES lib/${CPLEX_ARCH}_osx/static_pic)
    else()
        # Default for Linux systems
        set(CPLEX_ARCH x86-64)
        # Check which CPLEX directory exists and set it accordingly.
        if (EXISTS "/scratch/elamib/ibm/ILOG/CPLEX_Studio2211")
            set(CPLEX_STUDIO_DIR "/scratch/elamib/ibm/ILOG/CPLEX_Studio2211")
        elseif (EXISTS "/home/ibm/cplex-studio/22.1.1")
            set(CPLEX_STUDIO_DIR "/home/ibm/cplex-studio/22.1.1")
        else()
            message(FATAL_ERROR "No valid CPLEX installation found on Linux system.")
        endif()
        set(CPLEX_LIB_PATH_SUFFIXES lib/${CPLEX_ARCH}_linux/static_pic)
    endif()
endif()

# Clear previous search paths to prevent fallback to older directories.
unset(CPLEX_INCLUDE_DIR CACHE)
unset(CPLEX_CONCERT_INCLUDE_DIR CACHE)
unset(CPLEX_CP_INCLUDE_DIR CACHE)
unset(CPLEX_LIBRARY CACHE)
unset(CPLEX_CONCERT_LIBRARY CACHE)
unset(CPLEX_ILOCPLEX_LIBRARY CACHE)
unset(CPLEX_CP_LIBRARY CACHE)

message("Detected platform: ${CMAKE_SYSTEM_NAME}, Processor: ${CMAKE_SYSTEM_PROCESSOR}")
message("CPLEX Studio directories: ${CPLEX_STUDIO_DIR}")
message("Library path suffixes: ${CPLEX_LIB_PATH_SUFFIXES}")

find_package(Threads)

# set directories
set(CPLEX_DIR ${CPLEX_STUDIO_DIR}/cplex)
set(CPLEX_CONCERT_DIR ${CPLEX_STUDIO_DIR}/concert)
set(CPLEX_CP_DIR ${CPLEX_STUDIO_DIR}/cpoptimizer)
message("CPLEX directories: ${CPLEX_DIR}")
message("Concert directories: ${CPLEX_CONCERT_DIR}")
message("CP Optimizer directories: ${CPLEX_CP_DIR}")

# Find the include directories.
find_path(CPLEX_INCLUDE_DIR ilcplex/cplex.h PATHS ${CPLEX_DIR}/include)
find_path(CPLEX_CONCERT_INCLUDE_DIR ilconcert/ilosys.h PATHS ${CPLEX_CONCERT_DIR}/include)
find_path(CPLEX_CP_INCLUDE_DIR ilcp/cp.h PATHS ${CPLEX_CP_DIR}/include)

message("CPLEX include directories: ${CPLEX_INCLUDE_DIR}")
message("Concert include directories: ${CPLEX_CONCERT_INCLUDE_DIR}")
message("CP Optimizer include directories: ${CPLEX_CP_INCLUDE_DIR}")

# ----------------------------------------------------------------------------
# CPLEX
# Find the CPLEX library.
find_library(CPLEX_LIBRARY NAMES cplex PATHS ${CPLEX_DIR} PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES})
set(CPLEX_LIBRARY_DEBUG ${CPLEX_LIBRARY})
message("CPLEX library: ${CPLEX_LIBRARY}")
message("CPLEX debug library: ${CPLEX_LIBRARY_DEBUG}")

# ----------------------------------------------------------------------------
# Concert

macro(find_cplex_library var name paths)
    find_library(${var} NAMES ${name} PATHS ${paths} PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES})
    set(${var}_DEBUG ${${var}})
endmacro()

# Find the Concert library.
find_cplex_library(CPLEX_CONCERT_LIBRARY concert ${CPLEX_CONCERT_DIR})
message("Concert library: ${CPLEX_CONCERT_LIBRARY}")

# ----------------------------------------------------------------------------
# IloCplex - depends on CPLEX and Concert

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wno-long-long HAVE_WNO_LONG_LONG_FLAG)
if (HAVE_WNO_LONG_LONG_FLAG)
    # Required if -pedantic is used.
    set(CPLEX_ILOCPLEX_DEFINITIONS -Wno-long-long)
endif ()

# Find the IloCplex include directory - normally the same as the one for CPLEX
# but check if ilocplex.h is there anyway.
find_path(CPLEX_ILOCPLEX_INCLUDE_DIR ilcplex/ilocplex.h PATHS ${CPLEX_INCLUDE_DIR})

# Find the IloCplex library.
find_cplex_library(CPLEX_ILOCPLEX_LIBRARY ilocplex ${CPLEX_DIR})
message("IloCplex library: ${CPLEX_ILOCPLEX_LIBRARY}")

# ----------------------------------------------------------------------------
# CP Optimizer - depends on Concert

# Find the CP Optimizer library.
find_cplex_library(CPLEX_CP_LIBRARY cp ${CPLEX_CP_DIR})
message("CP Optimizer library: ${CPLEX_CP_LIBRARY}")

set(CPLEX_COMPILER_FLAGS "-DIL_STD" CACHE STRING "Cplex Compiler Flags")

include(FindPackageHandleStandardArgs)
# Handle the QUIETLY and REQUIRED arguments and set CPLEX_FOUND to TRUE
# if all listed variables are TRUE.
set(CPLEX_FOUND TRUE)
find_package_handle_standard_args(
        CPLEX DEFAULT_MSG CPLEX_LIBRARY CPLEX_LIBRARY_DEBUG CPLEX_INCLUDE_DIR
        CPLEX_FOUND)
mark_as_advanced(CPLEX_LIBRARY CPLEX_LIBRARY_DEBUG CPLEX_INCLUDE_DIR)
if (CPLEX_FOUND AND NOT TARGET cplex-library)
    set(CPLEX_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} ${CMAKE_DL_LIBS})
    check_library_exists(m floor "" HAVE_LIBM)
    if (HAVE_LIBM)
        set(CPLEX_LINK_LIBRARIES ${CPLEX_LINK_LIBRARIES} m)
    endif ()
    add_library(cplex-library STATIC IMPORTED GLOBAL)
    set_target_properties(cplex-library PROPERTIES
            IMPORTED_LOCATION "${CPLEX_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${CPLEX_LIBRARY_DEBUG}"
            INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${CPLEX_LINK_LIBRARIES}")
endif ()

# Handle the QUIETLY and REQUIRED arguments and set CPLEX_CONCERT_FOUND to
# TRUE if all listed variables are TRUE.
set(CPLEX_CONCERT_FOUND TRUE)
find_package_handle_standard_args(
        CPLEX_CONCERT DEFAULT_MSG CPLEX_CONCERT_LIBRARY CPLEX_CONCERT_LIBRARY_DEBUG
        CPLEX_CONCERT_INCLUDE_DIR CPLEX_CONCERT_FOUND)
mark_as_advanced(CPLEX_CONCERT_LIBRARY CPLEX_CONCERT_LIBRARY_DEBUG
        CPLEX_CONCERT_INCLUDE_DIR)

if (CPLEX_CONCERT_FOUND AND NOT TARGET cplex-concert)
    add_library(cplex-concert STATIC IMPORTED GLOBAL)
    set_target_properties(cplex-concert PROPERTIES
            IMPORTED_LOCATION "${CPLEX_CONCERT_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${CPLEX_CONCERT_LIBRARY_DEBUG}"
            INTERFACE_COMPILE_DEFINITIONS IL_STD # Require standard compliance.
            INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_CONCERT_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}")
endif ()

# Handle the QUIETLY and REQUIRED arguments and set CPLEX_ILOCPLEX_FOUND to
# TRUE if all listed variables are TRUE.
set(CPLEX_ILOCPLEX_FOUND TRUE)
find_package_handle_standard_args(
        CPLEX_ILOCPLEX DEFAULT_MSG
        CPLEX_ILOCPLEX_LIBRARY CPLEX_ILOCPLEX_LIBRARY_DEBUG
        CPLEX_ILOCPLEX_FOUND)

mark_as_advanced(CPLEX_ILOCPLEX_LIBRARY CPLEX_ILOCPLEX_LIBRARY_DEBUG
        CPLEX_ILOCPLEX_INCLUDE_DIR)

if (CPLEX_ILOCPLEX_FOUND AND NOT TARGET ilocplex)
    add_library(ilocplex STATIC IMPORTED GLOBAL)
    set_target_properties(ilocplex PROPERTIES
            IMPORTED_LOCATION "${CPLEX_ILOCPLEX_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${CPLEX_ILOCPLEX_LIBRARY_DEBUG}"
            INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_ILOCPLEX_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "cplex-concert;cplex-library")
endif ()

# Handle the QUIETLY and REQUIRED arguments and set CPLEX_CP_FOUND to TRUE
# if all listed variables are TRUE.
set(CPLEX_CP_FOUND TRUE)
find_package_handle_standard_args(
        CPLEX_CP DEFAULT_MSG CPLEX_CP_LIBRARY CPLEX_CP_LIBRARY_DEBUG
        CPLEX_CP_INCLUDE_DIR CPLEX_CP_FOUND)

mark_as_advanced(CPLEX_CP_LIBRARY CPLEX_CP_LIBRARY_DEBUG CPLEX_CP_INCLUDE_DIR)

if (CPLEX_CP_FOUND AND NOT TARGET cplex-cp)
    add_library(cplex-cp STATIC IMPORTED GLOBAL)
    set_target_properties(cplex-cp PROPERTIES
            IMPORTED_LOCATION "${CPLEX_CP_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${CPLEX_CP_LIBRARY_DEBUG}"
            INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_CP_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "cplex-concert;${CPLEX_CP_EXTRA_LIBRARIES}")
endif ()
