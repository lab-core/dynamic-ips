add_subdirectory(data)
add_subdirectory(utilities)

set(SOURCE_FILES main.cpp solvers/ReducedProblem.cpp solvers/ReducedProblem.h)

# build the static library
add_library(${DARP_IPS_LIB} STATIC ${SOURCE_FILES})
target_link_libraries(${DARP_IPS_LIB} data utilities)