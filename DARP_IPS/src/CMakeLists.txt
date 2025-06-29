add_subdirectory(data)
add_subdirectory(utilities)
add_subdirectory(solvers)
add_subdirectory(CplexSolver)
add_subdirectory(GurobiSolver)

set(SOURCE_FILES realTimeMain.cpp)

# build the static library
add_library(${DARP_IPS_LIB} SHARED ${SOURCE_FILES})
target_link_libraries(${DARP_IPS_LIB} data utilities solvers CplexSolver GurobiSolver ${LIBS})