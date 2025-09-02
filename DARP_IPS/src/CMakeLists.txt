add_subdirectory(data)
add_subdirectory(utilities)
add_subdirectory(solvers)
add_subdirectory(CplexSolver)
add_subdirectory(GurobiSolver)

# build the static library
add_executable(realtime_DARP realTimeMain.cpp)
target_link_libraries(realtime_DARP PRIVATE data utilities solvers CplexSolver GurobiSolver ${LIBS})