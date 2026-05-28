add_subdirectory(data)
add_subdirectory(utilities)
add_subdirectory(solvers)
add_subdirectory(CplexSolver)
add_subdirectory(GurobiSolver)

# build the executable
add_executable(realtime_DARP realTimeMain.cpp)

# Always link the core libraries; the solver backend libraries are only
# linked when their corresponding USE_* option is enabled.
set(REALTIME_DARP_LIBS solvers data utilities)

if(USE_CPLEX)
    list(APPEND REALTIME_DARP_LIBS CplexSolver)
endif()

if(USE_GUROBI)
    list(APPEND REALTIME_DARP_LIBS GurobiSolver)
endif()

target_link_libraries(realtime_DARP PRIVATE ${REALTIME_DARP_LIBS} ${LIBS})
