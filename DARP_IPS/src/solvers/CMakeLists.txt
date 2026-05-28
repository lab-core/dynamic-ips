set(LIB_NAME solvers)

set(SOURCE_FILES
        MasterAlgorithm.cpp
        LabelingSubProblem.cpp
        SubproModeler.cpp
        GreedyModeler.cpp
        CG_Algorithm.cpp
        ISUD_Algorithm.cpp
        LagrangianSolver.cpp
        BaseSolver.cpp
        BatchSolver.cpp
        AnytimeSolver.cpp
        OfflineSolver.cpp
        AssignmentSolver.cpp
        AssignmentSolver.h
        SolverEnv.cpp
        SolverEnv.h
)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})

# Non-solver dependencies that solvers itself needs (threading comes in via
# utilities headers; we keep it explicit so the link line is self-describing).
target_link_libraries(${LIB_NAME} PUBLIC Threads::Threads)

# Solver back-end libraries are linked only when the corresponding option is
# enabled. PRIVATE means consumers of `solvers` (e.g. realtime_DARP) need to
# list these themselves if they call them directly, which is already the case
# in src/CMakeLists.txt.
if(USE_CPLEX)
    target_link_libraries(${LIB_NAME} PRIVATE CplexSolver)
endif()

if(USE_GUROBI)
    target_link_libraries(${LIB_NAME} PRIVATE GurobiSolver)
endif()

set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)