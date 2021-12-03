set(LIB_NAME solvers)
set(SOURCE_FILES
        MIPSolver.cpp
        ISUDAlgorithm.cpp
        SubProblem.cpp
        CPLEXModeler.h
        ReducedProblem.cpp
        ComplementPro.cpp
        MasterModeler.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})