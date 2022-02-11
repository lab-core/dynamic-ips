set(LIB_NAME solvers)
set(SOURCE_FILES
        MIPSolver.cpp
        ISUDAlgorithm.cpp
        CPLEXSubProblem.cpp
        CPLEXModeler.h
        ReducedProblem.cpp
        ComplementPro.cpp
        MasterModeler.cpp
        LabelingSubProblem.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})