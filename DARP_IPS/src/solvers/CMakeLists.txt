set(LIB_NAME solvers)
set(SOURCE_FILES
        MasterAlgorithm.cpp
        LabelingSubProblem.cpp
        SubproModeler.cpp
        GreedyModeler.cpp
        Solver.cpp
        CG_Algorithm.cpp
        ISUD_Algorithm.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)