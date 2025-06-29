set(LIB_NAME CplexSolver)
set(SOURCE_FILES
        MIPSolver.cpp
        ReducedProblem.cpp
        ComplementPro.cpp
        CplexModeler.cpp
        MIPMasterProblem.cpp
        DualAuxSolver.cpp
        CPLEXSubProblem.cpp
)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)