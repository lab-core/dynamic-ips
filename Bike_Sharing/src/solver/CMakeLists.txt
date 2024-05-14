set(LIB_NAME solver)
set(SOURCE_FILES
        MasterAlgorithm.cpp
        LabelingSubProblem.cpp
        SubproModeler.cpp
        solver.cpp
        MasterModeler.cpp
        MasterModeler.h)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)