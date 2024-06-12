set(LIB_NAME solver)
set(SOURCE_FILES
        MasterAlgorithm.h
        MasterAlgorithm.cpp
        LabelingSubProblem.h
        LabelingSubProblem.cpp
        SubproModeler.h
        SubproModeler.cpp
        solver.h
        solver.cpp
        MasterModeler.h
        MasterModeler.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)