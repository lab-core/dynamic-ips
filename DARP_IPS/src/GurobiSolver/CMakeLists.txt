set(LIB_NAME GurobiSolver)
set(SOURCE_FILES
    GurobiModeler.cpp
    RP_Gurobi.cpp
    MP_Gurobi.cpp
    CP_Gurobi.cpp
    CP_Reduced.cpp
    CPModeler.cpp
)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)