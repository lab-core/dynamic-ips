set(LIB_NAME GurobiSolver)
set(SOURCE_FILES
    GurobiModeler.cpp
    RP_Gurobi.cpp
    RP_Gurobi.h
    MP_Gurobi.cpp
    MP_Gurobi.h
        CP_Gurobi.cpp
        CP_Gurobi.h
)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)