if(USE_GUROBI)
    set(LIB_NAME GurobiSolver)
    set(SOURCE_FILES
        GurobiModeler.cpp
        RP_Gurobi.cpp
        MP_Gurobi.cpp
        CPModeler.cpp
            SubProblem_Gurobi.cpp
            MIPSolver_Gurobi.cpp
    )

    add_library(${LIB_NAME} STATIC ${SOURCE_FILES})

    # Only the Gurobi C++ runtime library is needed here.
    target_link_libraries(${LIB_NAME} PUBLIC gurobi::gurobi_c++)

    set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
