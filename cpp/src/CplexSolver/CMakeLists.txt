if(USE_CPLEX)
    set(LIB_NAME CplexSolver)
    set(SOURCE_FILES
            MIPSolver_Cplex.cpp
            RP_Cplex.cpp
            CP_Cplex.cpp
            CplexModeler.cpp
            MP_Cplex.cpp
            SubProblem_Cplex.cpp
    )

    add_library(${LIB_NAME} STATIC ${SOURCE_FILES})

    # Only the CPLEX runtime libraries are needed here.
    target_link_libraries(${LIB_NAME} PUBLIC
            cplex-library
            cplex-concert
            ilocplex)

    set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
