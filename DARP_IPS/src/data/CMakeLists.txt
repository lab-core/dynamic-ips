set(LIB_NAME data)
set(SOURCE_FILES
        Vehicle.cpp
        Instance.cpp
        Request.cpp
        Graph.cpp
        Route.cpp
        Label.cpp
        Greedy.cpp
        Parameters.cpp
        SolutionMetrics.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)