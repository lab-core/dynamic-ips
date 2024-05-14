set(LIB_NAME data)
set(SOURCE_FILES
        Vehicle.cpp
        Instance.cpp
        Task.cpp
        Graph.cpp
        Route.cpp
        Parameters.cpp
        Stop.cpp
        Station.cpp
        Label.cpp
        Label.h)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)