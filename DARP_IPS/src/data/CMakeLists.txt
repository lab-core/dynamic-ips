set(LIB_NAME data)
set(SOURCE_FILES
        Vehicle.cpp
        Instance.cpp
        Request.cpp
        Graph.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})