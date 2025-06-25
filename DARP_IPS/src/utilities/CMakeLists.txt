set(LIB_NAME utilities)
set(SOURCE_FILES
        InputPaths.cpp
        MyTools.cpp
        ReadWrite.cpp
        Tools.cpp
        ConfigParser.cpp
        Types.h)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${LIB_NAME} ${LIBS})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)