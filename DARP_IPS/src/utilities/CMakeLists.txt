set(LIB_NAME utilities)
set(SOURCE_FILES
        InputPaths.cpp
        MyTools.cpp
        ReadWrite.cpp
        Tools.cpp)

add_library(${LIB_NAME} STATIC ${SOURCE_FILES})