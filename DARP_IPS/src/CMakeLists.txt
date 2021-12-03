add_subdirectory(data)
add_subdirectory(utilities)
add_subdirectory(solvers)

#set(SOURCE_FILES staticMain.cpp)
set(SOURCE_FILES dynamicMain.cpp)

# build the static library
add_library(${DARP_IPS_LIB} STATIC ${SOURCE_FILES})
target_link_libraries(${DARP_IPS_LIB} data utilities solvers)