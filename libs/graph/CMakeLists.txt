file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(graph ${HEADERS} ${SOURCES})
target_include_directories(graph PUBLIC include)