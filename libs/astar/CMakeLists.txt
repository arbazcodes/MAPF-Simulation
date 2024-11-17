file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(astar ${HEADERS} ${SOURCES})
target_include_directories(astar PUBLIC include)