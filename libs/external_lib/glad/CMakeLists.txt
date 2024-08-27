file(GLOB_RECURSE HEADERS "includes/*.h" "includes/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")
add_library(glad ${HEADERS} ${SOURCES})
target_include_directories(glad PUBLIC includes)