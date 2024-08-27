file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(tapf_lib ${HEADERS} ${SOURCES})
target_include_directories(tapf_lib PUBLIC include)