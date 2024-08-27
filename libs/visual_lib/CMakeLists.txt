file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(visual_lib ${HEADERS} ${SOURCES})
target_include_directories(visual_lib PUBLIC include)
target_link_libraries(visual_lib PRIVATE graph tapf_lib pibt ${OPENGL_LIBRARIES} glm glfw glad)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/../../apps")