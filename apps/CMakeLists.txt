project(simulation)

find_package(OpenGL REQUIRED)

add_executable(simulation_engine program.cpp)
target_link_libraries(simulation_engine PRIVATE graph pibt tapf_lib visual_lib ${OPENGL_LIBRARIES} glm glad glfw)