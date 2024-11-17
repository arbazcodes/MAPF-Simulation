project(mapf_simulation)

find_package(OpenGL REQUIRED)

add_executable(pibt_engine pibt_engine.cpp)
add_executable(cbs_engine cbs_engine.cpp)
target_link_libraries(pibt_engine PRIVATE graph pibt tapf_lib visual_lib ${OPENGL_LIBRARIES} glm glad glfw)
target_link_libraries(cbs_engine PRIVATE cbs tapf_lib visual_lib ${OPENGL_LIBRARIES} glm glad glfw)