find_package(SDL2 REQUIRED)

add_executable(CGB
    "main.cpp"
    "vk_renderer_frontend.cpp"
)

CGB_ADD_DEFINITIONS(CGB)

target_link_libraries(CGB PRIVATE SDL2::SDL2)
target_link_libraries(CGB PRIVATE Core)
