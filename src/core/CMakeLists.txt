find_package(Vulkan REQUIRED)
find_package(xbyak CONFIG REQUIRED)

add_library(Core STATIC
    "bus.cpp"

    "cartridge/bare.cpp"
    "cartridge/cartridge.cpp"
    "cartridge/mbc1.cpp"
    "cartridge/mbc5.cpp"
    "cartridge/specialized_cartridge.cpp"

    "cpu/cpu.cpp"
    "cpu/interpreter.cpp"

    "ppu/vulkan/vk_instance.hpp"
    "ppu/ppu.cpp"
    "ppu/renderer.cpp"

    "timer.cpp"
)

CGB_ADD_DEFINITIONS(Core)

target_link_libraries(Core PUBLIC Common Vulkan::Vulkan xbyak::xbyak)