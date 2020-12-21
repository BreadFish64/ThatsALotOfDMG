find_package(Vulkan REQUIRED)

add_library(Core STATIC
    "bus.hpp"
    "bus.cpp" 
    "cartridge/cartridge.hpp"
    "cartridge/cartridge.cpp"
    "cartridge/mbc1.hpp"
    "cartridge/mbc1.cpp"
    "cartridge/specialized_cartridge.hpp"
    "cartridge/specialized_cartridge.cpp"
    "cpu/cpu.hpp"
    "cpu/cpu.cpp"
    "ppu/ppu.cpp"
    "ppu/ppu.hpp"
 "cpu/interpreter.hpp" "cpu/interpreter.cpp" "timer.hpp" "timer.cpp" "ppu/renderer.hpp" "ppu/renderer.cpp" "ppu/vulkan/vk_frontend.hpp" "ppu/vulkan/vk_instance.hpp"   "cartridge/bare.hpp" "cartridge/bare.cpp" "cartridge/mbc5.hpp" "cartridge/mbc5.cpp")

CGB_ADD_DEFINITIONS(Core)

target_link_libraries(Core PUBLIC Common Vulkan::Vulkan)