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
 "cpu/interpreter.hpp" "cpu/interpreter.cpp")

CGB_ADD_DEFINITIONS(Core)

target_link_libraries(Core PUBLIC Common)