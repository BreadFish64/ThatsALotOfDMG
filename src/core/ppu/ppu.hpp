#include "common/virtual_memory.hpp"

namespace CGB::Core {

class Bus;

class PPU {
    struct TAG_TYPES {};

    std::array<u8, 0xA0> oam;

    Common::VirtualMemory::MemoryBacking vram_backing;
    Common::VirtualMemory::ReservedMappedSection vram;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> vram_tags;

    static constexpr u32 DOT_CLOCK = 1 << 22;
    static constexpr u32 SCANLINE_COUNT = 154;
    static constexpr u32 SCANLINE_CYCLES = 70224;

    u8 ReadOAM(GADDR addr, u64 timestamp);
    void WriteOAM(GADDR addr, u8 val, u64 timestamp);

    static u8 OAMReadHandler(Bus& bus, GADDR addr, u64 timestamp);
    static void OAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp);

    static u8 LCDReadHandler(Bus& bus, GADDR addr, u64 timestamp);
    static void LCDWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp);

public:
    PPU();
    void Install(Bus& bus);
};

} // namespace CGB::Core