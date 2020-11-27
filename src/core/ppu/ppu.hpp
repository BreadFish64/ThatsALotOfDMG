#include "common/virtual_memory.hpp"

namespace CGB {

class Bus;

class PPU {
    struct TAG_TYPES {};

    std::array<u8, 0xA0> oam;

    Common::VirtualMemory::MemoryBacking vram_backing;
    Common::VirtualMemory::ReservedMappedSection vram;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> vram_tags;

    u8 ReadOAM(GADDR addr, u64 timestamp);
    void WriteOAM(GADDR addr, u8 val, u64 timestamp);

    static u8 OAMReadHandler(Bus& bus, GADDR addr, u64 timestamp);
    static void OAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp);

public:
    PPU();
    void Install(Bus& bus);
};

} // namespace CGB