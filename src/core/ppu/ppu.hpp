#pragma once

#include "common/virtual_memory.hpp"

namespace CGB::Core {

class Bus;
class Renderer;

class PPU {
public:
    static constexpr u32 DOT_CLOCK = 1 << 22;
    static constexpr u32 SCANLINE_COUNT = 154;
    static constexpr u32 WIDTH = 160;
    static constexpr u32 HEIGHT = 144;
    static constexpr u32 FRAME_CYCLES = 70224;
    static constexpr u32 SCANLINE_CYCLES = FRAME_CYCLES / SCANLINE_COUNT;

    using OAM = std::array<u8, 0xA0>;

    struct FrameWrites {
        struct LCDWrite {
            u8 index;
            u8 val;
            u8 scanline;
        };
        struct OAM_DMA {
            u8 scanline;
            OAM data;
        };
        struct VRAMWrite {
            GADDR addr;
            u8 val;
            u8 scanline;
        };
        std::vector<LCDWrite> lcd_writes;
        std::vector<OAM_DMA> oam_dmas;
        std::vector<VRAMWrite> vram_writes;
        void PushLCD(GADDR addr, u8 val, u64 timestamp);
        void PushOAM(const OAM& oam, u64 timestamp);
        void PushVRAM(GADDR addr, u8 val, u64 timestamp);
        void Close();
    };

private:
    OAM oam;
    u8 lcdc;
    u8 lyc;

    FrameWrites frame_writes;

    Bus* bus;

    std::unique_ptr<Renderer> renderer;

    Common::VirtualMemory::MemoryBacking vram_backing;
    Common::VirtualMemory::ReservedMappedSection vram;
    Common::VirtualMemory::MemoryBacking vram_tag_backing;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> vram_tags;

    u8 ReadOAM(GADDR addr, u64 timestamp);
    void WriteOAM(GADDR addr, u8 val, u64 timestamp);

    static void VRAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp);

    static u8 OAMReadHandler(Bus& bus, GADDR addr, u64 timestamp);
    static void OAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp);

    static u8 LCDReadHandler(Bus& bus, GADDR addr, u64 timestamp);
    static void LCDWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp);

public:
    PPU(std::unique_ptr<Renderer> renderer);
    ~PPU();
    void Install(Bus& bus);
    void VBlank(u64 timestamp);
};

} // namespace CGB::Core