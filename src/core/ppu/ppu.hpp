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
        struct OAMWrite {
            u8 offset;
            u8 val;
            u8 scanline;
        };
        struct VRAMWrite {
            GADDR addr;
            u8 val;
            u8 scanline;
        };
        std::vector<LCDWrite> lcd_writes;
        std::vector<OAMWrite> oam_dmas;
        std::vector<VRAMWrite> vram_writes;
        void PushLCD(GADDR addr, u8 val, u64 timestamp);
        void PushOAM(GADDR addr, u8 val, u64 timestamp);
        void PushVRAM(GADDR addr, u8 val, u64 timestamp);
        void Close();
        void Clear();
    };

    struct LcdRegs {
        u8 control{0x91};
        u8 stat{};
        u8 scy{};
        u8 scx{};
        u8 ly{};
        u8 lyc{};
        u8 _dma{};
        u8 bgp{0xFC};
        u8 obp0{0xFF};
        u8 obp1{0xFF};
        u8 wy{};
        u8 wx{};

        [[nodiscard]] bool EnableLCD() const { return control & (1 << 7); }
        [[nodiscard]] bool UseUpperWindowTileMap() const { return control & (1 << 6); }
        [[nodiscard]] bool EnableWindow() const { return control & (1 << 5); }
        [[nodiscard]] bool UseLowerBGTileData() const { return control & (1 << 4); }
        [[nodiscard]] bool UseUpperBGTileMap() const { return control & (1 << 3); }
        [[nodiscard]] bool UseLargeObjects() const { return control & (1 << 2); }
        [[nodiscard]] bool EnableObjects() const { return control & (1 << 1); }
        [[nodiscard]] bool EnableBG() const { return control & (1 << 0); }
    };
    u64 last_dma{0};

private:
    OAM oam;
    LcdRegs lcd;

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

    const LcdRegs& GetLcdRegs() const { return lcd; }

    void Install(Bus& bus);
    void VBlank(u64 timestamp);
    void LCD_STAT_LYC_IS_LY(u64 timestamp);
};

} // namespace CGB::Core