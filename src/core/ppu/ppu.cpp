#include "common/logger.hpp"
#include "core/bus.hpp"
#include "ppu.hpp"
#include "renderer.hpp"

namespace CGB::Core {
void PPU::FrameWrites::PushLCD(GADDR addr, u8 val, u64 timestamp) {
    u8 offset = static_cast<u8>(addr - 0xFF40);
    u32 dot = timestamp % FRAME_CYCLES;
    u8 scanline = static_cast<u8>(dot / SCANLINE_CYCLES);
    lcd_writes.emplace_back(LCDWrite{.index = offset, .val = val, .scanline = scanline});
}

void PPU::FrameWrites::PushOAM(GADDR addr, u8 val, u64 timestamp) {
    u32 dot = timestamp % FRAME_CYCLES;
    u8 scanline = static_cast<u8>(dot / SCANLINE_CYCLES);
    oam_dmas.emplace_back(
        OAMWrite{.offset = static_cast<u8>(addr), .val = val, .scanline = scanline});
}

void PPU::FrameWrites::PushVRAM(GADDR addr, u8 val, u64 timestamp) {
    u32 dot = timestamp % FRAME_CYCLES;
    u8 scanline = static_cast<u8>((dot + (SCANLINE_CYCLES - 80)) / SCANLINE_CYCLES);
    vram_writes.emplace_back(VRAMWrite{.addr = addr, .val = val, .scanline = scanline});
}

void PPU::FrameWrites::Close() {
    lcd_writes.emplace_back(LCDWrite{.scanline = static_cast<u8>(~0)});
    oam_dmas.emplace_back(OAMWrite{.scanline = static_cast<u8>(~0)});
    vram_writes.emplace_back(VRAMWrite{.scanline = static_cast<u8>(~0)});
}

void PPU::FrameWrites::Clear() {
    lcd_writes.clear();
    oam_dmas.clear();
    vram_writes.clear();
}

void PPU::VRAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp) {
    bus.Memory()[addr] = val;
    bus.GetPPU().frame_writes.PushVRAM(addr, val, timestamp);
}

u8 PPU::ReadOAM(GADDR addr, [[maybe_unused]] u64 timestamp) { return oam[addr & 0xFF]; }
void PPU::WriteOAM(GADDR addr, u8 val, [[maybe_unused]] u64 timestamp) {
    oam[addr & 0xFF] = val;
    bus->GetPPU().frame_writes.PushOAM(addr, val, timestamp);
}

u8 PPU::OAMReadHandler(Bus& bus, GADDR addr, u64 timestamp) {
    return bus.GetPPU().ReadOAM(addr, timestamp);
}

void PPU::OAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp) {
    bus.GetPPU().WriteOAM(addr, val, timestamp);
}

u8 PPU::LCDReadHandler([[maybe_unused]] Bus& bus, GADDR addr, u64 timestamp) {
    PPU& ppu = bus.GetPPU();
    switch (addr) {
    case 0xFF40: {
        return ppu.lcd.control;
    }
    case 0xFF41: {
        u8 stat = ppu.lcd.stat;
        u8 ly = (timestamp / SCANLINE_CYCLES) % SCANLINE_COUNT;
        u64 dot = timestamp % SCANLINE_CYCLES;
        stat |= (ly == ppu.lcd.lyc) << 2;
        u8 mode{};
        if (ly >= 144)
            mode = 1;
        else if (dot <= 80)
            mode = 2;
        else if (dot <= 280)
            mode = 3;
        else
            mode = 0;
        stat |= mode;
        return stat;
    }
    case 0xFF42: return ppu.lcd.scy;
    case 0xFF43: return ppu.lcd.scx;
    case 0xFF44: {
        u8 ly = (timestamp / SCANLINE_CYCLES) % SCANLINE_COUNT;
        return ly;
    }
    case 0xFF47: return ppu.lcd.bgp;
    case 0xFF48: return ppu.lcd.obp0;
    case 0xFF49: return ppu.lcd.obp1;
    default:
        LOG(Error, "Unimplemented LCD Register read {:#06X} on cycle {}", addr, timestamp);
        return static_cast<u8>(~0);
    };
}

void PPU::LCDWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp) {
    PPU& ppu = bus.GetPPU();
    switch (addr) {
    case 0xFF40: {
        ppu.lcd.control = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF41: {
        LOG(Warning, "Stat interrupts are unimplemented");
        ppu.lcd.stat = val & 0xF0;
    } break;
    case 0xFF42: {
        ppu.lcd.scy = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF43: {
        ppu.lcd.scx = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF45: {
        ppu.lcd.lyc = val;
        const u64 scheduled_cycle_in_frame = static_cast<u64>(ppu.lcd.lyc) * PPU::SCANLINE_CYCLES;
        const u64 current_cycle_in_frame = timestamp % PPU::FRAME_CYCLES;
        const u64 frame_start = timestamp - current_cycle_in_frame;
        const u64 frame_start_schedule_offset = scheduled_cycle_in_frame +
                (current_cycle_in_frame > scheduled_cycle_in_frame ? PPU::FRAME_CYCLES : 0);
        auto& cpu = bus.GetCPU();
        cpu.DescheduleEvent(Event::LCD_STAT_LYC_IS_LY);
        cpu.ScheduleEvent(
            frame_start + frame_start_schedule_offset,
            Event::LCD_STAT_LYC_IS_LY);
    } break;
    case 0xFF46: {
        auto src = static_cast<GADDR>(val) << 8;
        for (GADDR i{0}; i < ppu.oam.size(); ++i) {
            ppu.WriteOAM(0xFE00 + i, bus.Memory()[src + i], timestamp);
        }
        ppu.last_dma = timestamp;
    } break;
    case 0xFF47: {
        ppu.lcd.bgp = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF48: {
        ppu.lcd.obp0 = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF49: {
        ppu.lcd.obp1 = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF4A: {
        ppu.lcd.wy = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    case 0xFF4B: {
        ppu.lcd.wx = val;
        ppu.frame_writes.PushLCD(addr, val, timestamp);
    } break;
    default:
        // ppu.frame_writes.PushLCD(addr, val, timestamp);
        LOG(Error, "Unimplemented LCD Register write {:#06X} = {:#04X} on cycle {}", addr, val,
            timestamp);
    };
}

PPU::PPU(std::unique_ptr<Renderer> renderer) : renderer{std::move(renderer)} {}
PPU::~PPU() {}

void PPU::Install(Bus& _bus) {
    LOG(Info, "Installing PPU on bus");
    this->bus = &_bus;

    vram_backing = Common::VirtualMemory::MemoryBacking{Bus::PAGE_SIZE * 4};
    bus->GetAddressSpace().Split(0x8000, Bus::PAGE_SIZE * 2);
    vram =
        vram_backing.Map(0x0000, Bus::PAGE_SIZE * 2, Common::VirtualMemory::PROTECTION::READ_WRITE,
                         bus->GetAddressSpace(), 0x8000);
    vram_tag_backing = Common::VirtualMemory::MemoryBacking{Bus::PAGE_SIZE};
    bus->GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0x8000, Bus::PAGE_SIZE);
    bus->GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0x9000, Bus::PAGE_SIZE);
    vram_tags[0] =
        vram_tag_backing.Map(0, Bus::PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE,
                             bus->GetAddressSpace(), Bus::ADDRESS_SPACE + 0x8000);
    vram_tags[1] =
        vram_tag_backing.Map(0, Bus::PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE,
                             bus->GetAddressSpace(), Bus::ADDRESS_SPACE + 0x9000);
    auto VRAM_tag = bus->RegisterMemoryTag(nullptr, VRAMWriteHandler);
    std::ranges::fill(vram_tags[0].Span<MemoryTag>(), VRAM_tag);
    auto OAM_tag = bus->RegisterMemoryTag(OAMReadHandler, OAMWriteHandler);
    std::ranges::fill(bus->Tags().subspan<0xFE00, 0xA0>(), OAM_tag);
    auto LCD_tag = bus->RegisterMemoryTag(LCDReadHandler, LCDWriteHandler);
    for (u8 io = 0x40; io <= 0x4B; ++io) bus->AttachIOHandler(io, LCD_tag);
}

void PPU::VBlank(u64 timestamp) {
    // TODO: handle late VBlank event in queue
    bus->GetCPU().ScheduleEvent(timestamp - (timestamp % FRAME_CYCLES) + FRAME_CYCLES,
                                Event::VBlank);
    renderer->RecieveFrameWrites(std::move(frame_writes));
    frame_writes = {};
}

void PPU::LCD_STAT_LYC_IS_LY(u64 timestamp) {
    const u64 current_cycle_in_frame = timestamp % FRAME_CYCLES;
    const u64 scheduled_timestamp = timestamp - current_cycle_in_frame + FRAME_CYCLES +
                                    (static_cast<u64>(lcd.lyc) * SCANLINE_CYCLES);
    bus->GetCPU().ScheduleEvent(scheduled_timestamp, Event::LCD_STAT_LYC_IS_LY);
}

} // namespace CGB::Core