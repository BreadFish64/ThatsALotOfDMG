#include "common/logger.hpp"
#include "core/bus.hpp"
#include "ppu.hpp"

namespace CGB {

u8 PPU::ReadOAM(GADDR addr, [[maybe_unused]] u64 timestamp) {
    LOG(Warning, "OAM access is not properly timed");
    return oam[addr & 0xFF];
}
void PPU::WriteOAM(GADDR addr, u8 val, [[maybe_unused]] u64 timestamp) {
    LOG(Warning, "OAM access is not properly timed");
    oam[addr & 0xFF] = val;
}

u8 PPU::OAMReadHandler(Bus& bus, GADDR addr, u64 timestamp) {
    return bus.GetPPU().ReadOAM(addr, timestamp);
}

void PPU::OAMWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp) {
    bus.GetPPU().WriteOAM(addr, val, timestamp);
}

PPU::PPU() {}

void PPU::Install(Bus& bus) {
    LOG(Info, "Installing PPU on bus");
    vram_backing = Common::VirtualMemory::MemoryBacking{Bus::PAGE_SIZE * 4};
    bus.GetAddressSpace().Split(0x8000, Bus::PAGE_SIZE * 2);
    vram =
        vram_backing.Map(0x0000, Bus::PAGE_SIZE * 2, Common::VirtualMemory::PROTECTION::READ_WRITE,
                         bus.GetAddressSpace(), 0x8000);
    bus.GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0x8000, Bus::PAGE_SIZE);
    bus.GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0x9000, Bus::PAGE_SIZE);
    vram_tags[0] = bus.MapPassthroughTag(0x8);
    vram_tags[1] = bus.MapPassthroughTag(0x9);
}

} // namespace CGB