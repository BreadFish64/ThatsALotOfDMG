#include <ranges>

#include "bus.hpp"
#include "common/logger.hpp"
#include "core/cartridge/specialized_cartridge.hpp"
#include "cpu/cpu.hpp"
#include "ppu/ppu.hpp"

namespace CGB {

u8 Bus::ReadNop(Bus&, GADDR addr, u64 timestamp) {
    LOG(Warning, "Unmapped memory read {:#06X} on cycle {}", addr, timestamp);
    return ~0;
}
void Bus::WriteNop(Bus&, GADDR addr, u8 val, u64 timestamp) {
    LOG(Warning, "Unmapped memory write {:#06X} = {:#04X} on cycle {}", addr, val,
        timestamp);
}

Bus::Bus(std::unique_ptr<Cartridge> _cartridge, CPU::BaseCPU _cpu, std::unique_ptr<PPU> _ppu)
    : address_space{ADDRESS_SPACE * 2}, cartridge{SpecializedCartridge::Make(
                                            std::move(*_cartridge))},
      cpu{std::move(_cpu)}, ppu{std::move(_ppu)}, tag_backing{TAG_TYPES::SIZE} {

    LOG(Info, "Installing WRAM on bus");
    {
        auto tag_data = tag_backing.Map(0, 0, Common::VirtualMemory::PROTECTION::READ_WRITE);
        // Generate tag data
        std::ranges::fill(tag_data.Span<MemoryTag>().subspan<TAG_TYPES::NOP, Bus::PAGE_SIZE>(),
                          MemoryTag{.read = true, .write = true, .handler = 0});
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::PASS_THROUGH, Bus::PAGE_SIZE>(),
            MemoryTag{.read = false, .write = false, .handler = 0});
        {
            auto io_tags = tag_data.Span<MemoryTag>().subspan<TAG_TYPES::IO, Bus::PAGE_SIZE>();
            std::ranges::fill(io_tags.first<0xE00>(),
                              MemoryTag{.read = false, .write = false, .handler = 0});
            std::ranges::fill(io_tags.last<0x100>(),
                              MemoryTag{.read = true, .write = true, .handler = 0});
        }
    }

    for (usize page = 0xC; page <= 0xE; ++page) {
        address_space.Split(ADDRESS_SPACE + page * PAGE_SIZE, PAGE_SIZE);
        wram_tags[page - 0xC] = MapPassthroughTag(page);
    }
    wram_tags[0x3] =
        tag_backing.Map(TAG_TYPES::IO, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE,
                        address_space, ADDRESS_SPACE + 0xF000);

    // Split the fixed and switchable WRAM into their own placeholders
    for (usize offset = 0xC000; offset < 0x10000; offset += PAGE_SIZE)
        address_space.Split(offset, PAGE_SIZE);

    // Create backing memory for WRAM
    wram_backing = Common::VirtualMemory::MemoryBacking{PAGE_SIZE * 8};
    fixed_wram[0] = wram_backing.Map(
        0x0000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xC000);
    fixed_wram[1] = wram_backing.Map(
        0x1000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xE000);
    switchable_wram[0] = wram_backing.Map(
        0x1000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xD000);
    switchable_wram[1] = wram_backing.Map(
        0x1000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xF000);

    cartridge->Install(*this);
    cpu.Install(*this);
    ppu->Install(*this);
    LOG(Info, "All hardware installed onto bus");
    cpu.Run();
}

Bus::~Bus() {}

Cartridge& Bus::GetCartridge() { return static_cast<Cartridge&>(*cartridge); }

const Cartridge& Bus::GetCartridge() const { return static_cast<const Cartridge&>(*cartridge); }

Common::VirtualMemory::ReservedMappedSection Bus::MapNopTag(u8 page) {
    return tag_backing.Map(TAG_TYPES::NOP, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE,
                           address_space, ADDRESS_SPACE + page * PAGE_SIZE);
}

Common::VirtualMemory::ReservedMappedSection Bus::MapPassthroughTag(u8 page) {
    return tag_backing.Map(TAG_TYPES::PASS_THROUGH, PAGE_SIZE,
                           Common::VirtualMemory::PROTECTION::READ_WRITE, address_space,
                           ADDRESS_SPACE + page * PAGE_SIZE);
}

void Bus::SwitchWRAMBank(u8 index) {
    switchable_wram = {};
    switchable_wram[0] =
        wram_backing.Map(index * PAGE_SIZE, PAGE_SIZE,
                         Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xD000);
    switchable_wram[1] =
        wram_backing.Map(index * PAGE_SIZE, PAGE_SIZE,
                         Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xF000);
}

MemoryTag Bus::RegisterMemoryTag(ReadHandler read_handler, WriteHandler write_handler) {
    if (memory_handler_count == read_handlers.size()) {
        LOG(Critical, "Too many memory handlers");
        return {};
    }
    read_handlers[memory_handler_count] = read_handler;
    write_handlers[memory_handler_count] = write_handler;
    return {.read = read_handler != nullptr,
            .write = write_handler != nullptr,
            .handler = static_cast<u8>(memory_handler_count++)};
}

} // namespace CGB
