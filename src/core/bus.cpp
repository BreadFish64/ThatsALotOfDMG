#include <ranges>

#include "bus.hpp"
#include "common/logger.hpp"
#include "core/cartridge/specialized_cartridge.hpp"
#include "cpu/cpu.hpp"
#include "ppu/ppu.hpp"
#include "timer.hpp"

namespace CGB::Core {

u8 Bus::ReadNop(Bus&, GADDR addr, u64 timestamp) {
    LOG(Trace, "Unmapped memory read {:#06X} on cycle {}", addr, timestamp);
    return static_cast<u8>(~0);
}
void Bus::WriteNop(Bus&, GADDR addr, u64 timestamp, u8 val) {
    LOG(Trace, "Unmapped memory write {:#06X} = {:#04X} on cycle {}", addr, val, timestamp);
}

Bus::Bus(std::unique_ptr<CartridgeHeader> _cartridge, CPU::MainCPU _cpu, std::unique_ptr<PPU> _ppu)
    : address_space{ADDRESS_SPACE * 2},
      cartridge{SpecializedCartridge::Make(std::move(*_cartridge))}, cpu{std::move(_cpu)},
      ppu{std::move(_ppu)}, timer{std::make_unique<Timer>()}, tag_backing{TAG_TYPES::SIZE} {
    address_space_loc = static_cast<void*>(address_space.Span().data());

    LOG(Info, "Installing WRAM on bus");
    auto echo_tag = RegisterMemoryTag(
        [](Bus& bus, GADDR addr, [[maybe_unused]] u64 timestamp) -> u8 {
            return bus.Memory()[addr - PAGE_SIZE * 2];
        },
        [](Bus& bus, GADDR addr, [[maybe_unused]] u64 timestamp, u8 val) {
            bus.Memory()[addr - PAGE_SIZE * 2] = val;
        });
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
            // TODO: echo RAM in 0xE000-0xEFFF
            std::ranges::fill(io_tags.first<0xE00>(), echo_tag);
            std::ranges::fill(io_tags.subspan<0xF00, 0x80>(),
                              MemoryTag{.read = true, .write = true, .handler = 0});
        }
    }

    for (u8 page = 0xC; page <= 0xE; ++page) {
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
    wram_backing = Common::VirtualMemory::MemoryBacking{PAGE_SIZE * 9};
    fixed_wram = wram_backing.Map(0x0000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE,
                                  address_space, 0xC000);
    switchable_wram = wram_backing.Map(
        0x1000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xD000);

    hram = wram_backing.Map(0x8000, PAGE_SIZE, Common::VirtualMemory::PROTECTION::READ_WRITE,
                            address_space, 0xF000);

    cartridge->Install(*this);
    cpu.Install(*this);
    ppu->Install(*this);
    timer->Install(*this);

    // TODO: Figure out something else to do with this serial stub
    // auto serial_tag =
    //    RegisterMemoryTag(ReadNop, []([[maybe_unused]] Bus& bus, [[maybe_unused]] GADDR addr,
    //                                  u8 val, [[maybe_unused]] u64 timestamp) {
    //        static std::ofstream serial_output{"serial_output.txt"};
    //        serial_output << static_cast<char>(val);
    //        serial_output.flush();
    //    });
    // AttachIOHandler(0x01, serial_tag);
    auto input_tag =
        RegisterMemoryTag([]([[maybe_unused]] Bus& bus, [[maybe_unused]] GADDR addr,
                             [[maybe_unused]] u64 timestamp) -> u8 { return static_cast<u8>(~0); },
                          WriteNop);
    AttachIOHandler(0x00, input_tag);

    LOG(Info, "All hardware installed onto bus");
}

Bus::~Bus() {}

CartridgeHeader& Bus::GetCartridge() { return static_cast<CartridgeHeader&>(*cartridge); }

const CartridgeHeader& Bus::GetCartridge() const {
    return static_cast<const CartridgeHeader&>(*cartridge);
}

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
    switchable_wram =
        wram_backing.Map(index * PAGE_SIZE, PAGE_SIZE,
                         Common::VirtualMemory::PROTECTION::READ_WRITE, address_space, 0xD000);
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

} // namespace CGB::Core
