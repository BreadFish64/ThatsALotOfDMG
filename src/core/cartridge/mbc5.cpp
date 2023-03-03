#include <filesystem>
#include <fstream>
#include <ranges>

#include "common/logger.hpp"
#include "core/bus.hpp"
#include "core/cartridge/mbc5.hpp"

namespace CGB::Core {

MBC5::MBC5(CartridgeHeader&& unspecialized) : SpecializedCartridge{std::move(unspecialized)} {
    if (CartridgeType() != CARTRIDGE_TYPE::MBC5) {
        if (CartridgeType() == CARTRIDGE_TYPE::MBC5_RAM_BATTERY) {
            ram_file_path = rom_file_path.replace_extension(".sav");
            // Create ram file if it does not already exist
            // std::ifstream{ram_file_path};

            if (std::filesystem::file_size(ram_file_path) != RamSize()) {
                LOG(Warning, "Resizing cartridge RAM from {}KiB to {}KiB",
                    std::filesystem::file_size(ram_file_path), RamSize());
                std::filesystem::resize_file(ram_file_path, RamSize());
            }
            ram_file_handle = Common::VirtualMemory::MappedFile{
                ram_file_path, Common::VirtualMemory::PROTECTION::READ_WRITE,
                Common::VirtualMemory::PROTECTION::READ};
        }
        ram_backing = Common::VirtualMemory::MemoryBacking{
            ram_file_handle, Common::VirtualMemory::PROTECTION::READ_WRITE,
            Common::VirtualMemory::PROTECTION::READ_WRITE, RamSize()};
    }
}

MBC5::~MBC5() {}

void MBC5::Install(Bus& bus) {
    LOG(Info, "Installing MBC5 cartridge on bus");

    bus.GetAddressSpace().Split(0, Bus::PAGE_SIZE * 4);
    bus.GetAddressSpace().Split(0x4000, Bus::PAGE_SIZE * 4);
    fixed_rom =
        rom_backing.Map(0, Bus::PAGE_SIZE * 4, Common::VirtualMemory::PROTECTION::READ_WRITE,
                        bus.GetAddressSpace(), 0x0000);
    switchable_rom =
        rom_backing.Map(0x4000, Bus::PAGE_SIZE * 4, Common::VirtualMemory::PROTECTION::READ_WRITE,
                        bus.GetAddressSpace(), 0x4000);

    // Create tag mapping
    tag_backing = Common::VirtualMemory::MemoryBacking{TAG_TYPES::SIZE};
    {
        auto tag_data = tag_backing.Map(0, 0, Common::VirtualMemory::PROTECTION::READ_WRITE);

        // Generate tag data
        auto ram_enable_tag = bus.RegisterMemoryTag(nullptr, RamEnableHandler);
        auto rom_bank_number_tag = bus.RegisterMemoryTag(nullptr, RomBankNumberHandler);
        auto ram_bank_number_tag = bus.RegisterMemoryTag(nullptr, RamBankNumberHandler);
        auto nop_tag = MemoryTag{.read = false, .write = true, .handler = 0};
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::RAM_ENABLE, Bus::PAGE_SIZE>(),
            ram_enable_tag);
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::ROM_BANK_SWITCH, Bus::PAGE_SIZE>(),
            rom_bank_number_tag);
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::RAM_BANK_SWITCH, Bus::PAGE_SIZE>(),
            ram_bank_number_tag);
        std::ranges::fill(tag_data.Span<MemoryTag>().subspan<TAG_TYPES::NOP, Bus::PAGE_SIZE>(),
                          nop_tag);
    }

    // Map tag data for ROM sections
    for (usize i{0}; i < 4; ++i) {
        usize offset = Bus::ADDRESS_SPACE + Bus::PAGE_SIZE * i * 2;
        bus.GetAddressSpace().Split(offset, Bus::PAGE_SIZE);
        bus.GetAddressSpace().Split(offset + Bus::PAGE_SIZE, Bus::PAGE_SIZE);
        rom_tags[i * 2] = tag_backing.Map(i * Bus::PAGE_SIZE, Bus::PAGE_SIZE,
                                          Common::VirtualMemory::PROTECTION::READ_WRITE,
                                          bus.GetAddressSpace(), offset);
        rom_tags[i * 2 + 1] = tag_backing.Map(i * Bus::PAGE_SIZE, Bus::PAGE_SIZE,
                                              Common::VirtualMemory::PROTECTION::READ_WRITE,
                                              bus.GetAddressSpace(), offset + Bus::PAGE_SIZE);
    }

    // Map tag data for RAM sections
    bus.GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0xA000, Bus::PAGE_SIZE);
    bus.GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0xB000, Bus::PAGE_SIZE);
    ram_tags[0] = bus.MapNopTag(0xA);
    ram_tags[1] = bus.MapNopTag(0xB);
}

void MBC5::RamEnableHandler(Bus& bus, [[maybe_unused]] GADDR addr, u8 val,
                            [[maybe_unused]] u64 timestamp) {
    auto& cartridge = static_cast<MBC5&>(bus.GetCartridge());
    bool old_enabled = (cartridge.ram_enable & 0x0F) == 0x0A;
    bool new_enabled = (val & 0x0F) == 0x0A;
    cartridge.ram_enable = val;

    if (new_enabled) {
        if (old_enabled) return;
        LOG(Critical, "MBC5 RAM enable is unimplemented");
        return;
    } else {
        if (!old_enabled) return;

        return;
    }
}

void MBC5::RomBankNumberHandler(Bus& bus, GADDR addr, u8 val,
                                [[maybe_unused]] u64 timestamp) {
    auto& cartridge = static_cast<MBC5&>(bus.GetCartridge());
    if (addr >= 0x3000) {
        cartridge.bank_number_high = val;
    } else {
        cartridge.bank_number_low = val;
    }
    unsigned bank = static_cast<unsigned>(cartridge.bank_number_high) << 8 | cartridge.bank_number_low;
    cartridge.switchable_rom = {};
    cartridge.switchable_rom = cartridge.rom_backing.Map(
        bank * Bus::PAGE_SIZE * 4, Bus::PAGE_SIZE * 4,
        Common::VirtualMemory::PROTECTION::READ_WRITE, bus.GetAddressSpace(), 0x4000);
}

void MBC5::RamBankNumberHandler(Bus& bus, [[maybe_unused]] GADDR addr, u8 val,
                                [[maybe_unused]] u64 timestamp) {
    auto& cartridge = static_cast<MBC5&>(bus.GetCartridge());
    cartridge.ram_bank_number = val;
    LOG(Critical, "MBC5 RAM bank switch is unimplemented");
}

} // namespace CGB::Core
