#include <filesystem>
#include <fstream>
#include <ranges>

#include "common/logger.hpp"
#include "core/bus.hpp"
#include "core/cartridge/mbc1.hpp"

namespace CGB {

MBC1::MBC1(Cartridge&& unspecialized) : SpecializedCartridge{std::move(unspecialized)} {
    if (CartridgeType() != CARTRIDGE_TYPE::MBC1) {
        if (CartridgeType() == CARTRIDGE_TYPE::MBC1_RAM_BATTERY) {
            ram_file_path = rom_file_path.replace_extension(".sav");
            // Create ram file if it does not already exist
            std::ifstream{ram_file_path};

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

MBC1::~MBC1() {}

void MBC1::Install(Bus& bus) {
    LOG(Info, "Installing MBC1 cartridge on bus");

    bus.GetAddressSpace().Split(0, Bus::PAGE_SIZE * 4);
    bus.GetAddressSpace().Split(0x4000, Bus::PAGE_SIZE * 4);
    fixed_rom = rom_backing.Map(0, Bus::PAGE_SIZE * 4, Common::VirtualMemory::PROTECTION::READ_WRITE,
                                bus.GetAddressSpace(), 0x0000);
    switchable_rom =
        rom_backing.Map(0x4000, Bus::PAGE_SIZE * 4, Common::VirtualMemory::PROTECTION::READ_WRITE,
                        bus.GetAddressSpace(), 0x4000);

    // Create tag mapping
    tag_backing = Common::VirtualMemory::MemoryBacking{TAG_TYPES::SIZE};
    {
        auto tag_data =
            tag_backing.Map(0, 0, Common::Windows::VirtualMemory::PROTECTION::READ_WRITE);

        // Generate tag data
        auto ram_enable_tag = bus.RegisterMemoryTag(nullptr, RamEnableHandler);
        auto rom_bank_number_tag = bus.RegisterMemoryTag(nullptr, RomBankNumberHandler);
        auto ram_bank_number_tag = bus.RegisterMemoryTag(nullptr, RamBankNumberHandler);
        auto banking_mode_select_tag = bus.RegisterMemoryTag(nullptr, BankingModeSelectHandler);
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::RAM_ENABLE, Bus::PAGE_SIZE>(),
            ram_enable_tag);
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::ROM_BANK_SWITCH, Bus::PAGE_SIZE>(),
            rom_bank_number_tag);
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::RAM_BANK_SWITCH, Bus::PAGE_SIZE>(),
            ram_bank_number_tag);
        std::ranges::fill(
            tag_data.Span<MemoryTag>().subspan<TAG_TYPES::BANKING_MODE_SELECT, Bus::PAGE_SIZE>(),
            banking_mode_select_tag);
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

void MBC1::RamEnable([[maybe_unused]] Bus& bus, u8 enable) {
    bool old_enabled = (enable & 0x0F) == 0x0A;
    bool new_enabled = (enable & 0x0F) == 0x0A;
    ram_enable = enable;

    if (new_enabled) {
        if (old_enabled) return;
        LOG(Critical, "MBC1 RAM enable is unimplemented");
        return;
    } else {
        if (!old_enabled) return;

        return;
    }
    UNREACHABLE();
};

void MBC1::RomBankNumber([[maybe_unused]] Bus& bus, u8 bank) {
    rom_bank_number = bank;
    LOG(Critical, "MBC1 ROM bank switch is unimplemented");
};

void MBC1::RamBankNumber([[maybe_unused]] Bus& bus, u8 bank) {
    ram_bank_number = bank;
    LOG(Critical, "MBC1 RAM bank switch is unimplemented");
};

void MBC1::BankingModeSelect([[maybe_unused]] Bus& bus, u8 mode) {
    banking_mode_select = mode;
    LOG(Critical, "MBC1 banking mode select is unimplemented");
};

void MBC1::RamEnableHandler(Bus& bus, [[maybe_unused]] GADDR addr, u8 val,
                            [[maybe_unused]] u64 timestamp) {
    static_cast<MBC1&>(bus.GetCartridge()).RamEnable(bus, val);
}

void MBC1::RomBankNumberHandler(Bus& bus, [[maybe_unused]] GADDR addr, u8 val,
                                [[maybe_unused]] u64 timestamp) {
    static_cast<MBC1&>(bus.GetCartridge()).RomBankNumber(bus, val);
}

void MBC1::RamBankNumberHandler(Bus& bus, [[maybe_unused]] GADDR addr, u8 val,
                                [[maybe_unused]] u64 timestamp) {
    static_cast<MBC1&>(bus.GetCartridge()).RamBankNumber(bus, val);
}

void MBC1::BankingModeSelectHandler(Bus& bus, [[maybe_unused]] GADDR addr, u8 val,
                                    [[maybe_unused]] u64 timestamp) {
    static_cast<MBC1&>(bus.GetCartridge()).BankingModeSelect(bus, val);
}

} // namespace CGB
