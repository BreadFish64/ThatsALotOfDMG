#pragma once

#include "common/virtual_memory.hpp"
#include "specialized_cartridge.hpp"

namespace CGB::Core {

class Bus;
struct MemoryTag;

class MBC1 : public SpecializedCartridge {
    struct TAG_TYPES {
        static constexpr usize RAM_ENABLE = 0x0000, ROM_BANK_SWITCH = 0x1000,
                               RAM_BANK_SWITCH = 0x2000, BANKING_MODE_SELECT = 0x3000,
                               SIZE = 0x4000;
    };

    // TODO: handle MBC1m
    u8 ram_enable = 0x00;
    u8 rom_bank_number = 0x01;
    u8 ram_bank_number = 0x00;
    u8 banking_mode_select = 0x00;

    u8 rom_bank_count = 0;
    u8 rom_bank_mask = 0x1F;

    Common::VirtualMemory::MappedFile ram_file_handle;
    Common::VirtualMemory::MemoryBacking ram_backing;
    Common::VirtualMemory::ReservedMappedSection fixed_rom;
    Common::VirtualMemory::ReservedMappedSection switchable_rom;
    Common::VirtualMemory::ReservedMappedSection switchable_ram;

    Common::VirtualMemory::MemoryBacking tag_backing;
    std::array<Common::VirtualMemory::ReservedMappedSection, 8> rom_tags;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> ram_tags;

    std::filesystem::path ram_file_path;

    void RamEnable(Bus& bus, u8 enable);
    void RomBankNumber(Bus& bus, u8 bank);
    void RamBankNumber(Bus& bus, u8 bank);
    void BankingModeSelect(Bus& bus, u8 mode);

    static void RamEnableHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);
    static void RomBankNumberHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);
    static void RamBankNumberHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);
    static void BankingModeSelectHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);

public:
    explicit MBC1(CartridgeHeader&& unspecialized);
    ~MBC1();

    virtual void Install(Bus& bus) override;
};

} // namespace CGB::Core