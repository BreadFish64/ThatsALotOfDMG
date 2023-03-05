#pragma once

#include "common/virtual_memory.hpp"
#include "specialized_cartridge.hpp"

namespace CGB::Core {

class Bus;
struct MemoryTag;

class MBC5 : public SpecializedCartridge {
    struct TAG_TYPES {
        static constexpr usize RAM_ENABLE = 0x0000, ROM_BANK_SWITCH = 0x1000, RAM_BANK_SWITCH = 0x2000,
                               NOP = 0x3000, SIZE = 0x4000;
    };

    u8 ram_enable = 0x00;
    u8 bank_number_low = 0x01;
    u8 bank_number_high = 0x00;
    u8 ram_bank_number = 0x00;

    Common::VirtualMemory::MappedFile ram_file_handle;
    Common::VirtualMemory::MemoryBacking ram_backing;
    Common::VirtualMemory::ReservedMappedSection fixed_rom;
    Common::VirtualMemory::ReservedMappedSection switchable_rom;
    Common::VirtualMemory::ReservedMappedSection switchable_ram;

    Common::VirtualMemory::MemoryBacking tag_backing;
    std::array<Common::VirtualMemory::ReservedMappedSection, 8> rom_tags;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> ram_tags;

    std::filesystem::path ram_file_path;

    static void RamEnableHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);
    static void RomBankNumberHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);
    static void RamBankNumberHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);

public:
    explicit MBC5(CartridgeHeader&& unspecialized);
    ~MBC5();

    virtual void Install(Bus& bus) override;
};

} // namespace CGB::Core