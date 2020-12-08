#pragma once

#include "common/virtual_memory.hpp"
#include "specialized_cartridge.hpp"

namespace CGB::Core {

class Bus;
struct MemoryTag;

class BARE : public SpecializedCartridge {

    Common::VirtualMemory::MappedFile ram_file_handle;
    Common::VirtualMemory::MemoryBacking ram_backing;
    Common::VirtualMemory::ReservedMappedSection fixed_rom;
    Common::VirtualMemory::ReservedMappedSection fixed_ram;

    Common::VirtualMemory::MemoryBacking tag_backing;
    std::array<Common::VirtualMemory::ReservedMappedSection, 8> rom_tags;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> ram_tags;

    std::filesystem::path ram_file_path;

public:
    explicit BARE(CartridgeHeader&& unspecialized);
    ~BARE();

    virtual void Install(Bus& bus) override;
};

} // namespace CGB::Core