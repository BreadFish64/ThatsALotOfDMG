#include <filesystem>
#include <fstream>
#include <ranges>

#include "common/logger.hpp"
#include "core/bus.hpp"
#include "core/cartridge/bare.hpp"

namespace CGB::Core {

BARE::BARE(CartridgeHeader&& unspecialized) : SpecializedCartridge{std::move(unspecialized)} {
    if (CartridgeType() != CARTRIDGE_TYPE::ROM_ONLY) {
        if (CartridgeType() == CARTRIDGE_TYPE::ROM_RAM_BATTERY) {
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

BARE::~BARE() {}

void BARE::Install(Bus& bus) {
    LOG(Info, "Installing bare cartridge on bus");

    bus.GetAddressSpace().Split(0, Bus::PAGE_SIZE * 8);
    fixed_rom =
        rom_backing.Map(0, Bus::PAGE_SIZE * 8, Common::VirtualMemory::PROTECTION::READ_WRITE,
                        bus.GetAddressSpace(), 0x0000);

    // Create tag mapping
    tag_backing = Common::VirtualMemory::MemoryBacking{Bus::PAGE_SIZE};
    {
        auto tag_data = tag_backing.Map(0, 0, Common::VirtualMemory::PROTECTION::READ_WRITE);

        std::ranges::fill(tag_data.Span<MemoryTag>(),
                          MemoryTag{.read = false, .write = true, .handler = 0});
    }

    // Map tag data for ROM sections
    for (usize i{0}; i < 8; ++i) {
        usize offset = Bus::ADDRESS_SPACE + Bus::PAGE_SIZE * i;
        bus.GetAddressSpace().Split(offset, Bus::PAGE_SIZE);
        rom_tags[i] = tag_backing.Map(0, Bus::PAGE_SIZE,
                                      Common::VirtualMemory::PROTECTION::READ_WRITE,
                                      bus.GetAddressSpace(), offset);
    }

    // Map tag data for RAM sections
    bus.GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0xA000, Bus::PAGE_SIZE);
    bus.GetAddressSpace().Split(Bus::ADDRESS_SPACE + 0xB000, Bus::PAGE_SIZE);
    ram_tags[0] = bus.MapNopTag(0xA);
    ram_tags[1] = bus.MapNopTag(0xB);
}

} // namespace CGB::Core
