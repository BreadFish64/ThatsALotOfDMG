#include <filesystem>

#include "cartridge.hpp"
#include "common/logger.hpp"

namespace CGB {

Cartridge::Cartridge(std::filesystem::path _path) : rom_file_path{std::move(_path)} {
    LOG(Info, "Opening ROM file {}", rom_file_path);
    LOG(Trace, "ROM file size {:#X}", std::filesystem::file_size(rom_file_path));
    rom_file_handle = Common::VirtualMemory::MappedFile{
        rom_file_path, Common::VirtualMemory::PROTECTION::READ_WRITE,
        Common::VirtualMemory::PROTECTION::READ};
    rom_backing = Common::VirtualMemory::MemoryBacking{
        rom_file_handle, Common::VirtualMemory::PROTECTION::READ_WRITE,
        Common::VirtualMemory::PROTECTION::READ_WRITE};
    rom_data = rom_backing.Map(0, 0, Common::VirtualMemory::PROTECTION::READ_WRITE);
}

Cartridge::~Cartridge() {}

u8 Cartridge::RomBanks() const {
    if (RomSizeCode() <= 0x08) return 2 << RomSizeCode();
    if (RomSizeCode() >= 0x52 && RomSizeCode() <= 0x54) return 64 + (2 << (RomSizeCode() - 50));
    LOG(Error, "Unknown Rom Size Code {}", RomSizeCode());
    return 0;
}

usize Cartridge::RamSize() const {
    // TODO: add exception for MBC2
    if (RamSizeCode() == 0) return 0;
    if (RamSizeCode() <= 4) return (2 * 1024) << ((RamSizeCode() - 1) * 2);
    if (RamSizeCode() == 5) return 64 * 1024;
    LOG(Error, "Unknown Ram Size Code {}", RamSizeCode());
    return 0;
}

u8 Cartridge::HashHeader() const {
    u8 x{0};
    for (auto byte : rom_data.Span().subspan<0x134, 0x14D - 0x134>()) x -= byte + 1;
    return x;
}

u16 Cartridge::HashROM() const {
    u16 x{0};
    for (auto byte : rom_data.Span().first<0x14E>()) x += byte;
    for (auto byte : rom_data.Span().subspan<0x150>()) x += byte;
    return x;
}

} // namespace CGB
