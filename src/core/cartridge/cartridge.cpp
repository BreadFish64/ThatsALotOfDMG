#include <filesystem>

#include "cartridge.hpp"
#include "common/logger.hpp"

namespace CGB::Core {

CartridgeHeader::CartridgeHeader(std::filesystem::path _path) : rom_file_path{std::move(_path)} {
    LOG(Info, "Opening ROM file {}", rom_file_path.string());
    LOG(Trace, "ROM file size {:#X}", std::filesystem::file_size(rom_file_path));
    rom_file_handle = Common::VirtualMemory::MappedFile{
        rom_file_path, Common::VirtualMemory::PROTECTION::READ_WRITE,
        Common::VirtualMemory::PROTECTION::READ_WRITE};
    rom_backing = Common::VirtualMemory::MemoryBacking{
        rom_file_handle, Common::VirtualMemory::PROTECTION::READ_WRITE,
        Common::VirtualMemory::PROTECTION::READ_WRITE};
    rom_data = rom_backing.Map(0, 0, Common::VirtualMemory::PROTECTION::READ_WRITE);
}

CartridgeHeader::~CartridgeHeader() {}

u8 CartridgeHeader::RomBanks() const {
    if (RomSizeCode() <= 0x08) return static_cast<u8>(2 << RomSizeCode());
    if (RomSizeCode() >= 0x52 && RomSizeCode() <= 0x54) return static_cast<u8>(64 + (2 << (RomSizeCode() - 50)));
    LOG(Error, "Unknown Rom Size Code {}", RomSizeCode());
    return 0;
}

usize CartridgeHeader::RamSize() const {
    // TODO: add exception for MBC2
    if (RamSizeCode() == 0) return 0;
    if (RamSizeCode() <= 4) return static_cast<usize>(2 * 1024) << ((RamSizeCode() - 1) * 2);
    if (RamSizeCode() == 5) return 64 * 1024;
    LOG(Error, "Unknown Ram Size Code {}", RamSizeCode());
    return 0;
}

u8 CartridgeHeader::HashHeader() const {
    u8 x{0};
    for (auto byte : rom_data.Span().subspan<0x134, 0x14D - 0x134>()) x -= byte + 1;
    return x;
}

u16 CartridgeHeader::HashROM() const {
    u16 x{0};
    for (auto byte : rom_data.Span().first<0x14E>()) x += byte;
    for (auto byte : rom_data.Span().subspan<0x150>()) x += byte;
    return x;
}

} // namespace CGB
