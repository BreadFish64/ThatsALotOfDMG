#include <filesystem>
#include <memory>

#include "common/logger.hpp"
#include "common/types.hpp"
#include "core/bus.hpp"
#include "core/cartridge/cartridge.hpp"
#include "core/cpu/cpu.hpp"
#include "core/ppu/ppu.hpp"

int main(int argc, const char** argv) {
    std::span<const char*> args{argv, static_cast<std::size_t>(argc)};
    if (args.size() < 2) return 1;
    std::filesystem::path game = args[1];
    auto cartridge = std::make_unique<CGB::Core::CartridgeHeader>(game);
    LOG(Info, "Title: {}", cartridge->Title());
    LOG(Info, "Header Checksum: {:#04X} Hash: {:#04X}", cartridge->HeaderChecksum(),
        cartridge->HashHeader());
    LOG(Info, "ROM Checksum: {:#06X} Hash: {:#06X}", cartridge->GlobalChecksum(),
        cartridge->HashROM());
    LOG(Info, "CartridgeType {:#04X}", cartridge->CartridgeType(), cartridge->HashROM());
    CGB::Core::Bus bus{std::move(cartridge), CGB::Core::CPU::MainCPU{}, std::make_unique<CGB::Core::PPU>()};
}