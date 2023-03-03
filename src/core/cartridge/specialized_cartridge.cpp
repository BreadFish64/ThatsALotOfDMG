#include "common/logger.hpp"

#include "bare.hpp"
#include "mbc1.hpp"
#include "mbc5.hpp"
#include "specialized_cartridge.hpp"

namespace CGB::Core {

std::unique_ptr<SpecializedCartridge> SpecializedCartridge::Make(CartridgeHeader unspecialized) {
    switch (unspecialized.CartridgeType()) {
    case CARTRIDGE_TYPE::ROM_ONLY:
    case CARTRIDGE_TYPE::ROM_RAM:
    case CARTRIDGE_TYPE::ROM_RAM_BATTERY: return std::make_unique<BARE>(std::move(unspecialized));
    case CARTRIDGE_TYPE::MBC1:
    case CARTRIDGE_TYPE::MBC1_RAM:
    case CARTRIDGE_TYPE::MBC1_RAM_BATTERY: return std::make_unique<MBC1>(std::move(unspecialized));
    case CARTRIDGE_TYPE::MBC5:
    case CARTRIDGE_TYPE::MBC5_RAM:
    case CARTRIDGE_TYPE::MBC5_RAM_BATTERY: return std::make_unique<MBC5>(std::move(unspecialized));
    default:
        LOG(Critical, "Unimplemented cartridge type: {:#04X}",
            underlying_cast(unspecialized.CartridgeType()));
    }
    return nullptr;
}

} // namespace CGB::Core