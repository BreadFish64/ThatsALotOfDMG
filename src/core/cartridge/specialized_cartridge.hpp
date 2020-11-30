#pragma once

#include <memory>

#include "cartridge.hpp"

namespace CGB::Core {

class Bus;

class SpecializedCartridge : public CartridgeHeader {

protected:
    explicit SpecializedCartridge(CartridgeHeader&& unspecialized)
        : CartridgeHeader{std::move(unspecialized)} {};

public:
    explicit SpecializedCartridge(SpecializedCartridge&&) = default;
    virtual ~SpecializedCartridge() = default;

    static std::unique_ptr<SpecializedCartridge> Make(CartridgeHeader unspecialized);
    virtual void Install(Bus& bus) = 0;
};

} // namespace CGB