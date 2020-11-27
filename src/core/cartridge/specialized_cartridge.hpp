#pragma once

#include <memory>

#include "cartridge.hpp"

namespace CGB {

class Bus;

class SpecializedCartridge : public Cartridge {

protected:
    explicit SpecializedCartridge(Cartridge&& unspecialized) : Cartridge{std::move(unspecialized)} {};

public:
    explicit SpecializedCartridge(SpecializedCartridge&&) = default;
    virtual ~SpecializedCartridge() = default;

    static std::unique_ptr<SpecializedCartridge> Make(Cartridge unspecialized);
    virtual void Install(Bus& bus) = 0;
};

} // namespace CGB