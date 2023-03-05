#pragma once

#include "common/types.hpp"

namespace CGB::Core {

class Bus;

class Timer {
    // TODO: verify behavior
    u64 last_reset{};
    u8 tima{};
    u8 tma{};
    u8 tac{};

    static u8 TimerReadHandler(Bus& bus, GADDR addr, u64 timestamp);
    static void TimerWriteHandler(Bus& bus, GADDR addr, u64 timestamp, u8 val);

    u8 ReadTIMA(u64 timestamp) const;

public:
    void Install(Bus& bus);
};

} // namespace CGB::Core::Timer