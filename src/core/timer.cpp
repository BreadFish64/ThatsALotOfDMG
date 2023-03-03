#include "bus.hpp"
#include "timer.hpp"

namespace CGB::Core {

u8 Timer::TimerReadHandler(Bus& bus, GADDR addr, u64 timestamp) {
    auto timer = bus.GetTimer();
    switch (addr) {
    case 0xFF04: return static_cast<u8>(timestamp - timer.last_reset);
    case 0xFF05: return (timer.tac & 0b100) ? timer.ReadTIMA(timestamp) : timer.tima;
    case 0xFF06: return timer.tma;
    case 0xFF07: return timer.tac;
    default: UNREACHABLE();
    }
}

void Timer::TimerWriteHandler(Bus& bus, GADDR addr, u8 val, u64 timestamp) {
    auto timer = bus.GetTimer();
    switch (addr) {
    case 0xFF04: timer.last_reset = timestamp; break;
    case 0xFF05: timer.tima = val; break;
    case 0xFF06: timer.tma = val; break;
    case 0xFF07:
        timer.tac = val;
        if (!(val & 0b100)) { timer.tima = timer.ReadTIMA(timestamp); }
        break;
    default: UNREACHABLE();
    }
}

u8 Timer::ReadTIMA(u64 timestamp) const {
    int shift = ((tac - 1) & 0b11) * 2 + 2;
    u64 ticks = ((timestamp - last_reset) >> shift) + tima;
    return static_cast<u8>(ticks % (0x100 - tma));
}

void Timer::Install(Bus& bus) {
    auto timer_tag = bus.RegisterMemoryTag(TimerReadHandler, TimerWriteHandler);
    for (u8 io = 0x04; io <= 0x07; ++io) bus.AttachIOHandler(io, timer_tag);
}

} // namespace CGB::Core