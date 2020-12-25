#include <algorithm>
#include <ranges>

#include "core/ppu/ppu.hpp"
#include "interpreter.hpp"

namespace CGB::Core::CPU {

void Interpreter::ScheduleEvent(u64 timestamp, Event event) { schedule.emplace(timestamp, event); }
void Interpreter::DescheduleEvent(Event event) {
    auto it = std::ranges::find_if(schedule, [event](decltype(schedule)::const_reference pair) {
        return pair.second == event;
    });
    if (it != schedule.end()) { schedule.erase(it); }
}

void Interpreter::Install(Bus& bus) {
    this->bus = &bus;

    auto interrupt_reg_tag =
        bus.RegisterMemoryTag(InterruptRegReadHandler, InterruptRegWriteHandler);
    bus.AttachIOHandler(0x0F, interrupt_reg_tag);
    bus.AttachIOHandler(0xFF, interrupt_reg_tag);
}

void Interpreter::Run() {
    PC = 0xFF;
    timestamp = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto prev_frame = start;
    schedule.emplace(PPU::HEIGHT * PPU::SCANLINE_CYCLES, Event::VBlank);
    while (true) {
        while (timestamp < schedule.begin()->first) [[likely]] {
                u8 opcode = Imm8();
                JUMP_TABLE[opcode](*this, opcode);
            }
        auto [t, event] = std::move(*schedule.begin());
        schedule.erase(schedule.begin());
        switch (event) {
        case Event::VBlank: {
            auto end_frame = std::chrono::high_resolution_clock::now();
            speed = (end_frame - prev_frame);
            prev_frame = end_frame;

            // TODO: Run VBlank interrupt
            bus->GetPPU().VBlank(timestamp);
            if (IME && (interrupt_enable & 0b00001)) {
                timestamp += 8;
                IME = false;
                PushWord(PC + 1);
                timestamp += 4;
                Jump(0x40);
            }
        } break;
        default: UNREACHABLE();
        }
    }
}

} // namespace CGB::Core::CPU