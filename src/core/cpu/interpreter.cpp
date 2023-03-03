#include "core/ppu/ppu.hpp"
#include "interpreter.hpp"

#include <algorithm>
#include <ranges>

namespace CGB::Core::CPU {

Assembler::Assembler() {}

Assembler::~Assembler() {}

void Assembler::prologue() {}

void Interpreter::ScheduleEvent(u64 event_timestamp, Event event) {
    schedule.emplace(event_timestamp, event);
}
void Interpreter::DescheduleEvent(Event event) {
    auto it = std::ranges::find_if(schedule, [event](decltype(schedule)::const_reference pair) {
        return pair.second == event;
    });
    if (it != schedule.end()) { schedule.erase(it); }
}

Interpreter::Interpreter() : assembler{std::make_unique<Assembler>()} {
    r8.C = 0x13;
    r8.B = 0x00;
    r8.E = 0xD8;
    r8.D = 0x00;
    r8.L = 0x4D;
    r8.H = 0x01;
    r8.A = 0x01;
    r8.F = 0;
}

void Interpreter::Install(Bus& _bus) {
    bus = &_bus;

    auto interrupt_reg_tag =
        bus->RegisterMemoryTag(InterruptRegReadHandler, InterruptRegWriteHandler);
    bus->AttachIOHandler(0x0F, interrupt_reg_tag);
    bus->AttachIOHandler(0xFF, interrupt_reg_tag);
}

void Interpreter::Run() {
    PC = 0xFF;
    timestamp = 0;
    start = std::chrono::high_resolution_clock::now();
    auto prev_frame = start;
    schedule.emplace(PPU::HEIGHT * PPU::SCANLINE_CYCLES, Event::VBlank);
    schedule.emplace(bus->GetPPU().GetLcdRegs().lyc * PPU::SCANLINE_CYCLES,
                     Event::LCD_STAT_LYC_IS_LY);
    while (true) {
        if (timestamp < schedule.begin()->first) [[likely]] {
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

            bus->GetPPU().VBlank(timestamp);
            if (IME && (interrupt_enable & (1 << 0))) {
                timestamp += 8;
                IME = false;
                PushWord(PC + 1);
                timestamp += 4;
                Jump(0x40);
            }
        } break;
        case Event::LCD_STAT_LYC_IS_LY: {
            bus->GetPPU().LCD_STAT_LYC_IS_LY(timestamp);
            if (IME && (interrupt_enable & (1 << 1)) &&
                bus->GetPPU().GetLcdRegs().stat & (1 << 6)) {
                timestamp += 8;
                IME = false;
                PushWord(PC + 1);
                timestamp += 4;
                Jump(0x48);
            }
        } break;
        default: UNREACHABLE();
        }
    }
}

} // namespace CGB::Core::CPU