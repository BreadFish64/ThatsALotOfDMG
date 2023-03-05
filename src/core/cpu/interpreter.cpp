#include "core/ppu/ppu.hpp"
#include "interpreter.hpp"

#include <algorithm>
#include <ranges>

namespace CGB::Core::CPU {

using namespace Xbyak;

Assembler::Assembler(Interpreter& interpreter) : interpreter{interpreter} {
    generatePrologue();
    generateEpilogue();

    generateInterruptControl();

    align();
    const auto embedded_unwind_info_pos = static_cast<DWORD>(getSize());
    auto* const embedded_unwind_info =
        static_cast<UNWIND_INFO*>(allocateFromCodeSpace(sizeof(UNWIND_INFO)));
    align();
    *embedded_unwind_info = unwind_info;

    debug_function_table.emplace_back(RUNTIME_FUNCTION{
        .BeginAddress = 0,
        .EndAddress = embedded_unwind_info_pos,
        .UnwindInfoAddress = embedded_unwind_info_pos,
    });
    RtlAddFunctionTable(debug_function_table.data(),
                        static_cast<DWORD>(debug_function_table.size()),
                        reinterpret_cast<DWORD64>(getCode()));
    readyRE();
}

Assembler::~Assembler() { RtlDeleteFunctionTable(debug_function_table.data()); }

void Assembler::generatePrologue() {
    align();
    prologue = getCurr<decltype(prologue)>();

    {
        const uint8_t* const begin_prologue = getCurr();
        auto unwind_code = unwind_info.UnwindCode.begin();
        for (auto reg : pushed_registers) {
            unwind_code->code = {
                .CodeOffset = static_cast<UBYTE>(getCurr() - begin_prologue),
                .UnwindOp = UWOP_PUSH_NONVOL,
                .OpInfo = static_cast<UBYTE>(reg.getIdx()),
            };
            push(reg);
        }
        const uint8_t* const end_prologue = getCurr();

        unwind_info.Version = 1;
        unwind_info.Flags = 0;
        unwind_info.SizeOfProlog = static_cast<UBYTE>(end_prologue - begin_prologue);
        unwind_info.CountOfCodes = static_cast<UBYTE>(unwind_code - unwind_info.UnwindCode.begin());
        unwind_info.FrameRegister = 0; // No frame register present
        unwind_info.FrameOffset = 0;   // Unused because FrameRegister == 0
    }

    mov(nv_interp, Reg64{Reg64::RCX});
    movzx(v_addr.cvt64(), Reg8{Reg8::DL});

    mov(v_bus, qword[nv_interp + offsetof(Interpreter, bus)]);
    mov(nv_mem, qword[v_bus + offsetof(Bus, address_space_loc)]);

    mov(nv_pc, word[nv_interp + offsetof(Interpreter, PC)]);
    mov(v_timestamp, qword[nv_interp + offsetof(Interpreter, timestamp)]);

    jmp(qword[nv_interp + v_addr.cvt64() * 8 + offsetof(Interpreter, OPCODE_TABLE)]);
}

void Assembler::generateEpilogue() {
    align();
    L(epilogue);

    mov(qword[nv_interp + offsetof(Interpreter, timestamp)], v_timestamp);
    mov(word[nv_interp + offsetof(Interpreter, PC)], nv_pc);

    for (auto reg : pushed_registers | std::ranges::views::reverse) { pop(reg); }
    ret();
    align();
}

void Assembler::beginOpcode(u8 opcode) {
    align();
    interpreter.OPCODE_TABLE[opcode] = getCurr<void*>();
    interpreter.JUMP_TABLE[opcode] = prologue;
    // TODO: control flow guard
}
void Assembler::endOpcode([[maybe_unused]] u8 opcode) { jmp(epilogue); }

void Assembler::generateInterruptControl() {
    for (u8 variant : {u8{0b11'110'011}, u8{0b11'111'011}}) {
        beginOpcode(variant);
        bool enable_interrupts = variant & 0b00'001'000;
        mov(byte[nv_interp + offsetof(Interpreter, IME)], enable_interrupts);
        endOpcode(variant);
    }
}

void Interpreter::ScheduleEvent(u64 event_timestamp, Event event) {
    schedule.emplace(event_timestamp, event);
}
void Interpreter::DescheduleEvent(Event event) {
    auto it = std::ranges::find_if(schedule, [event](decltype(schedule)::const_reference pair) {
        return pair.second == event;
    });
    if (it != schedule.end()) { schedule.erase(it); }
}

Interpreter::Interpreter() {
    r8.C = 0x13;
    r8.B = 0x00;
    r8.E = 0xD8;
    r8.D = 0x00;
    r8.L = 0x4D;
    r8.H = 0x01;
    r8.A = 0x01;
    r8.F = 0;

    assembler = std::make_unique<Assembler>(*this);
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