#include "core/ppu/ppu.hpp"
#include "interpreter.hpp"

#include <algorithm>
#include <ranges>

namespace CGB::Core::CPU {

using namespace Xbyak;

Assembler::Assembler(Interpreter& interpreter)
    : Xbyak::CodeGenerator(DEFAULT_MAX_CODE_SIZE, Xbyak::AutoGrow), interpreter{interpreter} {

    generatePrologue();
    generateEpilogue();

    align();
    L(jump_table);
    for (const auto& opcode : opcode_labels) { putL(opcode); }

    // NOP
    {
        beginOpcode(0b00000000);
        endOpcode();
    }
    // JR_s8
    {
        beginOpcode(0b00011000);
        movsx(nv_rhs16, Imm8());
        inc(nv_pc);
        add(nv_pc, nv_rhs16);
        accumulated_timestamp += 4;
        endOpcode();
    }
    // JP_u16
    {
        beginOpcode(0b11'000'011);
        mov(nv_pc, Imm16());
        accumulated_timestamp += 4;
        endOpcode();
    }
    generateInterruptControl();

    // Deal with undefined labels
    align();
    for (usize opcode_idx = 0; opcode_idx < opcode_labels.size(); ++opcode_idx) {
        if (!implemented_opcodes.test(opcode_idx)) { L(opcode_labels[opcode_idx]); }
    }
    endbr64();
    // Rollback last opcode fetch
    sub(v_timestamp, 4);
    dec(nv_pc);
    jmp(epilogue);

    align();
    const auto embedded_unwind_info_pos = static_cast<DWORD>(getSize());
    auto* const embedded_unwind_info =
        static_cast<UNWIND_INFO*>(allocateFromCodeSpace(sizeof(UNWIND_INFO)));
    *embedded_unwind_info = unwind_info;

    readyRE();

    debug_function_table.emplace_back(RUNTIME_FUNCTION{
        .BeginAddress = 0,
        .EndAddress = embedded_unwind_info_pos,
        .UnwindInfoAddress = embedded_unwind_info_pos,
    });
    RtlAddFunctionTable(debug_function_table.data(),
                        static_cast<DWORD>(debug_function_table.size()),
                        reinterpret_cast<DWORD64>(getCode()));

    for (usize opcode_idx = 0; opcode_idx < opcode_labels.size(); ++opcode_idx) {
        if (implemented_opcodes.test(opcode_idx)) {
            interpreter.JUMP_TABLE[opcode_idx] =
                std::bit_cast<Interpreter::Opcode>(prologue.getAddress());
        }
    }
}

Assembler::~Assembler() { RtlDeleteFunctionTable(debug_function_table.data()); }

void* Assembler::allocateFromCodeSpace(usize alloc_size) {
    if (size_ + alloc_size >= maxSize_) { throw Xbyak::Error(Xbyak::ERR_CODE_IS_TOO_BIG); }

    void* ret = getCurr<void*>();
    size_ += alloc_size;
    memset(ret, 0, alloc_size);
    return ret;
}

void Assembler::generatePrologue() {
    align();
    L(prologue);
    {
        const size_t begin_prologue = getSize();
        auto unwind_code = unwind_info.UnwindCode.begin();
        for (auto reg : pushed_registers) {
            (unwind_code++)->code = {
                .CodeOffset = static_cast<UBYTE>(getSize()),
                .UnwindOp = UWOP_PUSH_NONVOL,
                .OpInfo = static_cast<UBYTE>(reg.getIdx()),
            };
            push(reg);
        }
        const size_t end_prologue = getSize();

        unwind_info.Version = 1;
        unwind_info.Flags = 0;
        unwind_info.SizeOfProlog = static_cast<UBYTE>(end_prologue - begin_prologue);
        unwind_info.CountOfCodes = static_cast<UBYTE>(unwind_code - unwind_info.UnwindCode.begin());
        unwind_info.FrameRegister = 0; // No frame register present
        unwind_info.FrameOffset = 0;   // Unused because FrameRegister == 0
    }
    mov(nv_interp, Reg64{Reg64::RCX});
    movzx(v_addr.cvt32(), Reg8{Reg8::DL});
    mov(nv_jump_table, jump_table);

    mov(v_bus, qword[nv_interp + offsetof(Interpreter, bus)]);
    mov(nv_mem, qword[v_bus + offsetof(Bus, address_space_loc)]);

    movzx(nv_pc.cvt32(), word[nv_interp + offsetof(Interpreter, PC)]);
    movzx(nv_sp.cvt32(), word[nv_interp + offsetof(Interpreter, SP)]);
    mov(v_timestamp, qword[nv_interp + offsetof(Interpreter, timestamp)]);
    mov(nv_log_ptr, qword[nv_interp + offsetof(Interpreter, profile_log_ptr)]);

    jmp(qword[nv_jump_table + v_addr.cvt64() * 8]);
}

void Assembler::generateEpilogue() {
    align();
    L(epilogue);

    mov(qword[nv_interp + offsetof(Interpreter, profile_log_ptr)], nv_log_ptr);
    mov(qword[nv_interp + offsetof(Interpreter, timestamp)], v_timestamp);
    mov(word[nv_interp + offsetof(Interpreter, SP)], nv_sp);
    mov(word[nv_interp + offsetof(Interpreter, PC)], nv_pc);

    for (auto reg : pushed_registers | std::ranges::views::reverse) { pop(reg); }
    ret();
    align();
}

void Assembler::beginOpcode(u8 opcode) {
    accumulated_timestamp = 0;
    current_opcode = opcode;
    align();
    implemented_opcodes.set(opcode);
    L(opcode_labels[opcode]);
    endbr64();
    mov(byte[nv_log_ptr], v_addr.cvt8());
    inc(nv_log_ptr);
}
void Assembler::endOpcode() {
    Label exit;
    if (accumulated_timestamp) { add(v_timestamp, accumulated_timestamp); }
    cmp(v_timestamp, qword[nv_interp + offsetof(Interpreter, next_event_timestamp)]);
    jae(epilogue);

    movzx(v_addr.cvt32(), byte[nv_mem + nv_pc.cvt64()]);
    mov(v_jump_target, qword[nv_jump_table + v_addr.cvt64() * 8]);
    inc(nv_pc);
    add(v_timestamp, 4);
    jmp(v_jump_target);
}

void Assembler::generateInterruptControl() {
    for (u8 variant : {u8{0b11'110'011}, u8{0b11'111'011}}) {
        beginOpcode(variant);
        bool enable_interrupts = variant & 0b00'001'000;
        mov(byte[nv_interp + offsetof(Interpreter, IME)], enable_interrupts);
        endOpcode();
    }
}

void Interpreter::ScheduleEvent(u64 event_timestamp, Event event) {
    schedule.emplace(event_timestamp, event);
    UpdateNextEvent();
}
void Interpreter::DescheduleEvent(Event event) {
    auto it = std::ranges::find_if(schedule, [event](decltype(schedule)::const_reference pair) {
        return pair.second == event;
    });
    if (it != schedule.end()) {
        schedule.erase(it);
        UpdateNextEvent();
    }
}
void Interpreter::UpdateNextEvent() {
    next_event_timestamp =
        schedule.empty() ? std::numeric_limits<u64>::max() : schedule.cbegin()->first;
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
    PC = 0x100;
    timestamp = 0;

    // TODO: something with this log data
    profile_log = std::vector<u8>(PPU::FRAME_CYCLES / 4);
    profile_log_ptr = profile_log.data();

    start = std::chrono::high_resolution_clock::now();
    auto prev_frame = start;
    schedule.emplace(PPU::HEIGHT * PPU::SCANLINE_CYCLES, Event::VBlank);
    schedule.emplace(bus->GetPPU().GetLcdRegs().lyc * PPU::SCANLINE_CYCLES,
                     Event::LCD_STAT_LYC_IS_LY);
    UpdateNextEvent();
    while (true) {
        while (timestamp < next_event_timestamp) [[likely]] {
            u8 opcode = Imm8();
            JUMP_TABLE[opcode](*this, opcode);
        }
        auto [t, event] = std::move(*schedule.begin());
        schedule.erase(schedule.begin());
        UpdateNextEvent();
        switch (event) {
        case Event::VBlank: {
            auto end_frame = std::chrono::high_resolution_clock::now();
            speed = (end_frame - prev_frame);
            prev_frame = end_frame;

            // LOG(Debug, "Ran {} instructions this frame", profile_log_ptr - profile_log.data());
            profile_log_ptr = profile_log.data();

            bus->GetPPU().VBlank(timestamp);
            if (IME && (interrupt_enable & (1 << 0))) {
                timestamp += 8;
                IME = false;
                PushWord(PC);
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
                PushWord(PC);
                timestamp += 4;
                Jump(0x48);
            }
        } break;
        default: UNREACHABLE();
        }
    }
}

} // namespace CGB::Core::CPU