#pragma once

#include "common/types.hpp"
#include "core/bus.hpp"

#include <xbyak/xbyak.h>

#include <boost/container/flat_map.hpp>
#include <boost/container/static_vector.hpp>

#include <chrono>

// These macros were a mistake and I'm sorry to anyone trying to understand them
#define TABLE_FILL(operation, base, shift, bits)                                                   \
    [&table]<u8... idx>(u8 _base, u8 _shift, std::integer_sequence<u8, idx...>) {                  \
        ((table[_base | (idx << _shift)] = TailCallInstr<&Interpreter::operation<idx>>), ...);     \
    }                                                                                              \
    (base, shift, std::make_integer_sequence<u8, 1 << bits>());
#define DOUBLE_TABLE_FILL(operation, base, shift1, bits1, shift2, bits2)                           \
    [&table]<u8... idx1, u8... idx2>(u8 _base, u8 _shift1, std::integer_sequence<u8, idx1...>,     \
                                     u8 _shift2, std::integer_sequence<u8, idx2...> count2) {      \
        const auto helper = [&table]<u8 left, u8... right>(u8 _base, u8 _shift,                    \
                                                           std::integer_sequence<u8, right...>) {  \
            ((table[_base | (right << _shift)] =                                                   \
                  TailCallInstr<&Interpreter::operation<left, right>>),                            \
             ...);                                                                                 \
        };                                                                                         \
        ((helper.template operator()<idx1>(_base | (idx1 << _shift1), _shift2, count2)), ...);     \
    }                                                                                              \
    (base, shift1, std::make_integer_sequence<u8, 1 << bits1>(), shift2,                           \
     std::make_integer_sequence<u8, 1 << bits2>());

#ifndef __has_builtin
#define __has_builtin() false
#endif

#if !__has_builtin(__builtin_add_overflow)
template <std::unsigned_integral U>
inline bool __builtin_add_overflow(U a, U b, U* res) {
    U tmp_res = a + b;
    bool overflow = tmp_res < a;
    *res = tmp_res;
    return overflow;
}
#endif

#if !__has_builtin(__builtin_sub_overflow)
template <std::unsigned_integral U>
inline bool __builtin_sub_overflow(U a, U b, U* res) {
    U tmp_res = a - b;
    bool overflow = tmp_res > a;
    *res = tmp_res;
    return overflow;
}
#endif

#if !__has_builtin(__builtin_addcb)
inline unsigned char __builtin_addcb(unsigned char a, unsigned char b, unsigned char cin,
                                     unsigned char* cout) {
    unsigned char tmp_res{};
    unsigned char overflow = __builtin_add_overflow(a, b, &tmp_res) ? 1 : 0;
    overflow |= __builtin_add_overflow(tmp_res, cin, &tmp_res) ? 1 : 0;
    *cout = overflow;
    return tmp_res;
}
#endif

#if !__has_builtin(__builtin_subcb)
inline unsigned char __builtin_subcb(unsigned char a, unsigned char b, unsigned char cin,
                                     unsigned char* cout) {
    unsigned char tmp_res{};
    unsigned char overflow = __builtin_sub_overflow(a, b, &tmp_res) ? 1 : 0;
    overflow |= __builtin_sub_overflow(tmp_res, cin, &tmp_res) ? 1 : 0;
    *cout = overflow;
    return tmp_res;
}
#endif

namespace CGB::Core::CPU {

class Interpreter;

struct Assembler : Xbyak::CodeGenerator {
    Interpreter& interpreter;

    Assembler(Interpreter& interpreter);
    ~Assembler();

    static constexpr auto v_bus = Xbyak::Reg64{Xbyak::Reg64::RCX};
    static constexpr auto v_timestamp = Xbyak::Reg64{Xbyak::Reg64::RDX};

    static constexpr auto nv_interp = Xbyak::Reg64{Xbyak::Reg64::RDI};
    static constexpr auto nv_mem = Xbyak::Reg64{Xbyak::Reg64::RSI};
    static constexpr auto nv_pc = Xbyak::Reg16{Xbyak::Reg16::BP};
    static constexpr auto nv_scratch = Xbyak::Reg64{Xbyak::Reg64::RBX};

    static constexpr auto v_addr = Xbyak::Reg16{Xbyak::Reg16::R8};

    void (*prologue)(Interpreter&, u8);
    Xbyak::Label epilogue;

    void generatePrologue();
    void generateEpilogue();

    void beginOpcode(u8 opcode);
    void endOpcode(u8 opcode);

    void generateInterruptControl();
};

template <typename Key, typename Val, std::size_t size>
using StaticMap =
    boost::container::flat_multimap<Key, Val, std::less<Key>,
                                    boost::container::static_vector<std::pair<Key, Val>, size>>;

class Interpreter final : public BaseCPU {
    friend struct Assembler;

    static constexpr std::array<std::string_view, 8> R8_NAME{
        "B", "C", "D", "E", "H", "L", "[HL]", "A",
    };
    static constexpr std::array<std::string_view, 4> G1_R16_NAME{
        "BC",
        "DE",
        "HL",
        "SP",
    };
    static constexpr std::array<std::string_view, 4> G2_R16_NAME{
        "BC",
        "DE",
        "HL+",
        "HL-",
    };
    static constexpr std::array<std::string_view, 4> G3_R16_NAME{
        "BC",
        "DE",
        "HL",
        "AF",
    };
    static constexpr std::array<std::string_view, 4> CONDITION_NAME{
        "NZ",
        "Z",
        "NC",
        "C",
    };
    static constexpr std::array<std::string_view, 8> ALU_OP_NAME{
        "ADD", "ADC", "SUB", "SBC", "AND", "XOR", "OR", "CP",
    };
    static constexpr std::array<std::string_view, 8> CB_SHIFT_OP_NAME{
        "RLC", "RRC", "RL", "RR", "SLA", "SRA", "SWAP", "SRL",
    };

    union {
        std::array<u8, 8> reg_arr;
        struct {
            u16 BC;
            u16 DE;
            u16 HL;
            u16 AF;
        } r16;
        struct {
            u8 C;
            u8 B;
            u8 E;
            u8 D;
            u8 L;
            u8 H;
            u8 A;
            u8 F;
        } r8;
    };
    struct {
        bool Z{true}, N{false}, H{true}, C{true};
    } flag;
    u16 SP{0xFFFE};
    u16 PC{};
    u64 timestamp{0};

    Bus* bus{};

    u8 Load(GADDR addr) {
        u8 val = bus->Read(timestamp, addr);
        timestamp += 4;
        return val;
    }

    void Store(GADDR addr, u8 val) {
        bus->Write(timestamp, addr, val);
        timestamp += 4;
    }

    u8 Imm8() {
        u8 val = bus->Memory()[++PC];
        timestamp += 4;
        return val;
    }

    u16 Imm16() {
        u16 imm16 = Imm8();
        imm16 |= u16{Imm8()} << 8;
        return imm16;
    }

    u8 PackFlags() {
        u8 F{0};
        F |= flag.Z << 7;
        F |= flag.N << 6;
        F |= flag.H << 5;
        F |= flag.C << 4;
        return F;
    }

    void UnpackFlags(u8 F) {
        flag.Z = F & (1 << 7);
        flag.N = F & (1 << 6);
        flag.H = F & (1 << 5);
        flag.C = F & (1 << 4);
    }

    template <u8 reg_idx>
    u8 ReadR8() {
        if constexpr (reg_idx == 6) {
            return Load(r16.HL);
        } else {
            return reg_arr[reg_idx ^ 1];
        }
    }

    template <u8 reg_idx>
    void WriteR8(u8 val) {
        if constexpr (reg_idx == 6) {
            Store(r16.HL, val);
        } else {
            reg_arr[reg_idx ^ 1] = val;
        }
    }

    template <u8 reg_idx>
    u16 ReadG1R16() {
        switch (reg_idx) {
        case 0: return r16.BC;
        case 1: return r16.DE;
        case 2: return r16.HL;
        case 3: return SP;
        default: UNREACHABLE();
        }
    }

    template <u8 reg_idx>
    void WriteG1R16(u16 val) {
        switch (reg_idx) {
        case 0: r16.BC = val; break;
        case 1: r16.DE = val; break;
        case 2: r16.HL = val; break;
        case 3: SP = val; break;
        default: UNREACHABLE();
        }
    }

    template <u8 reg_idx>
    u16 ReadG2R16() {
        switch (reg_idx) {
        case 0: return r16.BC;
        case 1: return r16.DE;
        case 2: return r16.HL++;
        case 3: return r16.HL--;
        default: UNREACHABLE();
        }
    }

    template <u8 condition>
    bool CheckCondition() {
        switch (condition) {
        case 0: return !flag.Z;
        case 1: return flag.Z;
        case 2: return !flag.C;
        case 3: return flag.C;
        default: UNREACHABLE();
        }
    }

    template <u8 reg_idx>
    u16 ReadG3R16() {
        switch (reg_idx) {
        case 0: return r16.BC;
        case 1: return r16.DE;
        case 2: return r16.HL;
        case 3: return (u16{r8.A} << 8) | PackFlags();
        default: UNREACHABLE();
        }
    }

    template <u8 reg_idx>
    void WriteG3R16(u16 val) {
        switch (reg_idx) {
        case 0: r16.BC = val; break;
        case 1: r16.DE = val; break;
        case 2: r16.HL = val; break;
        case 3:
            r8.A = val >> 8;
            UnpackFlags(static_cast<u8>(val));
            break;
        }
    }

    void Jump(GADDR braddr) {
        timestamp += 4;
        PC = braddr - 1;
    }
    void PushByte(u8 val) { Store(--SP, val); }
    void PushWord(u16 val) {
        PushByte(static_cast<u8>(val >> 8));
        PushByte(static_cast<u8>(val));
    }
    u8 PopByte() { return Load(SP++); }
    u16 PopWord() { return PopByte() | (u16{PopByte()} << 8); }

    void ADD(u8 rhs) {
        u8 lhs = r8.A;
        flag.H = ((lhs & 0x0F) + (rhs & 0x0F)) & 0x10;
        u8 result;
        flag.C = __builtin_add_overflow(lhs, rhs, &result);
        flag.Z = result == 0;
        flag.N = false;
        r8.A = result;
    }
    void ADC(u8 rhs) {
        u8 lhs = r8.A;
        u8 carry;
        flag.H = __builtin_addcb(static_cast<u8>(lhs & 0x0F), static_cast<u8>(rhs & 0x0F), flag.C,
                                 &carry) &
                 0x10;
        u8 result = __builtin_addcb(lhs, rhs, flag.C, &carry);
        flag.C = carry;
        flag.Z = result == 0;
        flag.N = false;
        r8.A = result;
    }
    void SUB(u8 rhs) {
        u8 lhs = r8.A;
        flag.H = (lhs & 0x0F) < (rhs & 0x0F);
        u8 result;
        flag.C = __builtin_sub_overflow(lhs, rhs, &result);
        flag.Z = result == 0;
        flag.N = true;
        r8.A = result;
    }
    void SBC(u8 rhs) {
        u8 lhs = r8.A;
        u8 carry;
        flag.H = static_cast<s8>(__builtin_subcb(lhs & 0x0F, rhs & 0x0F, flag.C, &carry)) < 0;
        u8 result = __builtin_subcb(lhs, rhs, flag.C, &carry);
        flag.C = carry;
        flag.Z = result == 0;
        flag.N = true;
        r8.A = result;
    }
    void OR(u8 rhs) {
        u8 lhs = r8.A;
        u8 result = lhs | rhs;
        flag.Z = result == 0;
        flag.N = false;
        flag.H = false;
        flag.C = false;
        r8.A = result;
    }
    void AND(u8 rhs) {
        u8 lhs = r8.A;
        u8 result = lhs & rhs;
        flag.Z = result == 0;
        flag.N = false;
        flag.H = true;
        flag.C = false;
        r8.A = result;
    }
    void XOR(u8 rhs) {
        u8 lhs = r8.A;
        u8 result = lhs ^ rhs;
        flag.Z = result == 0;
        flag.N = false;
        flag.H = false;
        flag.C = false;
        r8.A = result;
    }
    void CP(u8 rhs) {
        u8 lhs = r8.A;
        flag.H = (lhs & 0x0F) < (rhs & 0x0F);
        u8 result;
        flag.C = __builtin_sub_overflow(lhs, rhs, &result);
        flag.Z = result == 0;
        flag.N = true;
    }

    u8 RLC(u8 val) {
        flag.C = val & (1 << 7);
        val = (val << 1) | (flag.C ? 1 : 0);
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 RRC(u8 val) {
        flag.C = val & 1;
        val = (flag.C << 7) | (val >> 1);
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 RL(u8 val) {
        bool carry = val & (1 << 7);
        val = (val << 1) | (flag.C ? 1 : 0);
        flag.C = carry;
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 RR(u8 val) {
        bool carry = val & 1;
        val = (flag.C << 7) | (val >> 1);
        flag.C = carry;
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 SLA(u8 val) {
        flag.C = val & (1 << 7);
        val <<= 1;
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 SRA(u8 val) {
        flag.C = val & 1;
        val = static_cast<s8>(val) >> 1;
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 SWAP(u8 val) {
        val = (val << 4) | (val >> 4);
        flag.C = false;
        flag.N = false;
        flag.H = false;
        return val;
    }
    u8 SRL(u8 val) {
        flag.C = val & 1;
        val >>= 1;
        flag.N = false;
        flag.H = false;
        return val;
    }

    template <u8 operation, u8 reg_idx>
    void ShiftRot_r8([[maybe_unused]] u8 instruction) {

        u8 val = ReadR8<reg_idx>();
        val = (this->*std::array{
                          &Interpreter::RLC,
                          &Interpreter::RRC,
                          &Interpreter::RL,
                          &Interpreter::RR,
                          &Interpreter::SLA,
                          &Interpreter::SRA,
                          &Interpreter::SWAP,
                          &Interpreter::SRL,
                      }[operation])(val);
        flag.Z = val == 0;
        WriteR8<reg_idx>(val);
        // R8_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void BIT_u3_r8(u8 instruction) {

        u8 bit = (instruction >> 3) & 0b111;
        flag.Z = !(ReadR8<reg_idx>() & (1 << bit));
        flag.N = false;
        flag.H = true;
    }
    template <u8 reg_idx>
    void RES_u3_r8(u8 instruction) {

        u8 bit = (instruction >> 3) & 0b111;
        WriteR8<reg_idx>(ReadR8<reg_idx>() & ~(1 << bit));
    }
    template <u8 reg_idx>
    void SET_u3_r8(u8 instruction) {

        u8 bit = (instruction >> 3) & 0b111;
        WriteR8<reg_idx>(ReadR8<reg_idx>() | (1 << bit));
    }

    bool IME = false;
    u8 interrupt_enable{0};
    u8 interrupt_flags{0};

    StaticMap<u64, Event, 8> schedule;

    static u8 InterruptRegReadHandler(Bus& bus, GADDR addr, [[maybe_unused]] u64 timestamp) {
        auto& interpreter = static_cast<Interpreter&>(bus.GetCPU().GetImpl());
        switch (addr) {
        case 0xFF0F: return interpreter.interrupt_flags;
        case 0xFFFF: return interpreter.interrupt_enable;
        default: std::abort();
        }
    }
    static void InterruptRegWriteHandler(Bus& bus, GADDR addr, u8 val,
                                         [[maybe_unused]] u64 timestamp) {
        auto& interpreter = static_cast<Interpreter&>(bus.GetCPU().GetImpl());
        switch (addr) {
        // TODO: Schedule interrupts when writing to these flags
        case 0xFF0F: interpreter.interrupt_flags = val; break;
        case 0xFFFF: interpreter.interrupt_enable = val; break;
        default: std::abort();
        }
    }

    virtual void ScheduleEvent(u64 event_timestamp, Event event) override;
    virtual void DescheduleEvent(Event event) override;

    std::chrono::high_resolution_clock::time_point start;

    void UnimplementedOpcode(u8 instruction) {

        auto seconds =
            std::chrono::duration<double>{std::chrono::high_resolution_clock::now() - start};

        LOG(Critical, "{:#06X} Unimplemented Opcode {:#010B}", PC, instruction);
        LOG(Info, "Host time: {} Cycles: {} Speed: {}Hz", seconds, timestamp,
            static_cast<u64>(timestamp / seconds.count()));
    }

    template <void (Interpreter::*handler)(u8)>
    [[gnu::flatten]] static void Devirtualize(Interpreter& state, u8 arg) {
        (state.*handler)(arg);
    }
    using RegisterOpcode = void (*)(Interpreter&, u8);

    static constexpr std::array ALU_OPS{
        Devirtualize<&Interpreter::ADD>, Devirtualize<&Interpreter::ADC>,
        Devirtualize<&Interpreter::SUB>, Devirtualize<&Interpreter::SBC>,
        Devirtualize<&Interpreter::AND>, Devirtualize<&Interpreter::XOR>,
        Devirtualize<&Interpreter::OR>,  Devirtualize<&Interpreter::CP>,
    };

    static constexpr std::array<RegisterOpcode, 256> CB_TABLE = []() constexpr {
        std::array<RegisterOpcode, 256> table{};
#define TailCallInstr Devirtualize
        DOUBLE_TABLE_FILL(ShiftRot_r8, 0b00'000'000, 3, 3, 0, 3);
        for (u8 base = 0b01'000'000; base != 0b10'000'000; base += 0b00'001'000)
            TABLE_FILL(BIT_u3_r8, base, 0, 3);
        for (u8 base = 0b10'000'000; base != 0b11'000'000; base += 0b00'001'000)
            TABLE_FILL(RES_u3_r8, base, 0, 3);
        for (u8 base = 0b11'000'000; base != 0b00'000'000; base += 0b00'001'000)
            TABLE_FILL(SET_u3_r8, base, 0, 3);
#undef TailCallInstr
        return table;
    }();

    void LD_u16addr_SP([[maybe_unused]] u8 instruction) {
        GADDR addr = Imm16();
        Store(addr, static_cast<u8>(SP));
        Store(addr + 1, static_cast<u8>(SP >> 8));
    }
    void NOP([[maybe_unused]] u8 instruction) {}
    void JR_s8([[maybe_unused]] u8 instruction) {
        s8 offset = static_cast<s8>(Imm8());
        Jump(PC + 1 + offset);
    }
    template <u8 condition>
    void JR_cond_s8([[maybe_unused]] u8 instruction) {
        s8 offset = static_cast<s8>(Imm8());
        if (CheckCondition<condition>()) { Jump(PC + 1 + offset); }
    }
    template <u8 reg_idx>
    void LD_r16_u16([[maybe_unused]] u8 instruction) {
        u16 imm = Imm16();
        WriteG1R16<reg_idx>(imm);
    }
    template <u8 reg_idx>
    void ADD_HL_r16([[maybe_unused]] u8 instruction) {
        u16 rhs = ReadG1R16<reg_idx>();
        flag.H = ((r16.HL & 0x0FFF) + (rhs & 0x0FFF)) & 0x1000;
        flag.C = __builtin_add_overflow(r16.HL, rhs, &r16.HL);
        flag.N = false;
        timestamp += 4;
    }
    template <u8 reg_idx>
    void LD_A_r16addr([[maybe_unused]] u8 instruction) {
        GADDR addr = ReadG2R16<reg_idx>();
        u8 val = Load(addr);
        r8.A = val;
    }
    template <u8 reg_idx>
    void LD_r16addr_A([[maybe_unused]] u8 instruction) {
        GADDR addr = ReadG2R16<reg_idx>();
        u8 val = r8.A;
        Store(addr, val);
    }
    template <u8 reg_idx>
    void INC_r16([[maybe_unused]] u8 instruction) {
        WriteG1R16<reg_idx>(ReadG1R16<reg_idx>() + 1);
        timestamp += 4;
    }
    template <u8 reg_idx>
    void DEC_r16([[maybe_unused]] u8 instruction) {
        WriteG1R16<reg_idx>(ReadG1R16<reg_idx>() - 1);
        timestamp += 4;
    }
    template <u8 reg_idx>
    void INC_r8([[maybe_unused]] u8 instruction) {
        u8 val = ReadR8<reg_idx>();
        flag.H = (val & 0x0F) == 0xF;
        ++val;
        flag.Z = val == 0;
        flag.N = false;
        WriteR8<reg_idx>(val);
    }
    template <u8 reg_idx>
    void DEC_r8([[maybe_unused]] u8 instruction) {
        u8 val = ReadR8<reg_idx>();
        flag.H = (val & 0x0F) == 0x00;
        --val;
        flag.Z = val == 0;
        flag.N = true;
        WriteR8<reg_idx>(val);
    }
    template <u8 reg_idx>
    void LD_r8_u8([[maybe_unused]] u8 instruction) {
        u8 rhs = Imm8();
        WriteR8<reg_idx>(rhs);
    }
    void RLC_A([[maybe_unused]] u8 instruction) {
        r8.A = RLC(r8.A);
        flag.Z = false;
    }
    void RRC_A([[maybe_unused]] u8 instruction) {
        r8.A = RRC(r8.A);
        flag.Z = false;
    }
    void RL_A([[maybe_unused]] u8 instruction) {
        r8.A = RL(r8.A);
        flag.Z = false;
    }
    void RR_A([[maybe_unused]] u8 instruction) {
        r8.A = RR(r8.A);
        flag.Z = false;
    }
    // https://ehaskins.com/2018-01-30%20Z80%20DAA/
    void DAA([[maybe_unused]] u8 instruction) {
        u8 value = r8.A;
        u8 correction{};
        if (flag.H || (!flag.N && (value & 0xf) > 9)) { correction = 0x6; }

        if (flag.C || (!flag.N && value > 0x99)) {
            correction |= 0x60;
            flag.C = true;
        }

        value += flag.N ? -correction : correction;
        flag.Z = value == 0;
        flag.H = false;
        r8.A = value;
    }
    void CPL_A([[maybe_unused]] u8 instruction) {
        r8.A = ~r8.A;
        flag.N = true;
        flag.H = true;
    }
    void SCF([[maybe_unused]] u8 instruction) {
        flag.N = false;
        flag.H = false;
        flag.C = true;
    }
    void CCF([[maybe_unused]] u8 instruction) {
        flag.N = false;
        flag.H = false;
        flag.C = !flag.C;
    }
    template <u8 dst_idx, u8 src_idx>
    void LD_r8_r8([[maybe_unused]] u8 instruction) {
        u8 val = ReadR8<src_idx>();
        WriteR8<dst_idx>(val);
    }
    void HALT([[maybe_unused]] u8 instruction) { timestamp = schedule.begin()->first; }
    template <u8 operation, u8 reg_idx>
    void ALU_A_r8([[maybe_unused]] u8 instruction) {
        u8 rhs = ReadR8<reg_idx>();
        ALU_OPS[operation](*this, rhs);
    }
    template <u8 condition>
    void RET_cond([[maybe_unused]] u8 instruction) {
        timestamp += 4;
        if (CheckCondition<condition>()) {
            GADDR braddr = PopWord();
            Jump(braddr);
        }
    }
    void LDIO_u8addr_A([[maybe_unused]] u8 instruction) {
        u8 offset = Imm8();
        Store(0xFF00 + offset, r8.A);
    }
    void ADD_SP_s8([[maybe_unused]] u8 instruction) {
        s8 rhs = static_cast<s8>(Imm8());
        flag.Z = false;
        flag.N = false;
        flag.H = ((SP & 0x0F) + (rhs & 0x0F)) & 0x10;
        flag.C = ((SP & 0xFF) + static_cast<u8>(rhs)) & 0x0100;
        SP += rhs;
        timestamp += 8;
    }
    void LDIO_A_u8addr([[maybe_unused]] u8 instruction) {
        u8 offset = Imm8();
        r8.A = Load(0xFF00 + offset);
    }
    void LD_HL_SP_s8([[maybe_unused]] u8 instruction) {

        s8 rhs = static_cast<s8>(Imm8());
        flag.Z = false;
        flag.N = false;
        flag.H = ((SP & 0x0F) + (rhs & 0x0F)) & 0x10;
        flag.C = ((SP & 0xFF) + static_cast<u8>(rhs)) & 0x0100;
        r16.HL = SP + rhs;
        timestamp += 4;
    }
    template <u8 reg_idx>
    void POP_r16([[maybe_unused]] u8 instruction) {
        WriteG3R16<reg_idx>(PopWord());
    };
    void RET([[maybe_unused]] u8 instruction) {
        GADDR braddr = PopWord();
        Jump(braddr);
    }
    void RETI([[maybe_unused]] u8 instruction) {
        GADDR braddr = PopWord();
        IME = true;
        Jump(braddr);
    }
    void JP_HL([[maybe_unused]] u8 instruction) {
        timestamp -= 4;
        Jump(r16.HL);
    }
    void LD_SP_HL([[maybe_unused]] u8 instruction) {
        SP = r16.HL;
        timestamp += 4;
    }
    template <u8 condition>
    void JP_cond_u16([[maybe_unused]] u8 instruction) {
        GADDR braddr = Imm16();
        if (CheckCondition<condition>()) { Jump(braddr); };
    }
    void LDIO_Caddr_A([[maybe_unused]] u8 instruction) { Store(0xFF00 + r8.C, r8.A); }
    void LD_u16addr_A([[maybe_unused]] u8 instruction) {
        GADDR addr = Imm16();
        Store(addr, r8.A);
    }
    void LDIO_A_Caddr([[maybe_unused]] u8 instruction) { r8.A = Load(0xFF00 + r8.C); }
    void LD_A_u16addr([[maybe_unused]] u8 instruction) {
        GADDR addr = Imm16();
        r8.A = Load(addr);
    }
    void JP_u16([[maybe_unused]] u8 instruction) {
        GADDR braddr = Imm16();
        Jump(braddr);
    }
    void CB_prefix([[maybe_unused]] u8 instruction) {
        u8 op = Imm8();
        CB_TABLE[op](*this, op);
    }
    void DI([[maybe_unused]] u8 instruction) { IME = false; }
    void EI([[maybe_unused]] u8 instruction) { IME = true; }
    template <u8 condition>
    void CALL_cond_u16([[maybe_unused]] u8 instruction) {
        GADDR braddr = Imm16();
        if (CheckCondition<condition>()) {
            PushWord(PC + 1);
            Jump(braddr);
        };
    }
    template <u8 reg_idx>
    void PUSH_r16([[maybe_unused]] u8 instruction) {
        PushWord(ReadG3R16<reg_idx>());
        timestamp += 4;
    }
    void CALL_u16([[maybe_unused]] u8 instruction) {
        GADDR braddr = Imm16();
        PushWord(PC + 1);
        Jump(braddr);
    }
    template <u8 operation>
    void ALU_A_u8([[maybe_unused]] u8 instruction) {
        u8 rhs = Imm8();
        ALU_OPS[operation](*this, rhs);
    }
    template <u8 vector>
    void RST([[maybe_unused]] u8 instruction) {
        u8 braddr = vector * 0x8;
        PushWord(PC + 1);
        Jump(braddr);
    }

    using Opcode = void (*)(Interpreter&, u8);

    template <void (Interpreter::*handler)(u8)>
    [[gnu::flatten]] static void TailCallInstr(Interpreter& state, u8 arg) {
        (state.*handler)(arg);
        if (state.timestamp < state.schedule.begin()->first) [[likely]] {
            u8 opcode = state.Imm8();
            [[clang::musttail]] return state.JUMP_TABLE[opcode](state, opcode);
        }
    }

    std::array<void*, 256> OPCODE_TABLE{};

    std::array<Opcode, 256> JUMP_TABLE = []() constexpr {
        std::array<Opcode, 256> table{};
        std::ranges::fill(table, TailCallInstr<&Interpreter::UnimplementedOpcode>);
        table[0b00000000] = TailCallInstr<&Interpreter::NOP>;
        table[0b00001000] = TailCallInstr<&Interpreter::LD_u16addr_SP>;
        // table[0b00010000] =  TailCallInstr< &Interpreter::STOP >;
        table[0b00011000] = TailCallInstr<&Interpreter::JR_s8>;
        TABLE_FILL(JR_cond_s8, 0b001'00'000, 3, 2);
        TABLE_FILL(LD_r16_u16, 0b00'00'0001, 4, 2);
        TABLE_FILL(ADD_HL_r16, 0b00'00'1001, 4, 2);
        TABLE_FILL(LD_r16addr_A, 0b00'00'0010, 4, 2);
        TABLE_FILL(LD_A_r16addr, 0b00'00'1010, 4, 2);
        TABLE_FILL(INC_r16, 0b00'00'0011, 4, 2);
        TABLE_FILL(DEC_r16, 0b00'00'1011, 4, 2);
        TABLE_FILL(INC_r8, 0b00'000'100, 3, 3);
        TABLE_FILL(DEC_r8, 0b00'000'101, 3, 3);
        TABLE_FILL(LD_r8_u8, 0b00'000'110, 3, 3);
        table[0b00'000'111] = TailCallInstr<&Interpreter::RLC_A>;
        table[0b00'001'111] = TailCallInstr<&Interpreter::RRC_A>;
        table[0b00'010'111] = TailCallInstr<&Interpreter::RL_A>;
        table[0b00'011'111] = TailCallInstr<&Interpreter::RR_A>;
        table[0b00'100'111] = TailCallInstr<&Interpreter::DAA>;
        table[0b00'101'111] = TailCallInstr<&Interpreter::CPL_A>;
        table[0b00'110'111] = TailCallInstr<&Interpreter::SCF>;
        table[0b00'111'111] = TailCallInstr<&Interpreter::CCF>;
        DOUBLE_TABLE_FILL(LD_r8_r8, 0b01'000'000, 3, 3, 0, 3);
        table[0b01'110'110] = TailCallInstr<&Interpreter::HALT>;
        DOUBLE_TABLE_FILL(ALU_A_r8, 0b10'000'000, 3, 3, 0, 3);
        TABLE_FILL(RET_cond, 0b110'00'000, 3, 2);
        table[0b11100000] = TailCallInstr<&Interpreter::LDIO_u8addr_A>;
        table[0b11101000] = TailCallInstr<&Interpreter::ADD_SP_s8>;
        table[0b11110000] = TailCallInstr<&Interpreter::LDIO_A_u8addr>;
        table[0b11111000] = TailCallInstr<&Interpreter::LD_HL_SP_s8>;
        TABLE_FILL(POP_r16, 0b11'00'0001, 4, 2);
        table[0b11'00'1001] = TailCallInstr<&Interpreter::RET>;
        table[0b11'01'1001] = TailCallInstr<&Interpreter::RETI>;
        table[0b11'10'1001] = TailCallInstr<&Interpreter::JP_HL>;
        table[0b11'11'1001] = TailCallInstr<&Interpreter::LD_SP_HL>;
        TABLE_FILL(JP_cond_u16, 0b110'00'010, 3, 2);
        table[0b11100010] = TailCallInstr<&Interpreter::LDIO_Caddr_A>;
        table[0b11101010] = TailCallInstr<&Interpreter::LD_u16addr_A>;
        table[0b11110010] = TailCallInstr<&Interpreter::LDIO_A_Caddr>;
        table[0b11111010] = TailCallInstr<&Interpreter::LD_A_u16addr>;
        table[0b11'000'011] = TailCallInstr<&Interpreter::JP_u16>;
        table[0b11'001'011] = TailCallInstr<&Interpreter::CB_prefix>;
        table[0b11'110'011] = TailCallInstr<&Interpreter::DI>;
        table[0b11'111'011] = TailCallInstr<&Interpreter::EI>;
        TABLE_FILL(CALL_cond_u16, 0b110'00'100, 3, 2);
        TABLE_FILL(PUSH_r16, 0b11'00'0101, 4, 2);
        table[0b11001101] = TailCallInstr<&Interpreter::CALL_u16>;
        TABLE_FILL(ALU_A_u8, 0b11'000'110, 3, 3);
        TABLE_FILL(RST, 0b11'000'111, 3, 3);
        return table;
    }();

    std::atomic<std::chrono::nanoseconds> speed;

    std::unique_ptr<Assembler> assembler;

public:
    Interpreter();
    virtual void Install(Bus& bus) override;
    virtual void Run() override;

    virtual std::chrono::nanoseconds GetSpeed() override { return speed; };
};

} // namespace CGB::Core::CPU

#undef TABLE_FILL
#undef DOUBLE_TABLE_FILL