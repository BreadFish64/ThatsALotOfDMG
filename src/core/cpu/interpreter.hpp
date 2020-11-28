#pragma once

#include "common/types.hpp"
#include "core/bus.hpp"

namespace CGB::CPU {

class Interpreter {
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
    u16 SP;
    u16 PC;
    struct {
        bool Z, N, H, C;
    } flag;
    bool IME = true;

    u64 timestamp;

    Bus* bus;

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

    u8 Load(GADDR addr) {
        u8 val = bus->Read(addr, timestamp);
        timestamp += 4;
        return val;
    }

    void Store(GADDR addr, u8 val) {
        bus->Write(addr, val, timestamp);
        timestamp += 4;
    }

    u8 Imm8() { return Load(++PC); }

    u16 Imm16() { return u16{Imm8()} | (u16{Imm8()} << 8); }

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
        }
    }

    template <u8 reg_idx>
    void WriteG1R16(u16 val) {
        switch (reg_idx) {
        case 0: r16.BC = val; break;
        case 1: r16.DE = val; break;
        case 2: r16.HL = val; break;
        case 3: SP = val; break;
        }
    }

    template <u8 reg_idx>
    u16 ReadG2R16() {
        switch (reg_idx) {
        case 0: return r16.BC;
        case 1: return r16.DE;
        case 2: return r16.HL++;
        case 3: return r16.HL--;
        }
    }

    template <u8 condition>
    bool CheckCondition() {
        switch (condition) {
        case 0: return !flag.Z;
        case 1: return flag.Z;
        case 2: return !flag.C;
        case 3: return flag.C;
        default: break;
        }
    }

    template <u8 reg_idx>
    u16 ReadG3R16() {
        switch (reg_idx) {
        case 0: return r16.BC;
        case 1: return r16.DE;
        case 2: return r16.HL;
        case 3: return (u16{r8.A} << 8) | PackFlags();
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
            UnpackFlags(val);
            break;
        }
    }

    void Jump(GADDR braddr) {
        timestamp += 4;
        PC = braddr - 1;
    }
    void PushByte(u8 val) { Store(--SP, val); }
    void PushWord(u16 val) {
        PushByte(static_cast<u8>(val));
        PushByte(static_cast<u8>(val >> 8));
    }
    u8 PopByte() { return Load(SP++); }
    u16 PopWord() { return (u16{PopByte()} << 8) | PopByte(); }

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
        flag.H = __builtin_addcb(lhs & 0x0F, rhs & 0x0F, flag.C, nullptr) & 0x10;
        u8 carry;
        u8 result = __builtin_addcb(lhs, rhs, flag.C, &carry);
        flag.C = carry;
        flag.Z = result == 0;
        flag.N = false;
        r8.A = result;
    }
    void SUB(u8 rhs) {
        u8 lhs = r8.A;
        flag.H = static_cast<s8>((lhs & 0x0F) - (rhs & 0x0F)) < 0;
        u8 result;
        flag.C = __builtin_sub_overflow(lhs, rhs, &result);
        flag.Z = result == 0;
        flag.N = true;
        r8.A = result;
    }
    void SBC(u8 rhs) {
        u8 lhs = r8.A;
        flag.H = static_cast<s8>(__builtin_subcb(lhs & 0x0F, rhs & 0x0F, flag.C, nullptr)) < 0;
        u8 carry;
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
        flag.H = static_cast<s8>((lhs & 0x0F) - (rhs & 0x0F)) < 0;
        u8 result;
        flag.C = __builtin_sub_overflow(lhs, rhs, &result);
        flag.Z = result == 0;
        flag.N = true;
    }

    static constexpr std::array ALU_OPS{
        &Interpreter::ADD, &Interpreter::ADC, &Interpreter::SUB, &Interpreter::SBC,
        &Interpreter::AND, &Interpreter::XOR, &Interpreter::OR,  &Interpreter::CP,
    };

    void UnimplementedOpcode() {
        LOG(Trace, "{:#06X} Unimplemented Opcode {:#010B}", PC, bus->Read(PC, timestamp));
    }
    void NOP() { LOG(Trace, "\t\t{:#06X} NOP", PC); }
    void JR_PCrel() {
        s8 offset = static_cast<s8>(Imm8());
        LOG(Trace, "\t\t{:#06X} JR\t{:#04X}", PC - 1, offset);
        Jump(PC + 1 + offset);
    }
    template <u8 condition>
    void JR_cond_PCrel() {
        s8 offset = static_cast<s8>(Imm8());
        LOG(Trace, "\t{:#06X} JR\t{},\t{:#04X}", PC - 1, CONDITION_NAME[condition], offset);
        if (CheckCondition<condition>()) { Jump(PC + 1 + offset); }
    }
    template <u8 reg_idx>
    void LD_r16_u16() {
        u16 imm = Imm16();
        WriteG1R16<reg_idx>(imm);
        LOG(Trace, "\t{:#06X} LD\t{},\t{:#06X}", PC - 2, G1_R16_NAME[reg_idx], imm);
    }
    template <u8 reg_idx>
    void LD_A_r16addr() {
        GADDR addr = ReadG2R16<reg_idx>();
        u8 val = Load(addr);
        r8.A = val;
        LOG(Trace, "\t{:#06X} LD\tA,\t[{}]", PC, G2_R16_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void LD_r16addr_A() {
        GADDR addr = ReadG2R16<reg_idx>();
        u8 val = r8.A;
        Store(addr, val);
        LOG(Trace, "\t{:#06X} LD\t[{}],\tA", PC, G2_R16_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void INC_r16() {
        WriteG1R16<reg_idx>(ReadG1R16<reg_idx>() + 1);
        timestamp += 4;
        LOG(Trace, "\t\t{:#06X} INC\t{}", PC, G1_R16_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void DEC_r16() {
        WriteG1R16<reg_idx>(ReadG1R16<reg_idx>() - 1);
        timestamp += 4;
        LOG(Trace, "\t\t{:#06X} INC\t{}", PC, G1_R16_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void INC_r8() {
        u8 val = ReadR8<reg_idx>();
        flag.H = (val & 0x0F) == 0xF;
        ++val;
        flag.Z = val == 0;
        flag.N = false;
        WriteR8<reg_idx>(val);
        LOG(Trace, "\t\t{:#06X} INC\t{}", PC, R8_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void DEC_r8() {
        u8 val = ReadR8<reg_idx>();
        flag.H = (val & 0x0F) == 0x00;
        --val;
        flag.Z = val == 0;
        flag.N = true;
        WriteR8<reg_idx>(val);
        LOG(Trace, "\t\t{:#06X} DEC\t{}", PC, R8_NAME[reg_idx]);
    }
    template <u8 reg_idx>
    void LD_r8_u8() {
        u8 rhs = Imm8();
        LOG(Trace, "\t\t{:#06X} LD\t{},\t{:#04X}", PC - 1, R8_NAME[reg_idx], rhs);
        WriteR8<reg_idx>(rhs);
    }
    template <u8 dst_idx, u8 src_idx>
    void LD_r8_r8() {
        u8 val = ReadR8<src_idx>();
        WriteR8<dst_idx>(val);
        LOG(Trace, "\t\t{:#06X} LD\t{},\t{}", PC, R8_NAME[dst_idx], R8_NAME[src_idx]);
    }
    template <u8 operation, u8 reg_idx>
    void ALU_A_r8() {
        u8 rhs = ReadR8<reg_idx>();
        (this->*ALU_OPS[operation])(rhs);
        LOG(Trace, "\t\t{:#06X} {}\tA,\t{}", PC, ALU_OP_NAME[operation], R8_NAME[reg_idx]);
    }
    template <u8 rhs_idx>
    void OR_A_r8() {
        u8 rhs = ReadR8<rhs_idx>();
        WriteR8<dst_idx>(val);
        LOG(Trace, "\t\t{:#06X} LD\t{},\t{}", PC, R8_NAME[dst_idx], R8_NAME[src_idx]);
    }
    void LDIO_u8addr_A() {
        u8 offset = Imm8();
        Store(0xFF00 + offset, r8.A);
        LOG(Trace, "\t{:#06X} LDIO\t[{:#04X}],\tA", PC, offset);
    }
    void LDIO_A_u8addr() {
        u8 offset = Imm8();
        r8.A = Load(0xFF00 + offset);
        LOG(Trace, "\t{:#06X} LDIO\tA,\t[{:#04X}]", PC, offset);
    }
    template <u8 reg_idx>
    void POP_r16() {
        WriteG3R16<reg_idx>(PopWord());
        LOG(Trace, "\t\t{:#06X} POP\t{}", PC, G3_R16_NAME[reg_idx]);
    };
    void RET() {
        GADDR braddr = PopWord();
        LOG(Trace, "\t\t{:#06X} RET", PC);
        Jump(braddr);
    }
    void LD_u16addr_A() {
        GADDR addr = Imm16();
        Store(addr, r8.A);
        LOG(Trace, "\t{:#06X} LD\t[{:#06X}],\tA", PC - 2, addr);
    }
    void LD_A_u16addr() {
        GADDR addr = Imm16();
        r8.A = Load(addr);
        LOG(Trace, "\t{:#06X} LD\tA,\t[{:#06X}]", PC - 2, addr);
    }
    void JP_u16() {
        GADDR braddr = Imm16();
        LOG(Trace, "\t\t{:#06X} JP\t{:#06X}", PC - 2, braddr);
        Jump(braddr);
    }
    template <u8 condition>
    void CALL_cond_u16() {
        GADDR braddr = Imm16();
        LOG(Trace, "\t{:#06X} CALL\t{},\t{:#06X}", PC - 2, CONDITION_NAME[condition], braddr);
        if (CheckCondition<condition>()) {
            PushWord(PC + 1);
            Jump(braddr);
        };
    }
    template <u8 reg_idx>
    void PUSH_r16() {
        PushWord(ReadG3R16<reg_idx>());
        timestamp += 4;
        LOG(Trace, "\t\t{:#06X} PUSH\t{}", PC, G3_R16_NAME[reg_idx]);
    };
    void CALL_u16() {
        GADDR braddr = Imm16();
        PushWord(PC + 1);
        LOG(Trace, "\t\t{:#06X} CALL\t{:#06X}", PC - 2, braddr);
        Jump(braddr);
    }
    void DI() {
        IME = false;
        LOG(Trace, "\t\t{:#06X} DI", PC);
    }
    template <u8 operation>
    void ALU_A_u8() {
        u8 rhs = Imm8();
        (this->*ALU_OPS[operation])(rhs);
        LOG(Trace, "\t\t{:#06X} {}\tA,\t{:#04X}", PC - 1, ALU_OP_NAME[operation], rhs);
    }
    using Opcode = decltype(&Interpreter::NOP);

    static constexpr std::array<Opcode, 256> JUMP_TABLE = []() constexpr {

        // These macros were a mistake and I'm sorry to anyone trying to understand them
#define TABLE_FILL(operation, base, shift, bits)                                                   \
    [&table]<u8... idx>(u8 _base, u8 _shift, std::integer_sequence<u8, idx...>) {                  \
        ((table[_base | (idx << _shift)] = &Interpreter::operation<idx>), ...);                    \
    }                                                                                              \
    (base, shift, std::make_integer_sequence<u8, 1 << bits>());
#define DOUBLE_TABLE_FILL(operation, base, shift1, bits1, shift2, bits2)                           \
    [&table]<u8... idx1, u8... idx2>(u8 _base, u8 _shift1, std::integer_sequence<u8, idx1...>,     \
                                     u8 _shift2, std::integer_sequence<u8, idx2...> count2) {      \
        const auto helper = [&table]<u8 left, u8... right>(u8 _base, u8 _shift,                    \
                                                           std::integer_sequence<u8, right...>) {  \
            ((table[_base | (right << _shift)] = &Interpreter::operation<left, right>), ...);      \
        };                                                                                         \
        ((helper.template operator()<idx1>(_base | (idx1 << _shift1), _shift2, count2)), ...);     \
    }                                                                                              \
    (base, shift1, std::make_integer_sequence<u8, 1 << bits1>(), shift2,                           \
     std::make_integer_sequence<u8, 1 << bits2>());

        std::array<Opcode, 256> table{};
        std::ranges::fill(table, &Interpreter::UnimplementedOpcode);
        table[0b00000000] = &Interpreter::NOP;
        // table[0b00001000] = &Interpreter::LD_u16_SP;
        // table[0b00010000] = &Interpreter::STOP;
        table[0b00011000] = &Interpreter::JR_PCrel;
        TABLE_FILL(JR_cond_PCrel, 0b001'00'000, 3, 2);
        TABLE_FILL(LD_r16_u16, 0b00'00'0001, 4, 2);
        // TABLE_FILL(ADD_HL_r16, 0b00'00'1001, 4, 2);
        TABLE_FILL(LD_r16addr_A, 0b00'00'0010, 4, 2);
        TABLE_FILL(LD_A_r16addr, 0b00'00'1010, 4, 2);
        TABLE_FILL(INC_r16, 0b00'00'0011, 4, 2);
        TABLE_FILL(DEC_r16, 0b00'00'1011, 4, 2);
        TABLE_FILL(INC_r8, 0b00'000'100, 3, 3);
        TABLE_FILL(DEC_r8, 0b00'000'101, 3, 3);
        TABLE_FILL(LD_r8_u8, 0b00'000'110, 3, 3);
        DOUBLE_TABLE_FILL(LD_r8_r8, 0b01'000'000, 3, 3, 0, 3);
        table[0b01'110'110] = &Interpreter::UnimplementedOpcode; // HALT
        DOUBLE_TABLE_FILL(ALU_A_r8, 0b10'000'000, 3, 3, 0, 3);
        table[0b11100000] = &Interpreter::LDIO_u8addr_A;
        // table[0b11101000] = &Interpreter::ADD_SP_s8;
        table[0b11110000] = &Interpreter::LDIO_A_u8addr;
        // table[0b11110000] = &Interpreter::LD_HL_SPrel;
        TABLE_FILL(POP_r16, 0b11'00'0001, 4, 2);
        table[0b11'00'1001] = &Interpreter::RET;
        // table[0b11'01'1001] = &Interpreter::RETI;
        // table[0b11'10'1001] = &Interpreter::JP_HL;
        // table[0b11'11'1001] = &Interpreter::LD_SP_HL;
        // table[0b110'00'010] = &Interpreter::JP_cond_u16;
        // table[0b11100010] = &Interpreter::LDIO_Caddr_A
        table[0b11101010] = &Interpreter::LD_u16addr_A;
        // table[0b11110010] = &Interpreter::LDIO_A_Caddr
        table[0b11111010] = &Interpreter::LD_A_u16addr;
        table[0b11'000'011] = &Interpreter::JP_u16;
        // table[0b11'001'011] = &Interpreter::CB;
        table[0b11'110'011] = &Interpreter::DI;
        // table[0b11'111'011] = &Interpreter::EI;
        TABLE_FILL(CALL_cond_u16, 0b110'00'100, 3, 2);
        TABLE_FILL(PUSH_r16, 0b11'00'0101, 4, 2);
        table[0b11001101] = &Interpreter::CALL_u16;
        TABLE_FILL(ALU_A_u8, 0b11'000'110, 3, 3);
        // TABLE_FILL(RST, 0b11'000'111, 3, 3);

#undef TABLE_FILL
#undef DOUBLE_TABLE_FILL
        return table;
    }
    ();

public:
    void Install(Bus& bus);
    void Run();
};

} // namespace CGB::CPU