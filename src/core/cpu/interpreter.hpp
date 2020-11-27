#pragma once

#include "common/types.hpp"
#include "core/bus.hpp"

namespace CGB::CPU {

class Interpreter {
    union {
        std::array<u8, 8> reg_arr;
        struct {
            u8 B;
            u8 C;
            u8 D;
            u8 E;
            u8 H;
            u8 L;
            u8 HL;
            u8 A;
        } r8;
        struct {
            u16 BC;
            u16 DE;
            u16 HL;
            u16 FA;
        } r16;
    };
    [[maybe_unused]] u16 SP;
    u16 PC;
    struct {
        bool Z, N, H, C;
    } flag;

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
    static constexpr std::array<std::string_view, 4> CONDITION_NAME{
        "NZ",
        "Z",
        "NC",
        "C",
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

    template <u8 reg_idx>
    u8 FetchR8() {
        if constexpr (reg_idx == 6) {
            return Load(r16.HL);
        } else {
            return reg_arr[reg_idx];
        }
    }

    template <u8 reg_idx>
    void StoreR8(u8 val) {
        if constexpr (reg_idx == 6) {
            Store(r16.HL, val);
        } else {
            reg_arr[reg_idx] = val;
        }
    }

    template <u8 reg_idx>
    u16 FetchG1R16() {
        switch (reg_idx) {
        case 0: return r16.BC;
        case 1: return r16.DE;
        case 2: return r16.HL;
        case 3: return SP;
        }
    }

    template <u8 reg_idx>
    void StoreG1R16(u16 val) {
        switch (reg_idx) {
        case 0: r16.BC = val; break;
        case 1: r16.DE = val; break;
        case 2: r16.HL = val; break;
        case 3: SP = val; break;
        }
    }

    template <u8 reg_idx>
    u16 FetchG2R16() {
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

    void UnimplementedOpcode() {
        LOG(Trace, "{:#06X} Unimplemented Opcode {:#010B}", PC, bus->Read(PC, timestamp));
    }
    void NOP() { LOG(Trace, "\t\t{:#06X} NOP", PC); }
    template <u8 condition>
    void JR_conditional_imm8() {
        s8 offset = static_cast<s8>(Imm8());
        LOG(Trace, "\t{:#06X} JR {},\t{:#04X}", PC - 1, CONDITION_NAME[condition], offset);
        if (CheckCondition<condition>()) {
            PC += offset;
            timestamp += 4;
        }
    }
    template <u8 reg_idx>
    void LD_r16_imm16() {
        u16 rhs = Imm16();
        LOG(Trace, "\t{:#06X} LD\t{},\t{:#06X}", PC - 2, G1_R16_NAME[reg_idx], rhs);
        StoreG1R16<reg_idx>(rhs);
    }
    template <u8 reg_idx>
    void LD_A_r16addr() {
        LOG(Trace, "\t{:#06X} LD\tA,\t[{}]", PC, G2_R16_NAME[reg_idx]);
        r8.A = Load(FetchG2R16<reg_idx>());
    }
    template <u8 reg_idx>
    void LD_r16addr_A() {
        LOG(Trace, "\t{:#06X} LD\t[{}],\tA", PC, G2_R16_NAME[reg_idx]);
        Store(FetchG2R16<reg_idx>(), r8.A);
    }
    template <u8 reg_idx>
    void INC_r8() {
        LOG(Trace, "\t\t{:#06X} INC {}", PC, R8_NAME[reg_idx]);
        u8 val = FetchR8<reg_idx>();
        flag.H = (val & 0x0F) == 0xF;
        ++val;
        flag.Z = val == 0;
        flag.N = false;
        StoreR8<reg_idx>(val);
    }
    template <u8 reg_idx>
    void DEC_r8() {
        LOG(Trace, "\t\t{:#06X} DEC {}", PC, R8_NAME[reg_idx]);
        u8 val = FetchR8<reg_idx>();
        // TODO: verify half-borrow behavior
        flag.H = (val & 0x0F) == 0x00;
        --val;
        flag.Z = val == 0;
        flag.N = true;
        StoreR8<reg_idx>(val);
    }
    template <u8 reg_idx>
    void LD_r8_imm8() {
        u8 rhs = Imm8();
        LOG(Trace, "\t{:#06X} LD\t{},\t{:#04X}", PC - 1, R8_NAME[reg_idx], rhs);
        StoreR8<reg_idx>(rhs);
    }
    template <u8 dst_idx, u8 src_idx>
    void LD_r8_r8() {
        LOG(Trace, "\t\t{:#06X} LD\t{},\t{}", PC, R8_NAME[dst_idx], R8_NAME[src_idx]);
        StoreR8<dst_idx>(FetchR8<src_idx>());
    }
    void JP_imm16() {
        GADDR braddr = Imm16();
        LOG(Trace, "\t\t{:#06X} JP\t{:#06X}", PC - 2, braddr);
        timestamp += 8;
        PC = braddr - 1;
    }
    void ADC_A(u8 rhs) {
        flag.H = __builtin_addcb(r8.A & 0x0F, rhs & 0x0F, flag.C, nullptr) & 0x10;
        u8 carry;
        r8.A = __builtin_addcb(r8.A & 0x0F, rhs & 0x0F, flag.C, &carry);
        flag.C = carry;
        flag.Z = r8.A == 0;
        flag.N = false;
    }
    void ADC_A_imm8() {
        u8 rhs = Imm8();
        LOG(Trace, "\t\t{:#06X} ADC A,\t{:#04X}", PC - 1, rhs);
        ADC_A(rhs);
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
        TABLE_FILL(JR_conditional_imm8, 0b001'00'000, 3, 2);
        TABLE_FILL(LD_r16_imm16, 0b00'00'0001, 4, 2);
        TABLE_FILL(LD_r16addr_A, 0b00'00'0010, 4, 2);
        TABLE_FILL(LD_A_r16addr, 0b00'00'1010, 4, 2);
        TABLE_FILL(INC_r8, 0b00'000'100, 3, 3);
        TABLE_FILL(DEC_r8, 0b00'000'101, 3, 3);
        TABLE_FILL(LD_r8_imm8, 0b00'000'110, 3, 3);
        DOUBLE_TABLE_FILL(LD_r8_r8, 0b01'000'000, 3, 3, 0, 3);
        table[0b11'000'011] = &Interpreter::JP_imm16;
        table[0b11'001'110] = &Interpreter::ADC_A_imm8;

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