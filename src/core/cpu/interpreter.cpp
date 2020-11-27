#include "interpreter.hpp"

namespace CGB::CPU {

void Interpreter::Install(Bus& bus) { this->bus = &bus; }
void Interpreter::Run() {
    PC = 0xFF;
    timestamp = 0;

    while (true) {
        u8 opcode = Imm8();
        (this->*JUMP_TABLE[opcode])();
    }
}
} // namespace CGB::CPU