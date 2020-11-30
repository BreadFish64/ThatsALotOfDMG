#include "interpreter.hpp"

namespace CGB::Core::CPU {

void Interpreter::Install(Bus& bus) {
    this->bus = &bus;
}
void Interpreter::Run() {
    PC = 0xFF;
    timestamp = 0;
    run = true;

    start = std::chrono::high_resolution_clock::now();

    while (run) {
        u8 opcode = Imm8();
        (this->*JUMP_TABLE[opcode])();
    }
}

} // namespace CGB::CPU