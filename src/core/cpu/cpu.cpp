#include "cpu.hpp"

#include "core/bus.hpp"
#include "cpu.hpp"
#include "interpreter.hpp"

namespace CGB::CPU {
BaseCPU::BaseCPU() = default;
BaseCPU::BaseCPU(BaseCPU&&) = default;
BaseCPU::~BaseCPU() = default;
BaseCPU& BaseCPU::operator=(BaseCPU&&) = default;

void BaseCPU::Install(Bus& bus) {
    LOG(Info, "Installing CPU onto bus");
    interpreter = std::make_unique<Interpreter>();
    interpreter->Install(bus);
}

void BaseCPU::Run() { interpreter->Run(); }

} // namespace CGB::CPU