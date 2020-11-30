#include "cpu.hpp"

#include "core/bus.hpp"
#include "cpu.hpp"
#include "interpreter.hpp"

namespace CGB::Core::CPU {
MainCPU::MainCPU() = default;
MainCPU::MainCPU(MainCPU&&) = default;
MainCPU::~MainCPU() = default;
MainCPU& MainCPU::operator=(MainCPU&&) = default;

void MainCPU::Install(Bus& bus) {
    LOG(Info, "Installing CPU onto bus");
    impl = std::make_unique<Interpreter>();
    impl->Install(bus);
}

void MainCPU::Run() { impl->Run(); }

} // namespace CGB::CPU