#include <filesystem>
#include <memory>
#include <thread>

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include "common/logger.hpp"
#include "common/types.hpp"
#include "core/bus.hpp"
#include "core/cartridge/cartridge.hpp"
#include "core/cpu/cpu.hpp"
#include "core/ppu/ppu.hpp"
#include "core/ppu/renderer.hpp"
#include "vk_renderer_frontend.hpp"

int main(int argc, const char** argv) {
    std::span<const char*> args{argv, static_cast<std::size_t>(argc)};
    if (args.size() < 2) return 1;
    std::filesystem::path game = args[1];

    auto cartridge = std::make_unique<CGB::Core::CartridgeHeader>(game);
    LOG(Info, "Title: {}", cartridge->Title());
    LOG(Info, "Header Checksum: {:#04X} Hash: {:#04X}", cartridge->HeaderChecksum(),
        cartridge->HashHeader());
    LOG(Info, "ROM Checksum: {:#06X} Hash: {:#06X}", cartridge->GlobalChecksum(),
        cartridge->HashROM());
    LOG(Info, "CartridgeType {:#04X}", cartridge->CartridgeType(), cartridge->HashROM());

    auto _render_frontend = std::make_unique<CGB::Frontend::SDL2::SDL2_VK_Frontend>();
    auto render_frontend = _render_frontend.get();
    std::function<std::chrono::nanoseconds()> cpu_speed_callback;
    std::function<std::chrono::nanoseconds()> renderer_speed_callback;
    std::thread emuthread{
        [&](auto cartridge_header, auto frontend) {
            auto renderer = std::make_unique<CGB::Core::Renderer>(std::move(frontend));
            renderer_speed_callback = [renderer = renderer.get()]() {
                return renderer->GetSpeed();
            };
            CGB::Core::Bus bus{std::move(cartridge_header), CGB::Core::CPU::MainCPU{},
                               std::make_unique<CGB::Core::PPU>(std::move(renderer))};
            cpu_speed_callback = [&cpu = bus.GetCPU()]() { return cpu.GetSpeed(); };
            bus.GetCPU().Run();
        },
        std::move(cartridge), std::move(_render_frontend)};
    std::chrono::nanoseconds cpu_speed, renderer_speed;
    CGB::u64 perf_count{0};
    while (true) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT: std::exit(0);
            default: break;
            }
        }
        if (cpu_speed_callback) cpu_speed += cpu_speed_callback();
        if (renderer_speed_callback) renderer_speed += renderer_speed_callback();
        ++perf_count;
        constexpr unsigned perf_interval = 200;
        if (perf_count == perf_interval) {
            perf_count = 0;
            cpu_speed /= perf_interval;
            renderer_speed /= perf_interval;
            auto title = fmt::format(FMT_STRING("CPU: {:.1f}fps PPU: {}"),
                                     1 / std::chrono::duration<double>{cpu_speed}.count(),
                                     std::chrono::duration<double, std::micro>{renderer_speed});
            SDL_SetWindowTitle(render_frontend->GetWindow(), title.data());
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}