#pragma once

#include <vector>

#include <immintrin.h>

#include <boost/lockfree/spsc_queue.hpp>

#include "common/types.hpp"
#include "ppu.hpp"
#include "vulkan/vk_frontend.hpp"

namespace CGB::Core {

class Renderer {
public:
    Renderer(std::unique_ptr<VulkanFrontend> frontend);
    ~Renderer();

    void RecieveFrameWrites(PPU::FrameWrites frame_writes) {
        // TODO: not busy wait here
        while (!frame_write_queue.write_available()) { std::this_thread::yield(); }
        frame_write_queue.push(std::move(frame_writes));
    };

    std::chrono::nanoseconds GetSpeed() { return speed; }

private:
    std::atomic<std::chrono::nanoseconds> speed;

    struct Frame {
        std::array<u32, 256 * 256> raw;
        auto Scanline(usize scanline) {
            return std::span<u32, PPU::WIDTH>{raw.data() + scanline * 256 + 256, PPU::WIDTH};
        }
    };

    PPU::LcdRegs lcd;
    struct Object {
        u8 ypos{PPU::HEIGHT};
        u8 xpos{};
        u8 tile{};
        u8 flags{};
    };
    std::array<Object, 0xA0 / sizeof(Object)> oam;
    std::vector<u8> vram = std::vector<u8>(0x4000);

    void RecieveLCDWrite(PPU::FrameWrites::LCDWrite lcd_write);
    void RecieveOAMWrite(PPU::FrameWrites::OAMWrite oam_write);
    void RecieveVRAMWrite(PPU::FrameWrites::VRAMWrite lcd_write);

    class Presenter {
        std::span<Frame, 3> frames{static_cast<Frame*>(nullptr), 3};
        s8 current_frame{1};
        std::atomic<s8> pending_frame{-2};
        s8 presenting_frame{3};

        std::unique_ptr<VulkanFrontend> frontend;
        vk::PhysicalDevice physical_device;
        std::uint32_t queue_family_index;
        vk::Queue queue;
        vk::UniqueDevice device;
        vk::UniqueCommandPool cmd_pool;

        vk::UniqueSwapchainKHR swapchain;
        std::vector<vk::Image> swapchain_images;

        vk::UniqueSemaphore image_available_semaphore;
        vk::UniqueSemaphore blit_finished_semaphore;
        vk::UniqueFence blit_fence;

        std::array<vk::UniqueImage, 3> staging_images;
        vk::UniqueDeviceMemory staging_buffer_mem;

        std::thread presentation_thread;
        std::atomic<bool> stop{false};
        void Present();

    public:
        Presenter(std::unique_ptr<VulkanFrontend> frontend);
        ~Presenter();

        Frame& GetCurrentFrame() { return frames[current_frame - 1]; }
        void PushFrame() {
            auto pending = pending_frame.exchange(current_frame);
            current_frame = pending < 0 ? -pending : pending;
        }
    } presenter;

    boost::lockfree::spsc_queue<PPU::FrameWrites, boost::lockfree::capacity<8>> frame_write_queue;
    std::atomic<bool> stop{false};
    std::thread render_thread;

    void Run();
    void SkipFrame(PPU::FrameWrites frame_writes);
    void RenderFrame(PPU::FrameWrites frame_writes);

    void RenderBGScanline(unsigned scanline, std::span<u32, PPU::WIDTH>& buffer);
    
    static std::array<u32, 4> GetPalette(u8 reg) {
        static constexpr std::array<u32, 4> palette{0xFF'FF'FF'FF, 0xFF'AA'AA'AA, 0xFF'53'53'53,
                                                    0xFF'00'00'00};
        return {palette[(reg >> 0) & 0b11], palette[(reg >> 2) & 0b11], palette[(reg >> 4) & 0b11],
                palette[(reg >> 6) & 0b11]};
    }
    unsigned GetBGTileIndex(unsigned x, unsigned y) {
        usize idx = y * 32 + x;
        idx += (lcd.control & 0x08) * (0x100 / 0x08);
        return vram[0x1800 + idx];
    }
    auto GetBGTile(unsigned index) {
        usize idx = index * 16;
        if (!(lcd.control & 0x10) && index < 0x80) idx += 0x1000;
        return std::span<const u16, 8>{reinterpret_cast<const u16*>(vram.data() + idx), 8};
    }
    auto GetSpriteTile(unsigned index) {
        usize idx = index * 16;
        return std::span<const u16, 8>{reinterpret_cast<const u16*>(vram.data() + idx), 8};
    }
    static __m256i DecodeTile(u16 tile_row);
    void RenderBGSpriteRow(std::span<u32, 8> pixels, u16 tile_row,
                           __m256i palette);
};

} // namespace CGB::Core