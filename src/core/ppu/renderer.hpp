#pragma once

#include <vector>

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
            return std::span<u32, PPU::WIDTH>{raw.data() + scanline * 256, PPU::WIDTH};
        }
    };

    struct LcdRegs {
        u8 control{0x91};
        u8 stat{};
        u8 scy{};
        u8 scx{};
        u8 ly{};
        u8 lyc{};
        u8 _dma{};
        u8 bgp{0xFC};
        u8 obp0{0xFF};
        u8 obp1{0xFF};
        u8 wy{};
    } lcd;
    PPU::OAM oam;
    std::vector<u8> vram = std::vector<u8>(0x4000);

    void RecieveLCDWrite(PPU::FrameWrites::LCDWrite lcd_write);
    void RecieveOAMWrite(PPU::FrameWrites::OAM_DMA oam_write);
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

    std::array<u32, 4> GetBGP() {
        static constexpr std::array<u32, 4> palette{0xFF'FF'FF'FF, 0xFF'AA'AA'AA, 0xFF'53'53'53,
                                                    0xFF'00'00'00};
        return {palette[(lcd.bgp >> 0) & 0b11], palette[(lcd.bgp >> 2) & 0b11],
                palette[(lcd.bgp >> 4) & 0b11], palette[(lcd.bgp >> 6) & 0b11]};
    }
    unsigned GetBGTileIndex(unsigned x, unsigned y) {
        usize idx = y * 32 + x;
        idx += (lcd.control & 0x08) * (0x100 / 0x08);
        return vram[0x1800 + idx];
    }
    auto GetTile(unsigned index) {
        usize idx = index * 16;
        if (!(lcd.control & 0x10) && index < 0x80) idx += 0x1000;
        return std::span<const u16, 8>{reinterpret_cast<const u16*>(vram.data() + idx), 8};
    }
    void RenderBGSpriteRow(std::span<u32, 8> pixels, u16 tile_row,
                           const std::array<u32, 4>& palette);
};

} // namespace CGB::Core