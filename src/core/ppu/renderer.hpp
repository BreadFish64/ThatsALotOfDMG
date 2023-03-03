#pragma once
#pragma once

#include <bitset>
#include <semaphore>
#include <vector>

#include <immintrin.h>

#include "common/types.hpp"
#include "ppu.hpp"
#include "vulkan/vk_frontend.hpp"

namespace CGB::Core {

class Renderer {
public:
    Renderer(std::unique_ptr<VulkanFrontend> frontend);
    ~Renderer();

    void RecieveFrameWrites(PPU::FrameWrites frame_writes) {
        frames_free.acquire();
        *push_iter = std::move(frame_writes);
        if (frame_write_queue.end() == ++push_iter) { push_iter = frame_write_queue.begin(); }
        frames_queued.release();
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

        [[nodiscard]] bool HighPriorityBG() const { return flags & (1 << 7); }
        [[nodiscard]] bool YFlip() const { return flags & (1 << 6); }
        [[nodiscard]] bool XFlip() const { return flags & (1 << 5); }
        [[nodiscard]] bool SecondPallete() const { return flags & (1 << 4); }
    };
    std::array<Object, 0xA0 / sizeof(Object)> oam;
    std::vector<u8> vram = std::vector<u8>(0x4000);
    std::bitset<0x2000> tile_row_dirty;
    std::vector<__m256i> cached_tile_rows = std::vector<__m256i>(0x2000);

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

    static constexpr usize FRAME_QUEUE_SIZE = 8;
    using FrameQueue = std::array<PPU::FrameWrites, FRAME_QUEUE_SIZE>;
    FrameQueue frame_write_queue{};
    FrameQueue::iterator push_iter;
    FrameQueue::iterator pop_iter;
    std::counting_semaphore<FRAME_QUEUE_SIZE> frames_queued;
    std::counting_semaphore<FRAME_QUEUE_SIZE> frames_free;

    //bool skip_frames = false;
    std::atomic<bool> stop{false};
    std::thread render_thread;

    void Run();
    void SkipFrame(PPU::FrameWrites& frame_writes);
    void RenderFrame(PPU::FrameWrites& frame_writes);

    void RenderBGScanline(unsigned scanline, u32* color_buffer, s32* priority_buffer);

    static __m128i GetPalette(u8 reg) {
        /*alignas(__m256i) static constexpr std::array<u32, 4> pallet_rgba{0xFF'FF'FF'FFu,
           0xFF'AA'AA'AAu, 0xFF'53'53'53u, 0xFF'00'00'00u};*/
        auto palette_vec =
            _mm_set_epi32(0xFF'FF'FF'FFu, 0xFF'AA'AA'AAu, 0xFF'53'53'53u, 0xFF'00'00'00u);
        auto permutation = _mm_and_si128(
            _mm_srlv_epi32(_mm_set1_epi32(reg), _mm_set_epi32(0, 2, 4, 6)), _mm_set1_epi32(0b11));
        return _mm256_extracti128_si256(
            _mm256_permutevar8x32_epi32(_mm256_set_m128i(_mm_setzero_si128(), palette_vec),
                                        _mm256_set_m128i(_mm_setzero_si128(), permutation)),
            0);
    }
    unsigned GetBGTileIndex(unsigned x, unsigned y) {
        usize idx = y * 32_sz + x;
        idx += (lcd.control & 0x08u) * (0x100_sz / 0x08_sz);
        return vram[0x1800_sz + idx];
    }
    usize GetBGTileOffset(unsigned index) {
        usize offset = index * 16_sz;
        if (!(lcd.control & 0x10) && index < 0x80) offset += 0x1000;
        return offset;
    }
    usize GetSpriteTileOffset(unsigned index) { return index * 16_sz;
    }
    __m256i GetTileRow(usize tile_offset, unsigned row);
    void RenderBGSpriteRow(u32* color_buffer, s32* priority_buffer,
                           usize tile_vram_offset, unsigned row, __m256i palette);
};

} // namespace CGB::Core