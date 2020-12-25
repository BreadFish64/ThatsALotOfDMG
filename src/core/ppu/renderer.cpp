#include <algorithm>

#include "common/logger.hpp"
#include "renderer.hpp"

namespace CGB::Core {

void Renderer::RecieveLCDWrite(PPU::FrameWrites::LCDWrite lcd_write) {
    auto [idx, val, scanline] = lcd_write;
    std::memcpy(reinterpret_cast<u8*>(&lcd) + idx, &val, 1);
}

void Renderer::RecieveOAMWrite(PPU::FrameWrites::OAMWrite oam_write) {
    auto [offset, val, scanline] = oam_write;
    std::memcpy(reinterpret_cast<u8*>(&oam) + offset, &val, 1);
}

void Renderer::RecieveVRAMWrite(PPU::FrameWrites::VRAMWrite vram_write) {
    auto [addr, val, scanline] = vram_write;
    vram[addr - 0x8000] = val;
}

void Renderer::SkipFrame(PPU::FrameWrites frame_writes) {
    LOG(Debug, "Skipping Frame");
    for (auto write : frame_writes.lcd_writes) RecieveLCDWrite(write);
    for (auto write : frame_writes.oam_dmas) RecieveOAMWrite(write);
    for (auto write : frame_writes.vram_writes) RecieveVRAMWrite(write);
}

void Renderer::RenderFrame(PPU::FrameWrites frame_writes) {
    auto start = std::chrono::high_resolution_clock::now();
    frame_writes.Close();
    auto lcd_it = frame_writes.lcd_writes.begin();
    auto oam_it = frame_writes.oam_dmas.begin();
    auto vram_it = frame_writes.vram_writes.begin();
    auto& frame = presenter.GetCurrentFrame();
    for (unsigned scanline{0}; scanline < PPU::HEIGHT; ++scanline) {
        while (lcd_it->scanline < scanline) RecieveLCDWrite(*lcd_it++);
        while (oam_it->scanline <= scanline) RecieveOAMWrite(*oam_it++);
        while (vram_it->scanline <= scanline) RecieveVRAMWrite(*vram_it++);
        auto buffer = frame.Scanline(scanline);
        RenderBGScanline(scanline, buffer);
        if (lcd.control & 0x02) {
            alignas(__m256i) const std::array<std::array<u32, 4>, 2> palette_arr{GetPalette(lcd.obp0),
                                                                           GetPalette(lcd.obp1)};
            const auto palette = _mm256_load_si256(reinterpret_cast<const __m256i*>(palette_arr.data()));
            const auto zero = _mm256_setzero_si256();
            const auto four = _mm256_set1_epi32(4);
            const auto reverse = _mm256_set_epi32(0, 1, 2, 3, 4, 5, 6, 7);
            for (const auto& object : oam) {
                // TODO: support 16 line sprites
                signed tile_y = scanline - object.ypos + 16;
                if (tile_y < 0 || tile_y > 7) continue;
                if (object.flags & 0x80) LOG(Trace, "Unimplemented sprite priority");
                if (object.flags & 0x40) tile_y = 7 - tile_y;
                auto tile_row = DecodeTile(GetSpriteTile(object.tile)[tile_y]);
                if (object.flags & 0x20) tile_row = _mm256_permutevar8x32_epi32(tile_row, reverse);
                auto mask = _mm256_cmpgt_epi32(tile_row, zero);
                if (object.flags & 0x10) tile_row = _mm256_add_epi32(tile_row, four);
                auto row = _mm256_permutevar8x32_epi32(palette, tile_row);
                _mm256_maskstore_epi32(reinterpret_cast<int*>(buffer.data() + object.xpos - 8),
                                        mask, row);
            }
        }
    }
    presenter.PushFrame();
    // update junk during vblank
    while (lcd_it != frame_writes.lcd_writes.end() - 1) RecieveLCDWrite(*lcd_it++);
    while (oam_it != frame_writes.oam_dmas.end() - 1) RecieveOAMWrite(*oam_it++);
    while (vram_it != frame_writes.vram_writes.end() - 1) RecieveVRAMWrite(*vram_it++);
    speed = std::chrono::high_resolution_clock::now() - start;
}

void Renderer::RenderBGScanline(unsigned scanline, std::span<u32, PPU::WIDTH>& buffer) {
    alignas(__m128i) auto palette_arr = GetPalette(lcd.bgp);
    auto background_palette =
        _mm256_set_m128i(_mm_undefined_si128(),
                         _mm_load_si128(reinterpret_cast<const __m128i*>(palette_arr.data())));
    // TODO: handle scx
    unsigned y = lcd.scy + scanline;
    unsigned tile_x = lcd.scx / 8;
    unsigned x_shift = lcd.scx % 8;
    for (unsigned tile_num{0}; tile_num < (PPU::WIDTH / 8 + 1); ++tile_num) {
        unsigned idx = GetBGTileIndex(tile_x + tile_num, y / 8);
        auto tile = GetBGTile(idx);
        RenderBGSpriteRow(std::span<u32, 8>{buffer.data() + tile_num * 8 - x_shift, 8}, tile[y % 8],
                          background_palette);
    }
}

__m256i Renderer::DecodeTile(u16 tile_row) {
    auto shift = _mm256_set_epi32(0, 1, 2, 3, 4, 5, 6, 7);
    auto low = _mm256_set1_epi32(tile_row >> 8);
    low = _mm256_srlv_epi32(low, shift);
    low = _mm256_and_si256(low, _mm256_set1_epi32(0b01));
    auto high = _mm256_set1_epi32(tile_row << 1);
    high = _mm256_srlv_epi32(high, shift);
    high = _mm256_and_si256(high, _mm256_set1_epi32(0b10));
    return _mm256_or_si256(low, high);
}

void Renderer::RenderBGSpriteRow(std::span<u32, 8> pixels, u16 tile_row, __m256i palette) {
    auto combined = DecodeTile(tile_row);
    auto row = _mm256_permutevar8x32_epi32(palette, combined);
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(pixels.data()), row);
}

void Renderer::Run() {
    while (!stop) {
        usize frame_count = frame_write_queue.read_available();
        if (!frame_count) {
            std::this_thread::yield();
            continue;
        }
        // skip to most recent frame
        while (frame_count-- > 1)
            frame_write_queue.consume_one(
                [this](PPU::FrameWrites frame_writes) { SkipFrame(std::move(frame_writes)); });
        frame_write_queue.consume_one(
            [this](PPU::FrameWrites frame_writes) { RenderFrame(std::move(frame_writes)); });
    }
}

Renderer::Renderer(std::unique_ptr<VulkanFrontend> frontend)
    : presenter{std::move(frontend)}, render_thread{[this] { Run(); }} {}

Renderer::~Renderer() {
    stop = true;
    while (!render_thread.joinable())
        ;
    render_thread.join();
}

Renderer::Presenter::Presenter(std::unique_ptr<VulkanFrontend> _frontend)
    : frontend{std::move(_frontend)} {
    physical_device = frontend->GetVulkanInstance().enumeratePhysicalDevices()[0];
    {
        auto physical_properties = physical_device.getProperties();
        LOG(Info, "Vulkan {}.{}", VK_VERSION_MAJOR(physical_properties.apiVersion),
            VK_VERSION_MINOR(physical_properties.apiVersion));
        LOG(Info, "Device: {}", physical_properties.deviceName);
    }
    // Create device and get queue
    {
        queue_family_index = Vulkan::GetQueueFamilyIndex(
            physical_device, vk::QueueFlagBits::eTransfer | vk::QueueFlagBits::eGraphics);
        vk::DeviceQueueCreateInfo queue_create_info;
        queue_create_info.queueCount = 1;
        queue_create_info.queueFamilyIndex = queue_family_index;
        static constexpr float priority = 1.0;
        queue_create_info.pQueuePriorities = &priority;
        vk::DeviceCreateInfo device_info;
        device_info.setQueueCreateInfoCount(1);
        device_info.setPQueueCreateInfos(&queue_create_info);

        constexpr std::array device_extensions{
            VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        };
        device_info.enabledExtensionCount = device_extensions.size();
        device_info.ppEnabledExtensionNames = device_extensions.data();

        device = physical_device.createDeviceUnique(device_info);
        queue = device->getQueue(queue_family_index, 0);
    }
    if (!physical_device.getSurfaceSupportKHR(queue_family_index, frontend->GetDisplaySurface()))
        LOG(Critical, "Window surface is not supported on this queue family");
    {
        vk::SwapchainCreateInfoKHR swapchain_create_info;
        swapchain_create_info.surface = frontend->GetDisplaySurface();
        auto surface_capabilities =
            physical_device.getSurfaceCapabilitiesKHR(frontend->GetDisplaySurface());

        // Choose swapchain image count
        swapchain_create_info.minImageCount = 2;
        {
            auto surface_formats =
                physical_device.getSurfaceFormatsKHR(frontend->GetDisplaySurface());

            // TODO: check format
            swapchain_create_info.imageFormat = surface_formats[0].format;
            swapchain_create_info.imageColorSpace = surface_formats[0].colorSpace;
        }
        // Prefer Mailbox mode for lower latency, otherwise default to FIFO
        {
            auto present_modes =
                physical_device.getSurfacePresentModesKHR(frontend->GetDisplaySurface());
            swapchain_create_info.presentMode = vk::PresentModeKHR::eFifo;
        }
        auto [window_width, window_height] = frontend->GetDisplayDimensions();
        swapchain_create_info.imageExtent.width = window_width;
        swapchain_create_info.imageExtent.height = window_height;

        swapchain_create_info.imageArrayLayers = 1;
        swapchain_create_info.imageUsage =
            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eColorAttachment;
        swapchain_create_info.imageSharingMode = vk::SharingMode::eExclusive;
        swapchain_create_info.preTransform = surface_capabilities.currentTransform;
        swapchain_create_info.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
        swapchain_create_info.clipped = VK_TRUE;

        swapchain = device->createSwapchainKHRUnique(swapchain_create_info);
    }
    swapchain_images = device->getSwapchainImagesKHR(*swapchain);
    {
        vk::ImageCreateInfo staging_image_create_info;
        staging_image_create_info.imageType = vk::ImageType::e2D;
        staging_image_create_info.extent = vk::Extent3D{256, 256, 1};
        staging_image_create_info.arrayLayers = 1;
        staging_image_create_info.mipLevels = 1;
        staging_image_create_info.format = vk::Format::eR8G8B8A8Unorm;
        staging_image_create_info.tiling = vk::ImageTiling::eLinear;
        staging_image_create_info.initialLayout = vk::ImageLayout::eUndefined;
        staging_image_create_info.sharingMode = vk::SharingMode::eExclusive;
        staging_image_create_info.usage = vk::ImageUsageFlagBits::eTransferSrc;

        for (auto& image : staging_images)
            image = device->createImageUnique(staging_image_create_info);

        auto staging_memory_requirements = device->getImageMemoryRequirements(*staging_images[0]);
        vk::MemoryAllocateInfo staging_memory_allocation_info;
        staging_memory_allocation_info.allocationSize =
            staging_memory_requirements.size * staging_images.size();
        staging_memory_allocation_info.memoryTypeIndex = Vulkan::GetMemoryType(
            physical_device, staging_memory_requirements.memoryTypeBits,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);

        staging_buffer_mem = device->allocateMemoryUnique(staging_memory_allocation_info);
        for (usize i{0}; i < staging_images.size(); ++i)
            device->bindImageMemory(*staging_images[i], *staging_buffer_mem,
                                    i * staging_memory_requirements.size);
        frames = std::span<Frame, 3>{
            static_cast<Frame*>(device->mapMemory(*staging_buffer_mem, 0,
                                                  staging_memory_allocation_info.allocationSize)),
            3};
    }
    image_available_semaphore = device->createSemaphoreUnique({});
    blit_finished_semaphore = device->createSemaphoreUnique({});
    blit_fence = device->createFenceUnique({});
    {
        vk::CommandPoolCreateInfo cmd_pool_create_info;
        cmd_pool_create_info.queueFamilyIndex = queue_family_index;
        cmd_pool = device->createCommandPoolUnique(cmd_pool_create_info);
    }
    {
        vk::CommandBufferAllocateInfo cmd_buffer_allocate_info;
        cmd_buffer_allocate_info.commandPool = *cmd_pool;
        cmd_buffer_allocate_info.commandBufferCount = 1;
        auto cmd_buffer =
            std::move(device->allocateCommandBuffersUnique(cmd_buffer_allocate_info)[0]);

        vk::ImageMemoryBarrier barrier;

        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.oldLayout = vk::ImageLayout::eUndefined;
        barrier.newLayout = vk::ImageLayout::eGeneral;
        barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.layerCount = 1;

        vk::CommandBufferBeginInfo cmd_buffer_begin;
        cmd_buffer_begin.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
        cmd_buffer->begin(cmd_buffer_begin);
        for (const auto& image : staging_images) {
            barrier.image = *image;
            cmd_buffer->pipelineBarrier(vk::PipelineStageFlagBits::eAllCommands,
                                        vk::PipelineStageFlagBits::eAllCommands, {}, {}, {},
                                        barrier);
        }
        cmd_buffer->end();

        vk::SubmitInfo submit_info;
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &*cmd_buffer;
        queue.submit(submit_info, nullptr);
        queue.waitIdle();
    }
    presentation_thread = std::thread{[this] { Present(); }};
}

Renderer::Presenter::~Presenter() {
    stop = true;
    while (!presentation_thread.joinable())
        ;
    presentation_thread.join();
}

void Renderer::Presenter::Present() {
    while (!stop) {
        auto [window_width, window_height] = frontend->GetDisplayDimensions();
        if (!(window_width | window_height)) {
            // The window is minimized
            std::this_thread::sleep_for(20ms);
            continue;
        }
        presenting_frame = pending_frame.exchange(presenting_frame);
        if (presenting_frame < 0) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        device->resetFences(*blit_fence);
        vk::UniqueCommandBuffer cmd;
        {
            vk::CommandBufferAllocateInfo cmd_alloc;
            cmd_alloc.commandPool = *cmd_pool;
            cmd_alloc.commandBufferCount = 1;
            cmd = std::move(device->allocateCommandBuffersUnique(cmd_alloc)[0]);
        }
        auto [acquire_image_result, image_index] = device->acquireNextImageKHR(
            *swapchain, std::numeric_limits<std::uint64_t>::max(), *image_available_semaphore, {});
        vk::Image present_image = swapchain_images[image_index];
        {
            std::array<vk::ImageMemoryBarrier, 2> barriers;
            auto& src_barrier = barriers[0];
            auto& dst_barrier = barriers[1];
            src_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            src_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            src_barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
            src_barrier.subresourceRange.levelCount = 1;
            src_barrier.subresourceRange.layerCount = 1;

            dst_barrier = src_barrier;

            dst_barrier.image = present_image;
            dst_barrier.oldLayout = vk::ImageLayout::eUndefined;
            dst_barrier.newLayout = vk::ImageLayout::eTransferDstOptimal;
            dst_barrier.srcAccessMask = {};
            dst_barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;

            src_barrier.image = *staging_images[presenting_frame - 1];
            src_barrier.oldLayout = vk::ImageLayout::eGeneral;
            src_barrier.newLayout = vk::ImageLayout::eTransferSrcOptimal;
            src_barrier.srcAccessMask = {};
            src_barrier.dstAccessMask = vk::AccessFlagBits::eTransferRead;

            vk::ImageBlit blit;
            blit.srcSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
            blit.srcSubresource.layerCount = 1;
            blit.dstSubresource = blit.srcSubresource;
            blit.srcOffsets[0] = vk::Offset3D{0, 1, 0};
            blit.srcOffsets[1] = vk::Offset3D{PPU::WIDTH, PPU::HEIGHT + 1, 1};
            blit.dstOffsets[0] = vk::Offset3D{0, 0, 0};
            blit.dstOffsets[1] =
                vk::Offset3D{static_cast<s32>(window_width), static_cast<s32>(window_height), 1};

            cmd->begin(vk::CommandBufferBeginInfo{vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
            cmd->pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                                 vk::PipelineStageFlagBits::eTransfer, {}, {}, {}, barriers);

            cmd->blitImage(*staging_images[presenting_frame - 1],
                           vk::ImageLayout::eTransferSrcOptimal, present_image,
                           vk::ImageLayout::eTransferDstOptimal, blit, vk::Filter::eNearest);

            std::swap(src_barrier.oldLayout, src_barrier.newLayout);
            std::swap(src_barrier.srcAccessMask, src_barrier.dstAccessMask);
            dst_barrier.oldLayout = dst_barrier.newLayout;
            dst_barrier.newLayout = vk::ImageLayout::ePresentSrcKHR;
            std::swap(dst_barrier.srcAccessMask, dst_barrier.dstAccessMask);
            cmd->pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                 vk::PipelineStageFlagBits::eBottomOfPipe, {}, {}, {}, barriers);
            cmd->end();
        }
        {
            vk::SubmitInfo submit_info;
            submit_info.commandBufferCount = 1;
            submit_info.pCommandBuffers = &*cmd;
            submit_info.waitSemaphoreCount = 1;
            submit_info.pWaitSemaphores = &*image_available_semaphore;
            vk::PipelineStageFlags stage{vk::PipelineStageFlagBits::eTransfer};
            submit_info.pWaitDstStageMask = &stage;
            submit_info.signalSemaphoreCount = 1;
            submit_info.pSignalSemaphores = &*blit_finished_semaphore;
            queue.submit(submit_info, *blit_fence);
        }
        {
            vk::PresentInfoKHR present_info;
            present_info.waitSemaphoreCount = 1;
            present_info.pWaitSemaphores = &*blit_finished_semaphore;
            present_info.swapchainCount = 1;
            present_info.pSwapchains = &*swapchain;
            present_info.pImageIndices = &image_index;
            queue.presentKHR(present_info);
        }
        device->waitForFences(*blit_fence, true, std::numeric_limits<std::uint64_t>::max());

        presenting_frame = -(presenting_frame);
    }
}

} // namespace CGB::Core