#pragma once

#include "vk_instance.hpp"

namespace CGB::Core {
class VulkanFrontend {
public:
    virtual ~VulkanFrontend() {}
    virtual vk::Instance GetVulkanInstance() const = 0;
    virtual vk::SurfaceKHR GetDisplaySurface() const = 0;
    virtual vk::Extent2D GetDisplayDimensions() const = 0;

    std::uint64_t frame_count{0};
};
} // namespace CGB::Core