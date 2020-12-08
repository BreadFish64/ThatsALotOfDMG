#pragma once

#include <memory>

#include "core/ppu/vulkan/vk_frontend.hpp"

struct SDL_Window;

namespace CGB::Frontend::SDL2 {

class SDL2_VK_Frontend final : public CGB::Core::VulkanFrontend {
public:
    SDL2_VK_Frontend();
    virtual ~SDL2_VK_Frontend() override;

    virtual vk::Instance GetVulkanInstance() const override;
    virtual vk::SurfaceKHR GetDisplaySurface() const override;
    virtual vk::Extent2D GetDisplayDimensions() const override;

    SDL_Window* GetWindow() { return window.get(); }

private:

    std::unique_ptr<SDL_Window, void (*)(SDL_Window*)> window{nullptr, nullptr};

    vk::UniqueInstance vk_instance;
    vk::UniqueSurfaceKHR vk_window_surface;
    vk::UniqueDebugUtilsMessengerEXT messenger;
};

} // namespace CGB::Frontend::SDL2
