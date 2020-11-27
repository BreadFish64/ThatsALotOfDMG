#pragma once

#include <array>
#include <cassert>
#include <string_view>

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/ostream.h>

#include "types.hpp"

namespace CGB::Logger {

enum class LogLevel {
    Trace,
    Info,
    Warning,
    Error,
    Critical,
};

template <typename... Args>
void Log(LogLevel level, Args&&... args) {
    static constexpr std::array styles{
        fmt::fg(fmt::color::gray), fmt::fg(fmt::color::white),   fmt::fg(fmt::color::yellow),
        fmt::fg(fmt::color::red),  fmt::fg(fmt::color::magenta),
    };
    fmt::print(styles[static_cast<usize>(level)], std::forward<Args>(args)...);
}

constexpr std::string_view NormalizePath(std::string_view str) {
    usize pos = str.find("src");
    return pos == str.npos ? str : str.data() + pos + 4;
}

} // namespace CGB::Logger

#define NORM_PATH                                                                                  \
    [] {                                                                                           \
        static constexpr std::string_view path = ::CGB::Logger::NormalizePath(__FILE__);           \
        return path;                                                                               \
    }()

#define LOG_USE_LONG_FUNCTION_NAMES false

#if LOG_USE_LONG_FUNCTION_NAMES
#if defined(_MSC_VER)
#define LOG_FUNC __FUNCSIG__
#else
#define LOG_FUNC __PRETTY_FUNCTION__
#endif
#else
#define LOG_FUNC __func__
#endif

#define LOG(level, message, ...)                                                                   \
    if constexpr (IS_DEBUG || ::CGB::Logger::LogLevel::level != ::CGB::Logger::LogLevel::Trace)    \
        ::CGB::Logger::Log(::CGB::Logger::LogLevel::level, "{}:{} {}: " message "\n", NORM_PATH,   \
                           __LINE__, LOG_FUNC, __VA_ARGS__);

#if IS_DEBUG
#define UNREACHABLE() std::abort()

#else

#if defined(__has_builtin) && __has_builtin(__builtin_unreachable)
#define UNREACHABLE() __builtin_unreachable()
#elif defined(_MSC_VER)
#define UNREACHABLE() __assume(false)
#else
[[noreturn]] inline void UNREACHABLE() {}
#endif

#endif