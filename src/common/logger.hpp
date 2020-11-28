#pragma once

#include <array>
#include <cassert>
#include <functional>
#include <string_view>

#include <boost/lockfree/spsc_queue.hpp>

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

using DeferredLog = std::function<std::pair<LogLevel, std::string>()>;
using LogQueue = boost::lockfree::spsc_queue<DeferredLog, boost::lockfree::capacity<256>>;

inline LogQueue log_queue{};

void InitLogger();

template <typename... Args, usize... idx>
std::string FormatLog(std::tuple<Args...> args, std::index_sequence<idx...>) {
    return fmt::format(std::get<idx>(args)...);
}

void ConsumeLog(DeferredLog log);

template <typename... Args>
void Log(LogLevel level, Args... args) {
    auto log = DeferredLog{[level, args = std::make_tuple(args...)] {
        return std::make_pair(level,
                              FormatLog(std::move(args), std::index_sequence_for<Args...>{}));
    }};
    if constexpr (IS_DEBUG) {
        ConsumeLog(std::move(log));
    } else {
        log_queue.push(std::move(log));
    }
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