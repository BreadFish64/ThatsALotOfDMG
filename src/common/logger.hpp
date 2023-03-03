#pragma once

#include <array>
#include <deque>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <string_view>
#include <thread>

#include <boost/lockfree/queue.hpp>

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/core.h>

#include "types.hpp"

namespace CGB::Logger {

constexpr bool ENABLE_TRACE_LOGS = false;

enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical,
};

using DeferredLog = std::function<void(LogLevel&, fmt::memory_buffer&)>;
using LogQueue = boost::lockfree::queue<DeferredLog*>;

class Logger {
    std::atomic<bool> run_logger;
    std::atomic<bool> logger_closed;
    std::thread logging_thread;
    std::FILE* log_file;

public:
    LogQueue log_queue{0};

    Logger();
    ~Logger();
    void ConsumeLog(DeferredLog* log);
    void ConsumeLogs();
};

inline Logger logger;

template <usize... idx>
auto FormatLog(fmt::string_view format, const auto& arg_tuple, std::index_sequence<idx...>) {
    return fmt::format(fmt::runtime(format), std::get<idx>(arg_tuple)...);
}

template <typename... Args>
void Log(LogLevel level, const auto& format, Args&&... args) {
    auto log = new DeferredLog{[level = level, format = fmt::string_view(format),
                                fmt_args = fmt::make_format_args(args...)](
                                   LogLevel& out_log_level, fmt::memory_buffer& buffer) {
        out_log_level = level;
        fmt::detail::vformat_to(buffer, format, fmt_args, {});
    }};
    // auto log = new DeferredLog{std::async(
    //     std::launch::deferred,
    //     [](LogLevel level, const std::string& message) { return std::make_pair(level, message);
    //     }, level, fmt::format(fmt::runtime(format), args...))};
    if constexpr (IS_DEBUG) {
        logger.ConsumeLog(log);
    } else {
        logger.log_queue.push(log);
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
    if constexpr (::CGB::Logger::ENABLE_TRACE_LOGS ||                                              \
                  ::CGB::Logger::LogLevel::level != ::CGB::Logger::LogLevel::Trace) {              \
        ::CGB::Logger::Log(::CGB::Logger::LogLevel::level, FMT_STRING("{}:{} {}: " message "\n"),  \
                           NORM_PATH, __LINE__, LOG_FUNC, __VA_ARGS__);                            \
    }

#if IS_DEBUG

#define UNREACHABLE() std::abort()

#else

#if defined(_MSC_VER)
#define UNREACHABLE() __assume(false)
#else
#if defined(__has_builtin) && __has_builtin(__builtin_unreachable)
#define UNREACHABLE() __builtin_unreachable()
#else
[[noreturn]] inline void UNREACHABLE() {
    while (true)
        ;
}
#endif
#endif
#endif