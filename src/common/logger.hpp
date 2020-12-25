#pragma once

#include <array>
#include <cassert>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <string_view>
#include <thread>

#include <boost/lockfree/queue.hpp>

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/ostream.h>

#include "types.hpp"

namespace CGB::Logger {

enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical,
};

using DeferredLog = std::function<std::pair<LogLevel, std::string>()>;
using LogQueue = boost::lockfree::queue<DeferredLog*>;

class Logger {
    std::atomic<bool> run_logger;
    std::atomic<bool> logger_closed;
    std::thread logging_thread;
    std::ofstream log_file;

public:
    LogQueue log_queue{0};

    Logger();
    ~Logger();
    void ConsumeLog(DeferredLog* log);
    void ConsumeLogs();
};

inline Logger logger;

template <typename... Args, usize... idx>
std::string FormatLog(std::tuple<Args...> args, std::index_sequence<idx...>) {
    return fmt::format(std::get<idx>(args)...);
}

template <typename... Args>
void Log(LogLevel level, Args... args) {
    auto log = new DeferredLog{[level, args = std::make_tuple(args...)] {
        return std::make_pair(level,
                              FormatLog(std::move(args), std::index_sequence_for<Args...>{}));
    }};
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
    if constexpr (/*IS_DEBUG ||*/ ::CGB::Logger::LogLevel::level != ::CGB::Logger::LogLevel::Trace)    \
        ::CGB::Logger::Log(::CGB::Logger::LogLevel::level, FMT_STRING("{}:{} {}: " message "\n"), NORM_PATH,   \
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