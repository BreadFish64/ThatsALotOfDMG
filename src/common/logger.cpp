#include <deque>
#include <fstream>
#include <iostream>
#include <thread>

#include "common/logger.hpp"

namespace CGB::Logger {

static constexpr std::array LEVEL_STYLES{
    fmt::fg(fmt::color::gray), fmt::fg(fmt::color::white),   fmt::fg(fmt::color::yellow),
    fmt::fg(fmt::color::red),  fmt::fg(fmt::color::magenta),
};
static constexpr std::array LEVEL_NAMES{
    "Trace"sv, "Info"sv, "Warning"sv, "Error"sv, "Critical"sv,
};

static inline std::thread logging_thread;
static inline std::ofstream log_file;
static inline std::deque<DeferredLog> log_buffer;

void ConsumeLog(DeferredLog log) {
    auto [level, str] = log();
    fmt::print(LEVEL_STYLES[static_cast<usize>(level)], str);
    fmt::print(log_file, "{}:\t{}", LEVEL_NAMES[static_cast<usize>(level)], str);
}

[[maybe_unused]] static void ConsumeLogs() {
    while (true) {
        log_queue.pop_to_output_iterator(std::back_inserter(log_buffer));
        ConsumeLog(std::move(log_buffer.front()));
        log_buffer.pop_front();
        if (log_queue.empty()) { std::this_thread::yield(); }
    }
}

void InitLogger() {
    log_file = std::ofstream{"log.txt"};
    if constexpr (!IS_DEBUG) { logging_thread = std::thread{ConsumeLogs}; }
}

} // namespace CGB::Logger