#include "common/logger.hpp"

namespace CGB::Logger {

static constexpr std::array LEVEL_STYLES{
    fmt::fg(fmt::color::gray),   fmt::fg(fmt::color::dark_cyan), fmt::fg(fmt::color::white),
    fmt::fg(fmt::color::yellow), fmt::fg(fmt::color::red),       fmt::fg(fmt::color::magenta),
};
static constexpr std::array LEVEL_NAMES{
    "Trace"sv, "Debug"sv, "Info"sv, "Warning"sv, "Error"sv, "Critical"sv,
};

void Logger::ConsumeLog(DeferredLog* log) {
    LogLevel level = LogLevel::Critical;
    fmt::memory_buffer buffer;
    (*log)(level, buffer);
    std::string_view message{buffer.data(), buffer.size()};
    if (level > LogLevel::Trace) fmt::print(LEVEL_STYLES[static_cast<usize>(level)], "{}", message);
    fmt::print(log_file, "{:8}:\t{}", LEVEL_NAMES[static_cast<usize>(level)], message);
    delete log;
}

void Logger::ConsumeLogs() {
    while (run_logger) {
        log_queue.consume_all([this](DeferredLog* log) { ConsumeLog(log); });
        if (log_queue.empty()) std::this_thread::sleep_for(100us);
    }
    logger_closed = true;
}

Logger::Logger() {
    fopen_s(&log_file, "log.txt", "w");

    if constexpr (!IS_DEBUG) {
        run_logger = true;
        logger_closed = false;
        logging_thread = std::thread{[this] { ConsumeLogs(); }};
    }
}

Logger::~Logger() {
    if constexpr (!IS_DEBUG) {
        run_logger = false;
        while (!logging_thread.joinable())
            ;
        logging_thread.join();
    }
    std::fclose(log_file);
}

} // namespace CGB::Logger