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
    auto [level, str] = (*log)();
    delete log;
    /*if (level > LogLevel::Debug)*/ fmt::print(LEVEL_STYLES[static_cast<usize>(level)], str);
    fmt::print(log_file, "{}:\t{}", LEVEL_NAMES[static_cast<usize>(level)], str);
}

void Logger::ConsumeLogs() {
    while (run_logger) {
        log_queue.consume_all([this](DeferredLog* log) { ConsumeLog(log); });
        if (log_queue.empty()) std::this_thread::sleep_for(100us);
    }
    logger_closed = true;
}

Logger::Logger() {
    log_file = std::ofstream{"log.txt"};

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
}

} // namespace CGB::Logger