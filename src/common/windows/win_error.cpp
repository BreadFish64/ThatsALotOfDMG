#include <Windows.h>

#include "common/logger.hpp"
#include "common/types.hpp"

namespace CGB::Common::Windows {

struct LocalFreeDeleter {
    void operator()(HLOCAL handle) { ::LocalFree(handle); }
};

void LogError() {
    DWORD error_id = ::GetLastError();
    if (!error_id) return;
    std::unique_ptr<CHAR, LocalFreeDeleter> message_buffer;
    usize size{0};
    {
        LPSTR message_buffer_raw{};
        size = ::FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                                    FORMAT_MESSAGE_IGNORE_INSERTS,
                                NULL, error_id, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                                reinterpret_cast<LPSTR>(&message_buffer_raw), 0, NULL);
        message_buffer.reset(message_buffer_raw);
    }
    LOG(Error, "{:#010X}: {}", static_cast<u32>(error_id), std::string_view{message_buffer.get(), size});
}

} // namespace CGB::Windows