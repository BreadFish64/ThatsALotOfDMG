find_package(fmt REQUIRED)
find_package(Boost REQUIRED)

add_library(Common STATIC
	"logger.hpp"
    "logger.cpp"
    "types.hpp"
    "virtual_memory.hpp"
 )

CGB_ADD_DEFINITIONS(Common)

target_link_libraries(Common PUBLIC fmt::fmt Boost::boost)

if(MSVC)
target_sources(Common PRIVATE
    "windows/virtual_memory_windows.hpp"
    "windows/virtual_memory_windows.cpp"
    "windows/win_error.hpp"
    "windows/win_error.cpp"
)

# https://docs.microsoft.com/en-us/answers/questions/129563/how-to-use-virtualalloc2-memory-api.html
# resolve linker errors for VirtualAlloc2
target_link_libraries(Common PRIVATE "mincore.lib")
target_link_libraries(Common PRIVATE "ntdll.lib")
endif()