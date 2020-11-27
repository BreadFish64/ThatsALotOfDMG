#pragma once

#ifdef _MSC_VER

#include "windows/virtual_memory_windows.hpp"

namespace CGB::Common::VirtualMemory {
using namespace Windows::VirtualMemory;
}

#else
#error "Virtual Memory is only implemented for Windows"
#endif
