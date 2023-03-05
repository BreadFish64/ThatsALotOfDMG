#include <Windows.h>

#include "common/logger.hpp"
#include "virtual_memory_windows.hpp"
#include "win_error.hpp"

#pragma warning(disable : 28160)

namespace CGB::Common::Windows::VirtualMemory {

constexpr DWORD AccessMask(PROTECTION protection) {
    switch (protection) {
    case PROTECTION::READ: return GENERIC_READ;
    case PROTECTION::WRITE: return GENERIC_WRITE;
    case PROTECTION::READ_WRITE: return GENERIC_WRITE | GENERIC_READ;
    default: return {};
    }
}

constexpr DWORD ShareMode(PROTECTION protection) {
    switch (protection) {
    case PROTECTION::READ: return FILE_SHARE_READ;
    case PROTECTION::WRITE: return FILE_SHARE_WRITE;
    case PROTECTION::READ_WRITE: return FILE_SHARE_READ | FILE_SHARE_WRITE;
    default: return {};
    }
}

constexpr ULONG PageProtection(PROTECTION protection) {
    switch (protection) {
    case PROTECTION::READ: return PAGE_READONLY;
    case PROTECTION::WRITE: return PAGE_READWRITE;
    case PROTECTION::READ_WRITE: return PAGE_READWRITE;
    default: return PAGE_NOACCESS;
    }
}

constexpr ULONG FileMap(PROTECTION protection) {
    switch (protection) {
    case PROTECTION::READ: return FILE_MAP_READ;
    case PROTECTION::WRITE: return FILE_MAP_WRITE;
    case PROTECTION::READ_WRITE: return FILE_MAP_WRITE | FILE_MAP_READ;
    default: return {};
    }
}

void HandleCloser::operator()(HANDLE handle) const { ::CloseHandle(handle); }

MappedFile::MappedFile() : file_handle{INVALID_HANDLE_VALUE} {};

MappedFile::MappedFile(const std::filesystem::path& path, PROTECTION access, PROTECTION share) {
    file_handle.reset(CreateFileW(path.c_str(), AccessMask(access), ShareMode(share), nullptr,
                                  OPEN_EXISTING, FILE_FLAG_RANDOM_ACCESS, nullptr));
    if (file_handle.get() == INVALID_HANDLE_VALUE) {
        LogError();
        return;
    }
    LARGE_INTEGER win_size{};
    if (!GetFileSizeEx(file_handle.get(), &win_size)) {
        LogError();
        return;
    }
    size = static_cast<usize>(win_size.QuadPart);
}

MappedFile::~MappedFile() {}

void WinVirtualFree::operator()(void* ptr) const { ::VirtualFree(ptr, 0, MEM_RELEASE); }

ReservedSpace::ReservedSpace(usize size, void* base) : size{size} {
    ptr.reset(::VirtualAlloc2(::GetCurrentProcess(), base, size,
                              MEM_RESERVE | MEM_RESERVE_PLACEHOLDER, PAGE_NOACCESS, nullptr, 0));
    if (!ptr) LogError();
}

ReservedSpace::~ReservedSpace() {}

void ReservedSpace::Split(usize offset, usize split_size) {
    if (!::VirtualFree(static_cast<u8*>(ptr.get()) + offset, split_size,
                       MEM_RELEASE | MEM_PRESERVE_PLACEHOLDER)) {
        LOG(Warning, "Address space slice failed, the region may already be the right size");
    }
}

MemoryBacking::MemoryBacking(usize size) {
    backing_handle.reset(CreateFileMapping2(
        INVALID_HANDLE_VALUE, nullptr, FileMap(PROTECTION::READ_WRITE),
        PageProtection(PROTECTION::READ_WRITE), SEC_COMMIT, size, nullptr, nullptr, 0));
    if (!backing_handle) LogError();
}

MemoryBacking::MemoryBacking(const MappedFile& file, PROTECTION access, PROTECTION page_protection,
                             usize size) {
    backing_handle.reset(CreateFileMapping2(file.WinHandle(), nullptr, FileMap(access),
                                            PageProtection(page_protection), SEC_COMMIT, size,
                                            nullptr, nullptr, 0));
    if (!backing_handle) LogError();
}

MemoryBacking::~MemoryBacking() {}

extern "C" __declspec(dllimport) NTSTATUS WINAPI RtlGetLastNtStatus();

MappedSection MemoryBacking::Map(usize offset, usize size, PROTECTION page_protection, void* base) {
    void* ptr = MapViewOfFile2(backing_handle.get(), GetCurrentProcess(), offset, base, size, {},
                               PageProtection(page_protection));
    if (!ptr) {
        LOG(Error, "{:#010X}", static_cast<u32>(RtlGetLastNtStatus()));
        LogError();
        return {ptr, 0};
    };
    if (!size) {
        MEMORY_BASIC_INFORMATION info;
        VirtualQuery(ptr, &info, sizeof(info));
        size = info.RegionSize;
    }
    return {ptr, size};
};

ReservedMappedSection MemoryBacking::Map(usize offset, usize size, PROTECTION page_protection,
                                         ReservedSpace& space, usize space_offset) {
    void* ptr = MapViewOfFile3(
        backing_handle.get(), GetCurrentProcess(), space.Span().data() + space_offset, offset, size,
        MEM_REPLACE_PLACEHOLDER, PageProtection(page_protection), nullptr, 0);
    if (!ptr) {
        LOG(Error, "{:#010X}", static_cast<u32>(RtlGetLastNtStatus()));
        LogError();
        return {ptr, 0};
    };
    if (!size) {
        MEMORY_BASIC_INFORMATION info;
        VirtualQuery(ptr, &info, sizeof(info));
        size = info.RegionSize;
    }
    return {ptr, size};
}

void Unmap::operator()(void* ptr) const { ::UnmapViewOfFile(ptr); }

void PreservePlaceholder::operator()(void* ptr) const {
    ::UnmapViewOfFile2(GetCurrentProcess(), ptr, MEM_PRESERVE_PLACEHOLDER);
}

} // namespace CGB::Common::Windows::VirtualMemory