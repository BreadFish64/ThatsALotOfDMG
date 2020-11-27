#pragma once

#include <filesystem>

#include "common/types.hpp"

namespace CGB::Common::Windows::VirtualMemory {

enum class PROTECTION {
    NONE = 0,
    READ = 1 << 0,
    WRITE = 1 << 1,
    READ_WRITE = READ | WRITE,
};

struct HandleCloser {
    void operator()(HANDLE handle) const;
};
using WinHandle = std::unique_ptr<std::remove_pointer_t<HANDLE>, HandleCloser>;


class MappedFile {
    WinHandle file_handle{};

public:
    MappedFile();
    MappedFile(MappedFile&&) = default;
    explicit MappedFile(const std::filesystem::path& path, PROTECTION access, PROTECTION share);
    ~MappedFile();
    MappedFile& operator=(MappedFile&&) = default;

    HANDLE WinHandle() const { return file_handle.get(); }
};

struct WinVirtualFree {
    void operator()(void* ptr) const;
};

class ReservedSpace {
    std::unique_ptr<void, WinVirtualFree> ptr{};
    usize size{};

public:
    ReservedSpace() = default;
    ReservedSpace(ReservedSpace&& other) = default;
    explicit ReservedSpace(usize size, void* base = nullptr);
    ~ReservedSpace();
    ReservedSpace& operator=(ReservedSpace&&) = default;

    void Split(usize offset, usize size);

    template <typename T = u8>
    std::span<T> Span() const {
        return {static_cast<T*>(ptr.get()), size / sizeof(T)};
    }

    template <typename T = u8>
    operator std::span<T>() const {
        return Span<T>();
    }
};

struct Unmap {
    void operator()(void* ptr) const;
};
struct PreservePlaceholder {
    void operator()(void* ptr) const;
};

template <typename Unmapper>
class GenericMappedSection {
    std::unique_ptr<void, Unmapper> ptr{};
    usize size{};

public:
    GenericMappedSection() = default;
    GenericMappedSection(GenericMappedSection&&) = default;
    GenericMappedSection(void* ptr, usize size) : ptr{ptr}, size{size} {};
    GenericMappedSection& operator=(GenericMappedSection&&) = default;

    template <typename T = u8>
    std::span<T> Span() const {
        return {static_cast<T*>(ptr.get()), size / sizeof(T)};
    }

    template <typename T = u8>
    operator std::span<T>() const {
        return Span<T>();
    }
};
using MappedSection = GenericMappedSection<Unmap>;
using ReservedMappedSection = GenericMappedSection<PreservePlaceholder>;

class MemoryBacking {
    WinHandle backing_handle{};

public:
    MemoryBacking() = default;
    MemoryBacking(MemoryBacking&&) = default;
    explicit MemoryBacking(usize size);
    explicit MemoryBacking(const MappedFile& file, PROTECTION access, PROTECTION page_protection,
                           usize size = 0);
    MemoryBacking& operator=(MemoryBacking&&) = default;

    [[nodiscard]] MappedSection Map(usize offset, usize size, PROTECTION page_protection,
                                    void* base = nullptr);
    [[nodiscard]] ReservedMappedSection Map(usize offset, usize size, PROTECTION page_protection,
                                            ReservedSpace& space, usize space_offset = 0);
    ~MemoryBacking();

    HANDLE WinHandle() const { return backing_handle.get(); }
};

} // namespace CGB::Common::Windows::VirtualMemory