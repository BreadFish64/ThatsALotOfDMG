#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

namespace CGB {

using namespace std::literals;

using HANDLE = void*;

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using s8 = std::int8_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;

using f32 = float;
using f64 = double;

using usize = std::size_t;

constexpr usize operator""_sz(unsigned long long x) { return static_cast<usize>(x); }

// represents a guest address
using GADDR = u16;

// Cast enum class to underlying type
template <typename E>
auto underlying_cast(E e) {
    return static_cast<std::underlying_type_t<E>>(e);
} 

} // namespace CGB