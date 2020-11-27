#pragma once

#include <memory>

#include "common/types.hpp"
#include "common/virtual_memory.hpp"
#include "cpu/cpu.hpp"

namespace CGB {

class Cartridge;
class SpecializedCartridge;
class BaseCPU;
class PPU;

struct MemoryTag {
    bool read : 1;
    bool write : 1;
    u8 handler : 6;
};
static_assert(sizeof(MemoryTag) == 1);

class Bus {
public:
    using ReadHandler = u8 (*)(Bus&, GADDR, u64);
    using WriteHandler = void (*)(Bus&, GADDR, u8, u64);
    static constexpr usize ADDRESS_SPACE = 0x10000;
    static constexpr usize PAGE_SIZE = 0x1000;

private:
    struct TAG_TYPES {
        static constexpr usize NOP = 0x0000, PASS_THROUGH = 0x1000, IO = 0x2000, SIZE = 0x3000;
    };

    static u8 ReadNop(Bus& bus, GADDR addr, u64 timestamp);
    static void WriteNop(Bus& bus, GADDR addr, u8 val, u64 timestamp);

    std::array<ReadHandler, 1 << 6> read_handlers{ReadNop};
    std::array<WriteHandler, 1 << 6> write_handlers{WriteNop};
    usize memory_handler_count{1};

    Common::VirtualMemory::ReservedSpace address_space;

    std::unique_ptr<SpecializedCartridge> cartridge;
    CPU::BaseCPU cpu;
    std::unique_ptr<PPU> ppu;

    Common::VirtualMemory::MemoryBacking wram_backing;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> fixed_wram;
    std::array<Common::VirtualMemory::ReservedMappedSection, 2> switchable_wram;

    Common::VirtualMemory::MemoryBacking tag_backing;
    std::array<Common::VirtualMemory::ReservedMappedSection, 4> wram_tags;


public:
    explicit Bus(std::unique_ptr<Cartridge> cartridge, CPU::BaseCPU cpu,
                 std::unique_ptr<PPU> ppu);
    ~Bus();
    auto Memory() { return address_space.Span().first<ADDRESS_SPACE>(); }
    auto Tags() { return address_space.Span<MemoryTag>().last<ADDRESS_SPACE>(); }

    Common::VirtualMemory::ReservedSpace& GetAddressSpace() { return address_space; };

    Cartridge& GetCartridge();
    const Cartridge& GetCartridge() const;

    PPU& GetPPU() { return *ppu; }
    CPU::BaseCPU& GetCPU() { return cpu; }

    Common::VirtualMemory::ReservedMappedSection MapNopTag(u8 page);
    Common::VirtualMemory::ReservedMappedSection MapPassthroughTag(u8 page);

    void SwitchWRAMBank(u8 index);

    u8 Read(GADDR addr, u64 timestamp) {
        auto tag = Tags()[addr];
        if (tag.read) [[unlikely]] return read_handlers[tag.handler](*this, addr, timestamp);
        return Memory()[addr];
    }
    void Write(GADDR addr, u8 val, u64 timestamp) {
        auto tag = Tags()[addr];
        if (tag.write) [[unlikely]] return write_handlers[tag.handler](*this, addr, val, timestamp);
        Memory()[addr] = val;
    }

    MemoryTag RegisterMemoryTag(ReadHandler read_handler, WriteHandler write_handler);
};

} // namespace CGB