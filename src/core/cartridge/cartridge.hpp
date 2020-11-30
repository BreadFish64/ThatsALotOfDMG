#pragma once

#include <filesystem>

#include "common/virtual_memory.hpp"
namespace CGB::Core {

static constexpr u16 GenLicenseeCode(u16 code) {
    return (((code & 0xF0) << 4) | (code & 0x0F)) + ('0' << 8 | '0');
}

class CartridgeHeader {
protected:
    Common::VirtualMemory::MappedFile rom_file_handle;
    Common::VirtualMemory::MemoryBacking rom_backing;

    Common::VirtualMemory::MappedSection rom_data;

    std::filesystem::path rom_file_path;

public:
    CartridgeHeader(CartridgeHeader&&) = default;
    explicit CartridgeHeader(std::filesystem::path path);
    ~CartridgeHeader();
    CartridgeHeader& operator=(CartridgeHeader&&) = default;

    enum class CGB_FLAG : u8 {
        BACKWARD_COMPATIBLE = 0x80,
        CGB_ONLY = 0xC0,
    };

    enum class NEW_LICENSEE_CODE : u16 {
        NONE = GenLicenseeCode(0x00),
        NINTENDO_R_AND_D_1 = GenLicenseeCode(0x01),
        CAPCOM = GenLicenseeCode(0x08),
        ELECTRONIC_ARTS = GenLicenseeCode(0x13),
        HUDSON_SOFT = GenLicenseeCode(0x18),
        B_AI = GenLicenseeCode(0x19),
        KSS = GenLicenseeCode(0x20),
        POW = GenLicenseeCode(0x22),
        PCM_COMPLETE = GenLicenseeCode(0x24),
        SAN_X = GenLicenseeCode(0x25),
        KEMCO_JAPAN = GenLicenseeCode(0x28),
        SETA = GenLicenseeCode(0x29),
        VIACOM = GenLicenseeCode(0x30),
        NINTENDO = GenLicenseeCode(0x31),
        BANDAI = GenLicenseeCode(0x32),
        OCEAN_ACCLAIM = GenLicenseeCode(0x33),
        KONAMI = GenLicenseeCode(0x34),
        HECTOR = GenLicenseeCode(0x35),
        TAITO = GenLicenseeCode(0x37),
        HUDSON = GenLicenseeCode(0x38),
        BANPRESTO = GenLicenseeCode(0x39),
        UBI_SOFT = GenLicenseeCode(0x41),
        ATLUS = GenLicenseeCode(0x42),
        MALIBU = GenLicenseeCode(0x44),
        ANGEL = GenLicenseeCode(0x46),
        BULLET_PROOF = GenLicenseeCode(0x47),
        IREM = GenLicenseeCode(0x49),
        ABSOLUTE = GenLicenseeCode(0x50),
        ACCLAIM = GenLicenseeCode(0x51),
        ACTIVISION = GenLicenseeCode(0x52),
        AMERICAN_SAMMY = GenLicenseeCode(0x53),
        KONAMI_2 = GenLicenseeCode(0x54),
        HI_TECH_ENTERTAINMENT = GenLicenseeCode(0x55),
        LJN = GenLicenseeCode(0x56),
        MATCHBOX = GenLicenseeCode(0x57),
        MATTEL = GenLicenseeCode(0x58),
        MILTON_BRADLEY = GenLicenseeCode(0x59),
        TITUS = GenLicenseeCode(0x60),
        VIRGIN = GenLicenseeCode(0x61),
        LUCASARTS = GenLicenseeCode(0x64),
        OCEAN = GenLicenseeCode(0x67),
        ELECTRONIC_ARTS_2 = GenLicenseeCode(0x69),
        INFOGRAMES = GenLicenseeCode(0x70),
        INTERPLAY = GenLicenseeCode(0x71),
        BRODERBUND = GenLicenseeCode(0x72),
        SCULPTURED = GenLicenseeCode(0x73),
        SCI = GenLicenseeCode(0x75),
        THQ = GenLicenseeCode(0x78),
        ACCOLADE = GenLicenseeCode(0x79),
        MISOWA = GenLicenseeCode(0x80),
        IOZC = GenLicenseeCode(0x83),
        TOKUMA_SHOTEN_INTERMEDIA = GenLicenseeCode(0x86),
        TSUKUDA_ORIGINAL = GenLicenseeCode(0x87),
        CHUNSOFT = GenLicenseeCode(0x91),
        VIDEO_SYSTEM = GenLicenseeCode(0x92),
        OCEAN_ACCLAIM_2 = GenLicenseeCode(0x93),
        VARIE = GenLicenseeCode(0x95),
        YONEZAWA_SPAL = GenLicenseeCode(0x96),
        KANEKO = GenLicenseeCode(0x97),
        PACK_IN_SOFT = GenLicenseeCode(0x99),
        KONAMI_YU_GI_OH = GenLicenseeCode(0xA4),
    };
    enum class CARTRIDGE_TYPE : u8 {
        ROM_ONLY = 0x00,
        MBC1 = 0x01,
        MBC1_RAM = 0x02,
        MBC1_RAM_BATTERY = 0x03,
        MBC2 = 0x05,
        MBC2_BATTERY = 0x06,
        ROM_RAM = 0x08,
        ROM_RAM_BATTERY = 0x09,
        MMM01 = 0x0B,
        MMM01_RAM = 0x0C,
        MMM01_RAM_BATTERY = 0x0D,
        MBC3_TIMER_BATTERY = 0x0F,
        MBC3_TIMER_BATTERY_RAM = 0x10,
        MBC3 = 0x11,
        MBC3_RAM = 0x12,
        MBC3_RAM_BATTERY = 0x13,
        MBC5 = 0x19,
        MBC5_RAM = 0x1A,
        MBC5_RAM_BATTERY = 0x1B,
        MBC5_RUMBLE = 0x1C,
        MBC5_RUMBLE_RAM = 0x1D,
        MBC5_RUMBLE_RAM_BATTERY = 0x1E,
        MBC6 = 0x20,
        MBC7_SENSOR_RUMBLE_RAM_BATTERY = 0x22,
        POCKET_CAMERA = 0xFC,
        BANDAI_TAMA5 = 0xFD,
        HUC3 = 0xFE,
        HUC1_RAM_BATTERY = 0xFF,
    };
    enum class DESTINATION_CODE : u8 {
        JAPANESE = 0x00,
        NON_JAPANESE = 0x01,
    };

    std::span<const u8, 0x133 - 0x104> NintendoLogo() const {
        return rom_data.Span().subspan<0x104, 0x133 - 0x104>();
    }
    std::string_view Title() const { return rom_data.Span<char>().data() + 0x134; };
    CGB_FLAG CGB_Flag() const { return static_cast<CGB_FLAG>(rom_data.Span()[0x143]); }
    NEW_LICENSEE_CODE NewLicenseeCode() const {
        NEW_LICENSEE_CODE code;
        std::memcpy(&code, rom_data.Span().data() + 0x144, sizeof(NEW_LICENSEE_CODE));
        return code;
    }
    bool SGB_Flag() const { return rom_data.Span()[0x146] == 0x03; }
    CARTRIDGE_TYPE CartridgeType() const { return static_cast<CARTRIDGE_TYPE>(rom_data.Span()[0x147]); }
    u8 RomSizeCode() const { return rom_data.Span()[0x148]; }
    u8 RomBanks() const;
    usize RomSize() const { return 16 * 1024 * RomBanks(); }
    u8 RamSizeCode() const { return rom_data.Span()[0x149]; }
    usize RamSize() const;
    u8 RamBanks() const { return static_cast<u8>((RamSize() + (8 * 1024 - 1)) / (8 * 1024)); }
    DESTINATION_CODE DestinationCode() const {
        return static_cast<DESTINATION_CODE>(rom_data.Span()[0x14A]);
    }
    u8 OldLicenseeCode() const { return rom_data.Span()[0x14B]; }
    u8 MaskROMVersionNumber() const { return rom_data.Span()[0x14C]; }
    u8 HeaderChecksum() const { return rom_data.Span()[0x14D]; }
    u16 GlobalChecksum() const {
        u16 checksum;
        std::memcpy(&checksum, rom_data.Span().data() + 0x14E, sizeof(u16));
        return checksum;
    }

    u8 HashHeader() const;
    u16 HashROM() const;
};

} // namespace CGB
