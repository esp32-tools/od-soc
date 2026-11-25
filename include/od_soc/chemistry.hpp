#pragma once

// Built-in signed reference CellParams presets (52 bytes each in Flash DROM).

#include "od_soc/od_soc.hpp"

namespace od_soc::chemistry {

namespace detail {

constexpr uint16_t crc16_byte(uint16_t crc, uint8_t byte) noexcept {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int b = 0; b < 8; ++b) {
        crc = (crc & 0x8000u) != 0u ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                    : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

constexpr uint16_t crc16_u16(uint16_t crc, uint16_t v) noexcept {
    crc = crc16_byte(crc, static_cast<uint8_t>(v & 0xFFu));
    return crc16_byte(crc, static_cast<uint8_t>((v >> 8) & 0xFFu));
}

constexpr CellParams make_signed(CellParams p) noexcept {
    p.magic[0] = PARAMS_MAGIC_0;
    p.magic[1] = PARAMS_MAGIC_1;
    p.version = PARAMS_VERSION;
    uint16_t crc = 0xFFFFu;
    crc = crc16_byte(crc, p.magic[0]);
    crc = crc16_byte(crc, p.magic[1]);
    crc = crc16_byte(crc, p.version);
    crc = crc16_byte(crc, p.flags);
    for (size_t i = 0; i < OCV_POINTS; ++i) {
        crc = crc16_u16(crc, p.ocv_mv[i]);
    }
    crc = crc16_u16(crc, p.q_max_dmah);
    crc = crc16_u16(crc, p.r0_mohm);
    crc = crc16_u16(crc, p.r1_mohm);
    crc = crc16_u16(crc, p.tau_s);
    crc = crc16_u16(crc, p.v_full_mv);
    crc = crc16_byte(crc, p.i_term_ma);
    crc = crc16_byte(crc, p.chg_eff_sub900);
    p.crc16 = crc;
    return p;
}

}  // namespace detail

// M5StickC Plus internal 120 mAh 3.7 V LiPo pouch (AXP192 4.20 V / 100 mA charger).
inline constexpr CellParams M5STICKC_PLUS_120MAH = detail::make_signed({
    {PARAMS_MAGIC_0, PARAMS_MAGIC_1},
    PARAMS_VERSION,
    PARAM_FLAG_TEMP_COMP,
    // 0%, 6.25%, 12.5%, 18.75%, 25%, 31.25%, 37.5%, 43.75%, 50%,
    // 56.25%, 62.5%, 68.75%, 75%, 81.25%, 87.5%, 93.75%, 100%
    {3300, 3625, 3700, 3726, 3750, 3775, 3795, 3816, 3840,
     3860, 3890, 3940, 3980, 4035, 4095, 4140, 4200},
    1200,   // 120.0 mAh
    280,    // 280 mOhm ohmic + protection FET
    140,    // 140 mOhm 1RC polarization
    35,     // 35 s relaxation tau
    4165,   // CV full threshold mV
    10,     // 10 mA taper termination
    95,     // 99.5% coulomb efficiency
    0,
});

// Generic 1S 4.20 V LiPo pouch (1000 mAh reference).
inline constexpr CellParams LIPO_4V2_1000MAH = detail::make_signed({
    {PARAMS_MAGIC_0, PARAMS_MAGIC_1},
    PARAMS_VERSION,
    PARAM_FLAG_TEMP_COMP,
    {3300, 3625, 3700, 3726, 3750, 3775, 3795, 3816, 3840,
     3860, 3890, 3940, 3980, 4035, 4095, 4140, 4200},
    10000,  // 1000.0 mAh
    80,     // 80 mOhm
    45,     // 45 mOhm
    40,     // 40 s
    4170,
    50,
    95,
    0,
});

// 18650 NMC Li-ion (3000 mAh reference).
inline constexpr CellParams NMC_18650_3000MAH = detail::make_signed({
    {PARAMS_MAGIC_0, PARAMS_MAGIC_1},
    PARAMS_VERSION,
    PARAM_FLAG_TEMP_COMP,
    {3000, 3420, 3540, 3610, 3660, 3705, 3745, 3785, 3825,
     3870, 3920, 3970, 4020, 4070, 4115, 4160, 4200},
    30000,  // 3000.0 mAh
    45,
    30,
    45,
    4175,
    100,
    96,
    0,
});

// LiFePO4 (LFP) 3.65 V cell (1000 mAh reference).
inline constexpr CellParams LFP_3V65_1000MAH = detail::make_signed({
    {PARAMS_MAGIC_0, PARAMS_MAGIC_1},
    PARAMS_VERSION,
    PARAM_FLAG_TEMP_COMP,
    {2800, 3160, 3215, 3240, 3260, 3275, 3285, 3290, 3295,
     3300, 3305, 3315, 3325, 3330, 3340, 3375, 3550},
    10000,
    50,
    35,
    60,
    3520,
    50,
    97,
    0,
});

}  // namespace od_soc::chemistry
