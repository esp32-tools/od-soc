#pragma once

// Core `od soc code` API — verified <= 512 bytes Xtensa LX6 code flash (.text + .literal, -Os).

#include "od_soc/types.hpp"

namespace od_soc {

// Advances the integer-only 1RC scalar-adaptive observer by one step.
// Zero FPU, zero heap, zero expf/sqrtf, ~1.5 us @ 80 MHz, ~400 B Xtensa code flash.
// Returns estimated SOC in centi-percent (0..10000 => 0.00%..100.00%).
uint16_t step(OdSocState& st, const CellParams& params, const Sample& sample) noexcept;

// Inverse OCV lookup (0..65536 Q16 SOC) for cold-start seeding.
uint32_t soc_q16_from_ocv_mv(const uint16_t ocv_mv[OCV_POINTS], uint16_t v_ocv_mv) noexcept;

// Seeds state from a known Q16 SOC (0..65536) and initial variance in Q18 (default: sigma=15% => 5898).
void seed_q16(OdSocState& st, uint32_t z_q16, uint32_t p_q18 = 5898u) noexcept;

// Cold-start seeding directly from a voltage (mV) and current (mA) reading.
void seed_from_vi(OdSocState& st, const CellParams& params, uint16_t v_mv, int16_t i_ma) noexcept;

// Helper conversions (inline, zero code cost if unused).
inline float soc_percent(const OdSocState& st) noexcept {
    return static_cast<float>(st.z_q16) * (100.0f / 65536.0f);
}

inline uint8_t soc_half_percent(const OdSocState& st) noexcept {
    const uint32_t hp = (st.z_q16 * 200u + 32768u) >> 16;
    return hp > 200u ? 200u : static_cast<uint8_t>(hp);
}

// CRC-16/CCITT-FALSE signing & validation for 52-byte CellParams blobs from ip-soc.
uint16_t crc16_ccitt(const uint8_t* data, size_t len) noexcept;
void sign_params(CellParams& params) noexcept;
bool validate_params(const CellParams& params) noexcept;

}  // namespace od_soc
