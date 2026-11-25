#pragma once

// esp32-tools/od-soc — On-Device State of Charge (`od soc code` + `od soc history`).
// Zero heap, zero FPU, zero exceptions, zero RTOS/HAL dependencies.

#include <cstddef>
#include <cstdint>

namespace od_soc {

inline constexpr size_t OCV_POINTS = 17;  // Uniform grid: 0%, 6.25%, ..., 100%

inline constexpr uint8_t PARAMS_MAGIC_0 = 'O';
inline constexpr uint8_t PARAMS_MAGIC_1 = 'S';
inline constexpr uint8_t PARAMS_VERSION = 1;

inline constexpr uint8_t PARAM_FLAG_CALIBRATED = 1u << 0;  // Fitted by esp32-tools/ip-soc
inline constexpr uint8_t PARAM_FLAG_TEMP_COMP  = 1u << 1;

inline constexpr uint8_t SAMPLE_HAS_DQ      = 1u << 0;  // dq_uah comes from HW coulomb counter
inline constexpr uint8_t SAMPLE_CHARGE_DONE = 1u << 1;  // Charger IC reports termination
inline constexpr uint8_t SAMPLE_CHARGING    = 1u << 2;  // External power charging active

// Exact 52-byte little-endian calibration payload shared bit-for-bit with esp32-tools/ip-soc.
// Naturally 2-byte aligned (zero unaligned loads on Xtensa LX6).
struct CellParams {
    uint8_t magic[2];              // "OS"                                  [0..1]
    uint8_t version;               // PARAMS_VERSION (1)                    [2]
    uint8_t flags;                 // PARAM_FLAG_*                          [3]
    uint16_t ocv_mv[OCV_POINTS];   // 0%, 6.25%, ..., 100% OCV in mV        [4..37]
    uint16_t q_max_dmah;           // Usable capacity in 0.1 mAh (1200=120) [38..39]
    uint16_t r0_mohm;              // Ohmic resistance at 25 C in mOhm      [40..41]
    uint16_t r1_mohm;              // 1RC polarization resistance in mOhm   [42..43]
    uint16_t tau_s;                // 1RC relaxation time constant (s)      [44..45]
    uint16_t v_full_mv;            // CV full-charge voltage threshold (mV) [46..47]
    uint8_t i_term_ma;             // CV taper current threshold (mA)       [48]
    uint8_t chg_eff_sub900;        // Charge eff = (900 + val) / 1000       [49]
    uint16_t crc16;                // CRC-16/CCITT-FALSE over bytes [0..49] [50..51]
};
static_assert(sizeof(CellParams) == 52, "CellParams wire layout must be exactly 52 bytes");

// Integer-native measurement sample.
// Sign convention: i_ma > 0 = charging, i_ma < 0 = discharging.
//                  dq_uah > 0 = charge added, dq_uah < 0 = charge removed.
struct Sample {
    uint16_t v_mv;       // Terminal voltage in mV (e.g. 3850)
    int16_t i_ma;        // Instantaneous current in mA (+ charge, - discharge)
    int32_t dq_uah;      // Hardware coulomb delta in uAh (valid when SAMPLE_HAS_DQ set)
    uint16_t dt_s;       // Elapsed seconds since previous step (0..65535)
    uint8_t flags;       // SAMPLE_*
    uint8_t reserved;
};

// 16-byte POD state for `od soc code` — drops directly into RTC_DATA_ATTR!
// Pure 32-bit integer representation: zero FPU usage, zero ESP32 FPU core-pinning hazard.
struct OdSocState {
    uint32_t z_q16;        // SOC in [0, 65536] (1 LSB = 0.00153%)
    int32_t v_rc_q4;       // 1RC polarization voltage in 1/16 mV (62.5 uV/LSB)
    uint32_t p_q18;        // Scalar SOC error variance in Q18 (0.09 = 23593, 2e-5 = 5)
    int16_t i_prev_ma;     // Previous current sample (for trapezoidal fallback)
    uint16_t sample_count; // Wrapping step counter (0 == unseeded)
};
static_assert(sizeof(OdSocState) == 16, "OdSocState must stay 16 bytes");

}  // namespace od_soc
