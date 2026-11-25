#include "od_soc/od_soc.hpp"

namespace od_soc {

uint32_t soc_q16_from_ocv_mv(const uint16_t ocv_mv[OCV_POINTS], uint16_t v_ocv_mv) noexcept {
    if (v_ocv_mv <= ocv_mv[0]) return 0u;
    if (v_ocv_mv >= ocv_mv[OCV_POINTS - 1]) return 65536u;
    for (size_t i = 1; i < OCV_POINTS; ++i) {
        if (v_ocv_mv < ocv_mv[i]) {
            const uint32_t lo = ocv_mv[i - 1];
            const uint32_t hi = ocv_mv[i];
            const uint32_t frac = hi > lo ? ((static_cast<uint32_t>(v_ocv_mv) - lo) << 12) / (hi - lo) : 0u;
            return (static_cast<uint32_t>(i - 1) << 12) + frac;
        }
    }
    return 65536u;
}

void seed_q16(OdSocState& st, uint32_t z_q16, uint32_t p_q18) noexcept {
    st.z_q16 = z_q16 > 65536u ? 65536u : z_q16;
    st.v_rc_q4 = 0;
    st.p_q18 = p_q18 < 5u ? 5u : (p_q18 > 23593u ? 23593u : p_q18);
    st.i_prev_ma = 0;
    st.sample_count = 1;
}

void seed_from_vi(OdSocState& st, const CellParams& params, uint16_t v_mv, int16_t i_ma) noexcept {
    const int32_t r_tot = static_cast<int32_t>(params.r0_mohm) + params.r1_mohm;
    int32_t v_est = static_cast<int32_t>(v_mv) - ((static_cast<int32_t>(i_ma) * r_tot) >> 10);
    if (v_est < 0) v_est = 0;
    if (v_est > 65535) v_est = 65535;
    const uint32_t p_init = (i_ma > 10 || i_ma < -10) ? 5898u : 420u;  // 15% vs 4% sigma
    seed_q16(st, soc_q16_from_ocv_mv(params.ocv_mv, static_cast<uint16_t>(v_est)), p_init);
    st.i_prev_ma = i_ma;
}

uint16_t crc16_ccitt(const uint8_t* data, size_t len) noexcept {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) != 0u ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                        : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void sign_params(CellParams& params) noexcept {
    params.magic[0] = PARAMS_MAGIC_0;
    params.magic[1] = PARAMS_MAGIC_1;
    params.version = PARAMS_VERSION;
    params.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&params), sizeof(CellParams) - 2);
}

bool validate_params(const CellParams& params) noexcept {
    if (params.magic[0] != PARAMS_MAGIC_0 || params.magic[1] != PARAMS_MAGIC_1) return false;
    if (params.version != PARAMS_VERSION || params.q_max_dmah < 10) return false;
    for (size_t i = 1; i < OCV_POINTS; ++i) {
        if (params.ocv_mv[i] < params.ocv_mv[i - 1]) return false;
    }
    const uint16_t expected =
        crc16_ccitt(reinterpret_cast<const uint8_t*>(&params), sizeof(CellParams) - 2);
    return expected == params.crc16;
}

}  // namespace od_soc
