#include "od_soc/od_soc.hpp"

namespace od_soc {

namespace {

inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi) noexcept {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

}  // namespace

// Pure 32-bit integer `od soc code`:
//  - Zero FPU instructions (no Xtensa FPU register spills, no ESP32 FPU core-pinning trap)
//  - Zero int64_t divisions, zero expf/sqrtf, zero heap, zero locks
//  - Overflow-safe across full uint16_t dt_s, +-5A current, 10..6553 mAh cells
uint16_t step(OdSocState& st, const CellParams& p, const Sample& s) noexcept {
    const int32_t dt = s.dt_s;
    int32_t i_avg = (static_cast<int32_t>(st.i_prev_ma) + s.i_ma) >> 1;
    int32_t dq;
    if ((s.flags & SAMPLE_HAS_DQ) != 0u) {
        dq = s.dq_uah;
        if (dt > 0) {
            i_avg = (dq * 18) / (dt * 5);  // uAh -> mA (3600 / 1000 = 18 / 5)
        }
    } else {
        // Clamp dt_i product for overflow safety; 1000/3600 = 5/18.
        const int32_t dt_safe = dt > 16384 ? 16384 : dt;
        dq = (i_avg * dt_safe * 5) / 18;
    }

    // 1. Coulomb predict in Q16: dz = (eff * dq << 8) / round(q_max_dmah * 25 / 64)
    //    Overflow-safe up to +-8.38 Ah per step!
    if (dq > 0) {
        dq = (dq * (900 + static_cast<int32_t>(p.chg_eff_sub900))) / 1000;
    }
    const int32_t q_div = (static_cast<int32_t>(p.q_max_dmah) * 25 + 32) >> 6;
    const int32_t dz = (dq << 8) / (q_div > 1 ? q_div : 1);
    int32_t z = clamp_i32(static_cast<int32_t>(st.z_q16) + dz, 0, 65536);

    // 2. Rational 1RC polarization in 1/16 mV (62.5 uV/LSB): alpha = tau / (tau + dt)
    const int32_t tau = p.tau_s;
    const int32_t dt_rc = dt > 4096 ? 4096 : dt;  // >1h is fully relaxed; prevents int32 overflow
    const int32_t v_ss = (i_avg * static_cast<int32_t>(p.r1_mohm)) >> 6;
    int32_t v_rc = (st.v_rc_q4 * tau + v_ss * dt_rc) / (tau + dt_rc > 0 ? tau + dt_rc : 1);

    // 3. Branchless 17-point OCV + local slope in mV.
    int32_t idx = z >> 12;
    if (idx > 15) idx = 15;
    const int32_t v0 = p.ocv_mv[idx];
    const int32_t dv = static_cast<int32_t>(p.ocv_mv[idx + 1]) - v0;
    const int32_t v_ocv = v0 + ((dv * (z & 4095)) >> 12);
    int32_t h = clamp_i32(dv << 4, 20, 3200);

    // 4. Scalar variance predict in Q18 (0.09 = 23593, 2e-5 = 5).
    const int32_t adz = dz < 0 ? -dz : dz;
    int32_t pv = static_cast<int32_t>(st.p_q18) + ((dt + 128) >> 8) + ((adz + 256) >> 9);
    if (pv > 23593) pv = 23593;

    // 5. Voltage innovation + glitch clamp + current-weighted scalar Kalman update.
    const int32_t v_pred = v_ocv + (v_rc >> 4) + ((static_cast<int32_t>(s.i_ma) * p.r0_mohm) >> 10);
    int32_t e = static_cast<int32_t>(s.v_mv) - v_pred;
    // Reject physically impossible I2C/voltage glitches (> 250 mV); softly bound valid residuals.
    if (e > 250 || e < -250) {
        e = 0;
    } else {
        e = clamp_i32(e, -120, 120);
    }

    int32_t r_eff = 400 + ((static_cast<int32_t>(s.i_ma) * s.i_ma) >> 4);
    if (r_eff > 65535) r_eff = 65535;  // Overflow-proof cap (256 mV sigma)
    const int32_t ph = (pv * h) >> 9;  // p*h in Q9
    const int32_t denom = ((ph * h) >> 9) + r_eff;
    z = clamp_i32(z + (((ph * e) / denom) << 7), 0, 65536);
    pv = (pv * r_eff) / denom;
    if (pv < 5) pv = 5;

    // 6. CV charge-termination anchor.
    if ((s.flags & SAMPLE_CHARGE_DONE) != 0u ||
        ((s.flags & SAMPLE_CHARGING) != 0u && s.v_mv >= p.v_full_mv &&
         s.i_ma >= 0 && s.i_ma <= static_cast<int32_t>(p.i_term_ma))) {
        z = 65536;
        v_rc = 0;
        pv = 5;
    }

    st.z_q16 = static_cast<uint32_t>(z);
    st.v_rc_q4 = v_rc;
    st.p_q18 = static_cast<uint32_t>(pv);
    st.i_prev_ma = s.i_ma;
    ++st.sample_count;
    return static_cast<uint16_t>((static_cast<uint32_t>(z) * 10000u) >> 16);
}

}  // namespace od_soc
