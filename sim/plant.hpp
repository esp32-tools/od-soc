#pragma once

// Higher-order 2RC + Plett 1-state Hysteresis + Arrhenius Thermal + Aging battery plant
// (Host-only simulator so we benchmark 1RC `od soc code` against a strictly harder physical truth!)

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "od_soc/types.hpp"

namespace od_soc::sim {

struct PlantConfig {
    float q_max_mah = 120.0f;
    float r0_25c_ohm = 0.28f;
    float r1_25c_ohm = 0.10f;
    float tau1_s = 12.0f;
    float r2_25c_ohm = 0.08f;
    float tau2_s = 120.0f;
    float hyst_max_v = 0.018f;   // 18 mV Plett hysteresis amplitude
    float hyst_gamma = 35.0f;    // Hysteresis decay rate
    float arrhenius_k = 1800.0f; // Resistance temp sensitivity (K)
    float chg_eff = 0.995f;
    uint16_t ocv_mv[OCV_POINTS] = {
        3300, 3625, 3700, 3726, 3750, 3775, 3795, 3816, 3840,
        3860, 3890, 3940, 3980, 4035, 4095, 4140, 4200,
    };
};

class BatteryPlant2RC {
  public:
    explicit BatteryPlant2RC(const PlantConfig& cfg, float initial_soc = 0.85f)
        : cfg_(cfg), z_(std::clamp(initial_soc, 0.0f, 1.0f)) {}

    // Advances truth plant by dt_s at current i_ma (+ charge, - discharge) and temperature temp_c.
    // Returns terminal voltage in Volts.
    float step(float i_ma, float dt_s, float temp_c = 25.0f) {
        const float i_a = i_ma * 1e-3f;
        const float dq_mah = i_ma * (dt_s / 3600.0f);
        const float eff = dq_mah > 0.0f ? cfg_.chg_eff : 1.0f;
        z_ = std::clamp(z_ + (eff * dq_mah) / cfg_.q_max_mah, 0.0f, 1.0f);

        const float tk = std::clamp(temp_c, -10.0f, 60.0f) + 273.15f;
        const float kr = std::exp(cfg_.arrhenius_k * (1.0f / tk - 1.0f / 298.15f));

        const float a1 = std::exp(-dt_s / cfg_.tau1_s);
        const float a2 = std::exp(-dt_s / cfg_.tau2_s);
        v1_ = a1 * v1_ + (1.0f - a1) * (cfg_.r1_25c_ohm * kr) * i_a;
        v2_ = a2 * v2_ + (1.0f - a2) * (cfg_.r2_25c_ohm * kr) * i_a;

        // Plett dynamic hysteresis state h in [-1, +1]
        const float dz_abs = std::fabs(dq_mah) / cfg_.q_max_mah;
        const float ah = std::exp(-cfg_.hyst_gamma * dz_abs);
        const float sgn = i_ma > 0.5f ? 1.0f : (i_ma < -0.5f ? -1.0f : 0.0f);
        if (sgn != 0.0f) {
            h_ = ah * h_ + (1.0f - ah) * sgn;
        }

        const float v_ocv = eval_ocv(z_);
        return v_ocv + v1_ + v2_ + cfg_.hyst_max_v * h_ + i_a * (cfg_.r0_25c_ohm * kr);
    }

    float true_soc() const { return z_; }
    float true_soc_pct() const { return z_ * 100.0f; }

  private:
    float eval_ocv(float z) const {
        const float u = std::clamp(z, 0.0f, 1.0f) * 16.0f;
        int idx = static_cast<int>(u);
        if (idx > 15) idx = 15;
        const float f = u - static_cast<float>(idx);
        const float v0 = static_cast<float>(cfg_.ocv_mv[idx]);
        const float v1 = static_cast<float>(cfg_.ocv_mv[idx + 1]);
        return (v0 + f * (v1 - v0)) * 1e-3f;
    }

    PlantConfig cfg_;
    float z_ = 0.85f;
    float v1_ = 0.0f;
    float v2_ = 0.0f;
    float h_ = 0.0f;
};

}  // namespace od_soc::sim
