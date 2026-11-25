#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "od_soc/chemistry.hpp"
#include "od_soc/history.hpp"
#include "od_soc/ip_frame.hpp"
#include "od_soc/m5stickc_plus/axp192_regs.hpp"
#include "od_soc/m5stickc_plus/fuel_gauge.hpp"
#include "od_soc/od_soc.hpp"
#include "od_soc/od_soc_c.h"
#include "plant.hpp"

using namespace od_soc;

namespace {

int failures = 0;

#define CHECK(cond, msg, ...)                                                         do {                                                                                  if (!(cond)) {                                                                        std::fprintf(stderr, "FAIL [%s:%d]: " msg "\n", __FILE__, __LINE__,                           ##__VA_ARGS__);                                                      ++failures;                                                                   }                                                                             } while (0)

void test_params_and_crc() {
    CHECK(validate_params(chemistry::M5STICKC_PLUS_120MAH), "M5STICKC_PLUS_120MAH CRC invalid");
    CHECK(validate_params(chemistry::LIPO_4V2_1000MAH), "LIPO_4V2_1000MAH CRC invalid");
    CHECK(validate_params(chemistry::NMC_18650_3000MAH), "NMC_18650_3000MAH CRC invalid");
    CHECK(validate_params(chemistry::LFP_3V65_1000MAH), "LFP_3V65_1000MAH CRC invalid");

    CellParams bad = chemistry::M5STICKC_PLUS_120MAH;
    bad.r0_mohm ^= 1u;
    CHECK(!validate_params(bad), "Corrupted CellParams should fail CRC");
    sign_params(bad);
    CHECK(validate_params(bad), "Re-signed CellParams should pass CRC");

    // Verify C99 ABI matches C++ directly.
    od_soc_state_t cst{};
    const auto* cp = reinterpret_cast<const od_soc_params_t*>(&chemistry::M5STICKC_PLUS_120MAH);
    CHECK(od_soc_validate_params(cp), "C ABI validate_params failed");
    od_soc_seed_from_vi(&cst, cp, 3840, 0);
    od_soc_sample_t cs{3840, 0, 0, 5, SAMPLE_HAS_DQ, 0};
    const uint16_t pct = od_soc_step(&cst, cp, &cs);
    CHECK(pct >= 4900 && pct <= 5100, "C ABI step expected ~50%%, got %u", pct);
}

void test_2rc_plant_discharge_and_wifi_bursts() {
    // Compare `od soc code` against a strictly harder 2RC + Plett hysteresis plant under
    // dynamic balancing + Wi-Fi TX burst load, starting with a 25% cold-start SOC error!
    sim::PlantConfig pcfg{};
    sim::BatteryPlant2RC plant(pcfg, 0.90f);

    OdSocState st{};
    seed_q16(st, static_cast<uint32_t>(0.65f * 65536.0f), 5898u);  // 25% initial error!

    double sq_err_sum = 0.0;
    int steady_n = 0;
    double naive_sq_err_sum = 0.0;

    // 40 minutes of 5-second steps (480 steps): alternating balancing, Wi-Fi TX spikes, and rests.
    for (int k = 0; k < 480; ++k) {
        float i_ma = -95.0f;
        if (k % 12 == 0) i_ma = -240.0f;       // Wi-Fi TX + motor recovery spike
        else if (k % 20 >= 16) i_ma = -12.0f;  // Brief idle pause

        // Sub-step plant at 1 Hz to accumulate exact coulomb delta across the 5 s cadence.
        float v_v = 0.0f;
        double dq_uah_f = 0.0;
        for (int s = 0; s < 5; ++s) {
            const float i_sub = (s == 4 && (k % 12 == 0)) ? -260.0f : i_ma;
            v_v = plant.step(i_sub, 1.0f, 30.0f);
            dq_uah_f += i_sub * (1000.0 / 3600.0);
        }

        const Sample smp{
            .v_mv = static_cast<uint16_t>(std::lround(v_v * 1000.0f)),
            .i_ma = static_cast<int16_t>(std::lround((k % 12 == 0) ? -260.0f : i_ma)),
            .dq_uah = static_cast<int32_t>(std::lround(dq_uah_f)),
            .dt_s = 5,
            .flags = SAMPLE_HAS_DQ,
            .reserved = 0,
        };
        const uint16_t est_cpct = step(st, chemistry::M5STICKC_PLUS_120MAH, smp);
        const double est_pct = est_cpct * 0.01;
        const double truth_pct = plant.true_soc_pct();

        // Naive stateless baseline (old battery_soc.h approach: V - I*0.30).
        const float naive_v_ocv = v_v - (smp.i_ma * 1e-3f) * 0.30f;
        const double naive_pct =
            soc_q16_from_ocv_mv(chemistry::M5STICKC_PLUS_120MAH.ocv_mv,
                                static_cast<uint16_t>(std::clamp(naive_v_ocv * 1000.0f, 0.0f, 5000.0f))) *
            (100.0 / 65536.0);

        if (k >= 60) {  // After initial convergence window (~5 min)
            const double err = est_pct - truth_pct;
            sq_err_sum += err * err;
            const double nerr = naive_pct - truth_pct;
            naive_sq_err_sum += nerr * nerr;
            ++steady_n;
        }
    }

    const double rmse = std::sqrt(sq_err_sum / steady_n);
    const double naive_rmse = std::sqrt(naive_sq_err_sum / steady_n);
    std::printf("  [bench] 2RC+hysteresis dynamic load: od-soc RMSE = %.2f%% (vs stateless naive = %.2f%%)\n",
                rmse, naive_rmse);
    CHECK(rmse < 2.0, "od-soc RMSE %.2f%% exceeded 2.0%% target", rmse);
    CHECK(rmse < naive_rmse * 0.5, "od-soc should decisively beat naive stateless baseline");
}

void test_glitch_rejection_and_deep_sleep() {
    sim::PlantConfig pcfg{};
    sim::BatteryPlant2RC plant(pcfg, 0.72f);

    OdSocState st{};
    seed_q16(st, static_cast<uint32_t>(0.72f * 65536.0f), 100u);

    // Inject a massive -400 mV I2C glitch on one tick — SOC should barely budge (< 1.5%).
    const float v_ok = plant.step(-80.0f, 5.0f, 25.0f);
    const Sample glitch{
        .v_mv = static_cast<uint16_t>(v_ok * 1000.0f - 400.0f),
        .i_ma = -80,
        .dq_uah = -111,
        .dt_s = 5,
        .flags = SAMPLE_HAS_DQ,
        .reserved = 0,
    };
    const uint16_t after_glitch = step(st, chemistry::M5STICKC_PLUS_120MAH, glitch);
    const double err_glitch = std::fabs(after_glitch * 0.01 - plant.true_soc_pct());
    CHECK(err_glitch < 1.5, "Glitch moved SOC too far: %.2f%% error", err_glitch);

    // 8-hour deep sleep (28,800 s) at 1.5 mA quiescent draw (12.0 mAh = 12,000 uAh).
    const float v_wake = plant.step(-1.5f, 28800.0f, 25.0f);
    const Sample wake_s{
        .v_mv = static_cast<uint16_t>(std::lround(v_wake * 1000.0f)),
        .i_ma = -2,
        .dq_uah = -12000,
        .dt_s = 28800,
        .flags = SAMPLE_HAS_DQ,
        .reserved = 0,
    };
    const uint16_t after_sleep = step(st, chemistry::M5STICKC_PLUS_120MAH, wake_s);
    const double err_sleep = std::fabs(after_sleep * 0.01 - plant.true_soc_pct());
    std::printf("  [bench] 8h deep-sleep wake catchup: est = %.2f%%, truth = %.2f%% (err = %.2f%%)\n",
                after_sleep * 0.01, plant.true_soc_pct(), err_sleep);
    CHECK(err_sleep < 1.5, "8h sleep catch-up error too high: %.2f%%", err_sleep);
}

void test_od_history_ring_and_offline_soh() {
    // 1. Test log-dt codec monotonicity & bounds.
    CHECK(decode_dt_log(encode_dt_log(0)) == 0, "dt=0");
    CHECK(decode_dt_log(encode_dt_log(59)) == 59, "dt=59");
    CHECK(decode_dt_log(encode_dt_log(300)) == 300, "dt=300");
    CHECK(decode_dt_log(encode_dt_log(3600)) == 3600, "dt=3600");
    CHECK(decode_dt_log(encode_dt_log(86400)) >= 86340 && decode_dt_log(encode_dt_log(86400)) <= 86460, "dt=1d");
    CHECK(decode_dt_log(encode_dt_log(1500000)) >= 1499000, "dt=17d");

    // 2. Test 8-byte PackedEvent roundtrip (cross-language vector!).
    const UnpackedEvent ev{
        .dt_s = 300,
        .type = EventType::CHG_DONE,
        .v_mv = 4195,
        .i_ma = -123,
        .dq_qmah = -205,
        .soc_half_pct = 167,
        .temp_half_c = encode_temp_half_c(34.5f),
    };
    const PackedEvent pe = pack_event(ev);
    const UnpackedEvent dec = unpack_event(pe);
    CHECK(dec.dt_s == 300 && dec.type == EventType::CHG_DONE && dec.v_mv == 4195 &&
          dec.i_ma == -123 && dec.dq_qmah == -205 && dec.soc_half_pct == 167 &&
          dec.temp_half_c == encode_temp_half_c(34.5f),
          "PackedEvent roundtrip mismatch");

    // 3. Test 2-sector mock flash ring (2 x 4096 B = 1024 slots), wraparound, recovery & offline SOH!
    struct MockFlash {
        std::vector<uint8_t> mem = std::vector<uint8_t>(8192, 0xFF);
        int erases = 0;
    } mf;

    FlashOps ops{
        .read = [](void* c, uint32_t off, void* out, size_t len) noexcept -> bool {
            auto* f = static_cast<MockFlash*>(c);
            if (off + len > f->mem.size()) return false;
            std::memcpy(out, f->mem.data() + off, len);
            return true;
        },
        .write = [](void* c, uint32_t off, const void* data, size_t len) noexcept -> bool {
            auto* f = static_cast<MockFlash*>(c);
            if (off + len > f->mem.size()) return false;
            const auto* src = static_cast<const uint8_t*>(data);
            for (size_t i = 0; i < len; ++i) f->mem[off + i] &= src[i];  // NOR flash AND semantics!
            return true;
        },
        .erase_sector = [](void* c, uint32_t sec_off) noexcept -> bool {
            auto* f = static_cast<MockFlash*>(c);
            if (sec_off + 4096 > f->mem.size()) return false;
            std::memset(f->mem.data() + sec_off, 0xFF, 4096);
            ++f->erases;
            return true;
        },
        .ctx = &mf,
        .total_bytes = 8192,
        .sector_bytes = 4096,
    };

    FlashHistoryRing ring;
    CHECK(ring.attach(ops), "attach flash ring failed");

    RtcEventRing rtc{};
    rtc.reset();
    // Simulate an aged 96 mAh cell charging from 20% (40 hp) to 100% (200 hp): delta Q = 76.8 mAh = 307 qmah.
    rtc.push(pack_event({10, EventType::CHG_START, 3730, 95, 0, 40, encode_temp_half_c(28.0f)}));
    rtc.push(pack_event({1800, EventType::PERIODIC, 3980, 95, 180, 120, encode_temp_half_c(32.0f)}));
    rtc.push(pack_event({1500, EventType::CHG_DONE, 4195, 8, 127, 200, encode_temp_half_c(29.0f)}));
    CHECK(ring.flush_rtc(rtc), "flush_rtc failed");
    CHECK(rtc.count == 0, "rtc count should reset after flush");
    CHECK(mf.erases == 0, "no sector erase needed on fresh erased flash");

    // Re-attach from scratch (simulates cold power loss) and verify recovery + offline SOH!
    FlashHistoryRing ring2;
    CHECK(ring2.attach(ops), "re-attach failed");
    PackedEvent latest{};
    CHECK(ring2.read_latest(latest), "read_latest failed");
    CHECK(unpack_event(latest).type == EventType::CHG_DONE, "latest event should be CHG_DONE");

    const uint16_t q_est = ring2.estimate_offline_qmax_dmah(1200);
    // 307 * 0.25 mAh / 0.80 = 95.9 mAh => ~959 dmah
    CHECK(q_est >= 940 && q_est <= 980, "offline SOH expected ~959 dmah, got %u", q_est);
}

void test_ip_frame_codec() {
    const IpRecord recs[2] = {
        {10000, 3850, -95, -132, 295, 0x10, 102},
        {15000, 3842, -100, -139, 297, 0x10, 101},
    };
    uint8_t buf[64]{};
    const size_t n = encode_ip_frame(42, 0x01, recs, 2, buf, sizeof(buf));
    CHECK(n == 12 + 32 + 2, "expected 46-byte SOC1 frame, got %zu", n);

    uint32_t seq = 0;
    uint8_t flags = 0;
    IpRecord out[2]{};
    const size_t cnt = decode_ip_frame(buf, n, seq, flags, out, 2);
    CHECK(cnt == 2 && seq == 42 && flags == 0x01 && out[1].v_mv == 3842 && out[1].dq_uah == -139,
          "SOC1 frame roundtrip failed");

    // Corrupt a byte -> CRC must reject.
    buf[20] ^= 0x40;
    CHECK(decode_ip_frame(buf, n, seq, flags, out, 2) == 0, "Corrupted SOC1 frame should fail CRC");
}

void test_m5stickc_plus_mock_axp192() {
    using namespace m5stickc_plus;

    struct MockAxp {
        uint8_t reg[256]{};
        void set_be12(uint8_t r, uint16_t v) {
            reg[r] = static_cast<uint8_t>((v >> 4) & 0xFF);
            reg[r + 1] = static_cast<uint8_t>(v & 0x0F);
        }
        void set_be13(uint8_t r, uint16_t v) {
            reg[r] = static_cast<uint8_t>((v >> 5) & 0xFF);
            reg[r + 1] = static_cast<uint8_t>(v & 0x1F);
        }
        void set_be32(uint8_t r, uint32_t v) {
            reg[r] = (v >> 24) & 0xFF;
            reg[r + 1] = (v >> 16) & 0xFF;
            reg[r + 2] = (v >> 8) & 0xFF;
            reg[r + 3] = v & 0xFF;
        }
    } axp;

    // Seed AXP192 registers: VBUS=5.0V (2941), Temp=38.0C (1827), Vbat=3.85V (3500),
    // I_dis=100 mA (200 in 0.5mA LSB), ADC rate=25 Hz default, coulomb disabled initially.
    axp.reg[axp192::REG_POWER_STATUS] = 0x00;
    axp.reg[axp192::REG_CHARGE_STATUS] = axp192::CHARGE_BAT_PRESENT;
    axp.set_be12(axp192::REG_VBUS_V_H, 0);
    axp.set_be12(axp192::REG_VBUS_I_H, 0);
    axp.set_be12(axp192::REG_TEMP_H, 1827);     // 38.0 C die
    axp.set_be12(axp192::REG_BAT_V_H, 3500);    // 3850 mV
    axp.set_be13(axp192::REG_BAT_CHG_H, 0);
    axp.set_be13(axp192::REG_BAT_DIS_H, 200);   // -100 mA
    axp.reg[axp192::REG_ADC_RATE] = axp192::ADC_RATE_25HZ;
    axp.reg[axp192::REG_COULOMB_CTRL] = 0x00;

    I2cTransport bus{
        .read_reg = [](void* c, uint8_t r, uint8_t* out, size_t len) noexcept -> bool {
            auto* m = static_cast<MockAxp*>(c);
            std::memcpy(out, m->reg + r, len);
            return true;
        },
        .write_reg = [](void* c, uint8_t r, uint8_t val) noexcept -> bool {
            auto* m = static_cast<MockAxp*>(c);
            m->reg[r] = val;
            return true;
        },
        .ctx = &axp,
    };

    RtcFuelGaugeState rtc{};
    FuelGauge fg;
    CHECK(fg.init(bus, rtc, 100), "FuelGauge cold init failed");
    CHECK((axp.reg[axp192::REG_COULOMB_CTRL] & axp192::COULOMB_ENABLE) != 0, "Coulomb counter not enabled");
    CHECK((axp.reg[axp192::REG_ADC_RATE] & axp192::ADC_RATE_MASK) == axp192::ADC_RATE_200HZ,
          "Active ADC rate should switch to 200 Hz");
    CHECK(rtc.events.count == 1 && unpack_event(rtc.events.entries[0]).type == EventType::BOOT,
          "BOOT event missing");

    // Poll before cadence (dt = 2 < 5): must skip od_soc::step() (updated == false)!
    PollResult pr{};
    CHECK(fg.poll(102, 5, false, pr) && !pr.updated, "Should skip step before min_period_s");

    // Advance 200 Hz coulomb discharge counter by 100 ticks (~4.55 mAh = 4551 uAh) + 160 s.
    axp.set_be32(axp192::REG_COULOMB_DIS_0, 100);
    axp.set_be12(axp192::REG_BAT_V_H, 3475);  // 3823 mV
    CHECK(fg.poll(260, 5, false, pr) && pr.updated, "Poll at cadence should update");
    CHECK(pr.dq_uah <= -4540 && pr.dq_uah >= -4560, "Expected ~-4551 uAh @ 200Hz, got %d", pr.dq_uah);
    CHECK(pr.raw.cell_est_dc == 330, "Self-heating compensated cell temp expected 33.0C (330), got %d",
          pr.raw.cell_est_dc);

    // Prepare sleep -> drops ADC rate to 25 Hz!
    CHECK(fg.prepare_sleep(270, true), "prepare_sleep failed");
    CHECK((axp.reg[axp192::REG_ADC_RATE] & axp192::ADC_RATE_MASK) == axp192::ADC_RATE_25HZ,
          "Sleep ADC rate should drop to 25 Hz");

    // Simulate 10 ticks at 25 Hz during deep sleep (~3.64 mAh = 3641 uAh), then warm wake!
    axp.set_be32(axp192::REG_COULOMB_DIS_0, 110);
    FuelGauge fg_wake;
    CHECK(fg_wake.init(bus, rtc, 3870), "Warm wake init failed");
    CHECK((axp.reg[axp192::REG_ADC_RATE] & axp192::ADC_RATE_MASK) == axp192::ADC_RATE_200HZ,
          "Wake should restore 200 Hz ADC rate");

    // Apply calibrated CellParams from ip-soc -> must validate CRC and mark params_dirty!
    CellParams cal = chemistry::M5STICKC_PLUS_120MAH;
    cal.q_max_dmah = 1050;
    cal.flags |= PARAM_FLAG_CALIBRATED;
    sign_params(cal);
    CHECK(fg_wake.apply_remote_params(cal, 3880), "apply_remote_params failed");
    CHECK(fg_wake.params_dirty(), "params_dirty should be set for caller NVS flush");
    CHECK(fg_wake.params().q_max_dmah == 1050, "calibrated q_max not applied");
}

}  // namespace

int main() {
    std::printf("Running esp32-tools/od-soc test suite...\n");
    test_params_and_crc();
    test_2rc_plant_discharge_and_wifi_bursts();
    test_glitch_rejection_and_deep_sleep();
    test_od_history_ring_and_offline_soh();
    test_ip_frame_codec();
    test_m5stickc_plus_mock_axp192();

    if (failures > 0) {
        std::fprintf(stderr, "FAILED: %d check(s)\n", failures);
        return 1;
    }
    std::printf("ALL od-soc TESTS PASSED\n");
    return 0;
}
