#include "od_soc/m5stickc_plus/axp192_sensor.hpp"
#include "od_soc/m5stickc_plus/axp192_regs.hpp"

namespace od_soc::m5stickc_plus {

namespace {

inline constexpr uint16_t COULOMB_MAGIC = 0xC01Bu;

inline uint16_t decode_12bit(const uint8_t* b) noexcept {
    return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 4) | (b[1] & 0x0Fu));
}

inline uint16_t decode_13bit(const uint8_t* b) noexcept {
    return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 5) | (b[1] & 0x1Fu));
}

inline uint32_t decode_be32(const uint8_t* b) noexcept {
    return (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
           (static_cast<uint32_t>(b[2]) << 8)  | static_cast<uint32_t>(b[3]);
}

inline uint8_t rate_bits_for_hz(uint16_t hz) noexcept {
    if (hz >= 200) return axp192::ADC_RATE_200HZ;
    if (hz >= 100) return axp192::ADC_RATE_100HZ;
    if (hz >= 50)  return axp192::ADC_RATE_50HZ;
    return axp192::ADC_RATE_25HZ;
}

}  // namespace

bool Axp192Sensor::attach(const I2cTransport& transport) noexcept {
    if (!transport.read_reg || !transport.write_reg) {
        attached_ = false;
        return false;
    }
    bus_ = transport;
    attached_ = true;
    return true;
}

bool Axp192Sensor::ensure_coulomb_enabled(bool& was_already_running) noexcept {
    was_already_running = false;
    if (!attached_) return false;
    uint8_t ctrl = 0;
    if (!bus_.read_reg(bus_.ctx, axp192::REG_COULOMB_CTRL, &ctrl, 1)) return false;
    if ((ctrl & axp192::COULOMB_ENABLE) != 0u) {
        was_already_running = true;
        return true;
    }
    // Cold power-on: enable and clear counter atomically (0xA0 keeps bit 7 set!).
    return bus_.write_reg(bus_.ctx, axp192::REG_COULOMB_CTRL,
                          axp192::COULOMB_ENABLE | axp192::COULOMB_CLEAR);
}

uint16_t Axp192Sensor::read_adc_hz() const noexcept {
    if (!attached_) return 25;
    uint8_t reg84 = 0;
    if (!bus_.read_reg(bus_.ctx, axp192::REG_ADC_RATE, &reg84, 1)) return 25;
    return static_cast<uint16_t>(25u << ((reg84 >> 6) & 0x03u));
}

int16_t Axp192Sensor::estimate_cell_temp_dc(int16_t pmic_dc, uint16_t usb_mv, uint16_t bat_mv,
                                            int16_t bat_ma) noexcept {
    // M5StickC Plus has no battery thermistor; AXP192 die runs ~5 C above pouch ambient,
    // plus linear-charger dissipation: P = (Vbus - Vbat) * Ichg, ~35 C/W die-to-cell rise.
    int32_t offset_dc = 50;  // 5.0 C baseline buck/LDO self-heating
    if (bat_ma > 0 && usb_mv > bat_mv) {
        const int32_t p_mw = (static_cast<int32_t>(usb_mv - bat_mv) * bat_ma) / 1000;
        offset_dc += (p_mw * 35) / 100;  // 35 C/W -> 0.35 (0.1 C) / mW
    }
    int32_t est = static_cast<int32_t>(pmic_dc) - offset_dc;
    if (est < -200) est = -200;
    if (est > 1000) est = 1000;
    return static_cast<int16_t>(est);
}

bool Axp192Sensor::read_telemetry(RawReading& out) const noexcept {
    if (!attached_) return false;
    uint8_t st[2];
    uint8_t vbus_temp[6];
    uint8_t bat[6];
    if (!bus_.read_reg(bus_.ctx, axp192::REG_POWER_STATUS, st, sizeof(st))) return false;
    if (!bus_.read_reg(bus_.ctx, axp192::REG_VBUS_V_H, vbus_temp, sizeof(vbus_temp))) return false;
    if (!bus_.read_reg(bus_.ctx, axp192::REG_BAT_V_H, bat, sizeof(bat))) return false;

    const uint16_t usb_v_raw = decode_12bit(vbus_temp + 0);
    const uint16_t usb_i_raw = decode_12bit(vbus_temp + 2);
    const uint16_t temp_raw  = decode_12bit(vbus_temp + 4);

    const uint16_t bat_v_raw   = decode_12bit(bat + 0);
    const uint16_t bat_chg_raw = decode_13bit(bat + 2);
    const uint16_t bat_dis_raw = decode_13bit(bat + 4);

    out.usb_mv = static_cast<uint16_t>((static_cast<uint32_t>(usb_v_raw) * 17u + 5u) / 10u);
    out.usb_ma = static_cast<uint16_t>((static_cast<uint32_t>(usb_i_raw) * 3u + 4u) / 8u);
    out.pmic_dc = static_cast<int16_t>(static_cast<int32_t>(temp_raw) - 1447);

    out.bat_mv = static_cast<uint16_t>((static_cast<uint32_t>(bat_v_raw) * 11u + 5u) / 10u);
    out.bat_ma = static_cast<int16_t>((static_cast<int32_t>(bat_chg_raw) - static_cast<int32_t>(bat_dis_raw)) / 2);

    out.usb_present = (st[0] & axp192::POWER_VBUS_PRESENT) != 0u;
    out.bat_present = (st[1] & axp192::CHARGE_BAT_PRESENT) != 0u;
    out.charging    = (st[1] & axp192::CHARGE_CHARGING) != 0u;
    out.charge_done = out.usb_present && out.bat_present && !out.charging && out.bat_mv >= 4130u;
    out.cell_est_dc = estimate_cell_temp_dc(out.pmic_dc, out.usb_mv, out.bat_mv, out.bat_ma);
    return true;
}

bool Axp192Sensor::read_coulomb_delta_uah(CoulombBaseline& baseline, int32_t& out_dq_uah) const noexcept {
    out_dq_uah = 0;
    if (!attached_) return false;
    uint8_t raw[8];
    if (!bus_.read_reg(bus_.ctx, axp192::REG_COULOMB_CHG_0, raw, sizeof(raw))) return false;
    const uint32_t c_chg = decode_be32(raw + 0);
    const uint32_t c_dis = decode_be32(raw + 4);
    const uint16_t hz = read_adc_hz();

    if (baseline.valid_magic != COULOMB_MAGIC || baseline.adc_hz == 0) {
        baseline.c_chg = c_chg;
        baseline.c_dis = c_dis;
        baseline.adc_hz = hz;
        baseline.valid_magic = COULOMB_MAGIC;
        return true;
    }

    // Unsigned modular 32-bit deltas handle counter wraparound cleanly without ever clearing 0xB8.
    const uint32_t d_chg = c_chg - baseline.c_chg;
    const uint32_t d_dis = c_dis - baseline.c_dis;
    const int64_t net_ticks = static_cast<int64_t>(d_chg) - static_cast<int64_t>(d_dis);

    // Datasheet formula: C(mAh) = 65536 * 0.5 * net_ticks / (3600 * f_adc)
    // In uAh: C(uAh) = net_ticks * 32768000 / (3600 * f_adc) = net_ticks * 81920 / (9 * f_adc).
    const int64_t denom = 9LL * static_cast<int64_t>(baseline.adc_hz);
    const int64_t uah = (net_ticks * 81920LL) / denom;
    out_dq_uah = static_cast<int32_t>(uah);

    baseline.c_chg = c_chg;
    baseline.c_dis = c_dis;
    baseline.adc_hz = hz;
    return true;
}

bool Axp192Sensor::set_adc_hz_preserving_coulombs(uint16_t target_hz, CoulombBaseline& baseline,
                                                  int32_t& out_harvested_uah) const noexcept {
    out_harvested_uah = 0;
    if (!attached_) return false;
    // 1. Harvest pending delta at the OLD ADC rate before changing REG 0x84!
    if (!read_coulomb_delta_uah(baseline, out_harvested_uah)) return false;

    uint8_t reg84 = 0;
    if (!bus_.read_reg(bus_.ctx, axp192::REG_ADC_RATE, &reg84, 1)) return false;
    const uint8_t next = static_cast<uint8_t>((reg84 & ~axp192::ADC_RATE_MASK) | rate_bits_for_hz(target_hz));
    if (next != reg84) {
        if (!bus_.write_reg(bus_.ctx, axp192::REG_ADC_RATE, next)) return false;
    }
    baseline.adc_hz = static_cast<uint16_t>(25u << ((next >> 6) & 0x03u));
    return true;
}

}  // namespace od_soc::m5stickc_plus
