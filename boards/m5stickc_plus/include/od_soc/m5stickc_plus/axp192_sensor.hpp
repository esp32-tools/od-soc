#pragma once

// Standalone AXP192 burst-read & hardware coulomb counter driver for M5StickC Plus.
// Zero dependency on esp32-tools/tools; works via a 2-function I2cTransport (or ESP-IDF i2c_master).

#include <cstddef>
#include <cstdint>

namespace od_soc::m5stickc_plus {

// Minimal vtable-free I2C transport (works with ESP-IDF i2c_master OR host mock registers).
struct I2cTransport {
    bool (*read_reg)(void* ctx, uint8_t reg, uint8_t* out, size_t len) noexcept;
    bool (*write_reg)(void* ctx, uint8_t reg, uint8_t val) noexcept;
    void* ctx;
};

struct RawReading {
    uint16_t bat_mv;
    int16_t bat_ma;        // + charge, - discharge
    uint16_t usb_mv;
    uint16_t usb_ma;
    int16_t pmic_dc;       // AXP192 die temperature in 0.1 C
    int16_t cell_est_dc;   // Estimated pouch cell temperature in 0.1 C (self-heating compensated)
    bool usb_present;
    bool bat_present;
    bool charging;
    bool charge_done;
};

// Persisted in RTC_DATA_ATTR across deep sleep so sleep coulombs are harvested cleanly on wake.
struct CoulombBaseline {
    uint32_t c_chg;
    uint32_t c_dis;
    uint16_t adc_hz;
    uint16_t valid_magic;  // 0xC01B when valid
};

class Axp192Sensor {
  public:
    bool attach(const I2cTransport& transport) noexcept;

    // Ensures battery ADCs + coulomb counter are active.
    // Returns true if the counter was ALREADY enabled (warm reset / deep-sleep wake!).
    bool ensure_coulomb_enabled(bool& was_already_running) noexcept;

    // Reads active ADC sample rate from REG 0x84 (25, 50, 100, or 200 Hz).
    uint16_t read_adc_hz() const noexcept;

    // Contiguous burst reads: 0x00..0x01 (2B), 0x5A..0x5F (6B), 0x78..0x7D (6B).
    // Cuts I2C0 bus hold time vs. 5 separate per-register transactions!
    bool read_telemetry(RawReading& out) const noexcept;

    // Reads 8-byte contiguous hardware coulomb block (0xB0..0xB7) and returns net delta uAh
    // since `baseline`, updating `baseline` without clearing hardware registers (zero lost counts!).
    bool read_coulomb_delta_uah(CoulombBaseline& baseline, int32_t& out_dq_uah) const noexcept;

    // Safely switches AXP192 ADC rate (e.g. 200 Hz active <-> 25 Hz sleep):
    // harvests any pending delta at the OLD rate first, updates REG 0x84, and updates `baseline`.
    bool set_adc_hz_preserving_coulombs(uint16_t target_hz, CoulombBaseline& baseline,
                                        int32_t& out_harvested_uah) const noexcept;

    // Estimates pouch cell temperature (0.1 C) from AXP192 die temp minus linear-charger heat.
    static int16_t estimate_cell_temp_dc(int16_t pmic_dc, uint16_t usb_mv, uint16_t bat_mv,
                                         int16_t bat_ma) noexcept;

  private:
    I2cTransport bus_{};
    bool attached_ = false;
};

}  // namespace od_soc::m5stickc_plus
