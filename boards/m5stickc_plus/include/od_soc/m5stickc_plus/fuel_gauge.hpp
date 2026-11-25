#pragma once

// High-level M5StickC Plus FuelGauge wrapper combining:
//  - `od soc code` (<= 512 B integer 1RC estimator)
//  - `od soc history` (8 B/event RTC ring -> caller-flushed flash ring)
//  - AXP192 hardware coulomb counter (200 Hz active / 25 Hz sleep) + self-heating proxy
//  - Pull-driven Core-0 execution (0 tasks, 0 heap, 0 flash writes while armed)
//  - Remote `ip-soc` 52-byte CellParams calibration intake

#include "od_soc/chemistry.hpp"
#include "od_soc/history.hpp"
#include "od_soc/ip_frame.hpp"
#include "od_soc/m5stickc_plus/axp192_sensor.hpp"
#include "od_soc/od_soc.hpp"

namespace od_soc::m5stickc_plus {

// All persistent RTC-RAM state bundled in one POD struct (put in RTC_DATA_ATTR on ESP32!).
struct RtcFuelGaugeState {
    uint32_t magic;                  // 0x4D355331 ("M5S1")
    OdSocState soc;                  // 16 B
    CoulombBaseline coulomb;         // 12 B
    RtcEventRing events;             // 136 B
    uint32_t last_step_s;
    uint32_t last_event_s;
    int32_t event_dq_accum_uah;
    uint8_t prev_usb_present;
    uint8_t prev_charging;
    uint8_t prev_charge_done;
    uint8_t params_dirty;
};

struct PollResult {
    bool updated;          // True when od_soc::step() actually ran this call
    uint16_t soc_cpct;     // 0..10000 (0.00%..100.00%)
    RawReading raw;        // Latest AXP192 telemetry burst
    int32_t dq_uah;        // Coulomb delta consumed this update
    IpRecord ip_record;    // Ready to stream to `ip soc history` when remote is connected
};

class FuelGauge {
  public:
    bool init(const I2cTransport& i2c, RtcFuelGaugeState& rtc, uint32_t now_s,
              const FlashOps* flash_history = nullptr,
              const CellParams* persisted_params = nullptr) noexcept;

    // Pull-driven Core-0 poll. Runs `od soc code` ONLY when `now_s - last_step_s >= min_period_s`
    // or a power-state event occurs (USB plug/unplug, charge start/done, low battery) or force=true.
    bool poll(uint32_t now_s, uint32_t min_period_s, bool force, PollResult& out) noexcept;

    // Call right before entering `wifi-sleep` or `usb-sleep` (drops AXP192 ADC to 25 Hz cleanly).
    bool prepare_sleep(uint32_t now_s, bool usb_sleep) noexcept;

    // Call on wake from wifi-sleep (restores AXP192 ADC to 200 Hz preserving coulombs).
    bool wake(uint32_t now_s) noexcept;

    // Records an explicit lifecycle event (ARM, DISARM, SHUTDOWN, THERMAL, FAULT).
    void log_event(EventType type, uint32_t now_s) noexcept;

    // Single-flash-writer intake: validates & applies 52-byte CellParams pushed by `ip-soc`.
    // Marks params_dirty so caller can persist to NVS when disarmed.
    bool apply_remote_params(const CellParams& params, uint32_t now_s) noexcept;

    // Call ONLY while disarmed! Flushes RTC event ring to flash history ring.
    bool flush_history_if_safe() noexcept;

    bool params_dirty() const noexcept { return rtc_ && rtc_->params_dirty != 0u; }
    void clear_params_dirty() noexcept { if (rtc_) rtc_->params_dirty = 0u; }
    const CellParams& params() const noexcept { return params_; }
    const OdSocState& state() const noexcept { return rtc_->soc; }
    const FlashHistoryRing& flash_history() const noexcept { return history_; }

  private:
    void push_od_event(EventType type, uint32_t now_s, const RawReading& raw) noexcept;

    Axp192Sensor sensor_{};
    FlashHistoryRing history_{};
    bool has_flash_history_ = false;
    RtcFuelGaugeState* rtc_ = nullptr;
    CellParams params_ = chemistry::M5STICKC_PLUS_120MAH;
    RawReading last_raw_{};
};

}  // namespace od_soc::m5stickc_plus
