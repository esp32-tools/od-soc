#include "od_soc/m5stickc_plus/fuel_gauge.hpp"

namespace od_soc::m5stickc_plus {

namespace {
inline constexpr uint32_t RTC_FG_MAGIC = 0x4D355331u;  // "M5S1"
}  // namespace

bool FuelGauge::init(const I2cTransport& i2c, RtcFuelGaugeState& rtc, uint32_t now_s,
                     const FlashOps* flash_history, const CellParams* persisted_params) noexcept {
    rtc_ = &rtc;
    if (!sensor_.attach(i2c)) return false;

    if (persisted_params && validate_params(*persisted_params)) {
        params_ = *persisted_params;
    } else {
        params_ = chemistry::M5STICKC_PLUS_120MAH;
    }

    has_flash_history_ = false;
    if (flash_history && history_.attach(*flash_history)) {
        has_flash_history_ = true;
    }

    bool coulomb_was_running = false;
    if (!sensor_.ensure_coulomb_enabled(coulomb_was_running)) return false;

    RawReading raw{};
    if (!sensor_.read_telemetry(raw)) return false;
    last_raw_ = raw;

    const bool warm_rtc = (rtc.magic == RTC_FG_MAGIC) && (rtc.soc.sample_count > 0);
    if (warm_rtc && coulomb_was_running) {
        // Warm wake from deep sleep or soft reset: harvest sleep coulombs and raise ADC to 200 Hz!
        int32_t sleep_dq_uah = 0;
        sensor_.set_adc_hz_preserving_coulombs(200, rtc.coulomb, sleep_dq_uah);
        const uint32_t dt = now_s >= rtc.last_step_s ? (now_s - rtc.last_step_s) : 0u;
        const Sample s{
            raw.bat_mv,
            raw.bat_ma,
            sleep_dq_uah,
            static_cast<uint16_t>(dt > 65535u ? 65535u : dt),
            static_cast<uint8_t>(SAMPLE_HAS_DQ |
                                 (raw.charge_done ? SAMPLE_CHARGE_DONE : 0u) |
                                 (raw.charging ? SAMPLE_CHARGING : 0u)),
            0,
        };
        step(rtc.soc, params_, s);
        rtc.last_step_s = now_s;
        rtc.event_dq_accum_uah += sleep_dq_uah;
        push_od_event(EventType::WAKE, now_s, raw);
    } else {
        // Cold boot: reset RTC struct, switch AXP192 to 200 Hz anti-aliasing, seed from history or V/I.
        rtc = {};
        rtc.magic = RTC_FG_MAGIC;
        rtc.events.reset();
        int32_t dummy = 0;
        sensor_.set_adc_hz_preserving_coulombs(200, rtc.coulomb, dummy);

        PackedEvent latest{};
        if (has_flash_history_ && history_.read_latest(latest)) {
            const UnpackedEvent ev = unpack_event(latest);
            if (ev.soc_half_pct <= 200u) {
                seed_q16(rtc.soc, (static_cast<uint32_t>(ev.soc_half_pct) << 16) / 200u, 2600u);
            } else {
                seed_from_vi(rtc.soc, params_, raw.bat_mv, raw.bat_ma);
            }
            // Refine with current voltage immediately.
            const Sample s{raw.bat_mv, raw.bat_ma, 0, 0,
                           static_cast<uint8_t>((raw.charge_done ? SAMPLE_CHARGE_DONE : 0u) |
                                                (raw.charging ? SAMPLE_CHARGING : 0u)),
                           0};
            step(rtc.soc, params_, s);
        } else {
            seed_from_vi(rtc.soc, params_, raw.bat_mv, raw.bat_ma);
        }

        rtc.last_step_s = now_s;
        rtc.last_event_s = now_s;
        rtc.prev_usb_present = raw.usb_present ? 1u : 0u;
        rtc.prev_charging = raw.charging ? 1u : 0u;
        rtc.prev_charge_done = raw.charge_done ? 1u : 0u;
        push_od_event(EventType::BOOT, now_s, raw);
    }
    return true;
}

bool FuelGauge::poll(uint32_t now_s, uint32_t min_period_s, bool force, PollResult& out) noexcept {
    out = {};
    if (!rtc_) return false;
    if (!sensor_.read_telemetry(out.raw)) return false;
    last_raw_ = out.raw;

    const bool usb_changed = (out.raw.usb_present ? 1u : 0u) != rtc_->prev_usb_present;
    const bool chg_changed = (out.raw.charging ? 1u : 0u) != rtc_->prev_charging;
    const bool done_changed = (out.raw.charge_done ? 1u : 0u) != rtc_->prev_charge_done;
    const uint32_t dt = now_s >= rtc_->last_step_s ? (now_s - rtc_->last_step_s) : 0u;

    const bool event_due = usb_changed || chg_changed || done_changed;
    if (!force && !event_due && dt < min_period_s) {
        out.updated = false;
        out.soc_cpct = static_cast<uint16_t>((rtc_->soc.z_q16 * 10000u) >> 16);
        return true;
    }

    int32_t dq_uah = 0;
    const bool has_dq = sensor_.read_coulomb_delta_uah(rtc_->coulomb, dq_uah);
    const uint8_t sflags = static_cast<uint8_t>((has_dq ? SAMPLE_HAS_DQ : 0u) |
                                                (out.raw.charge_done ? SAMPLE_CHARGE_DONE : 0u) |
                                                (out.raw.charging ? SAMPLE_CHARGING : 0u));
    const Sample s{
        out.raw.bat_mv,
        out.raw.bat_ma,
        dq_uah,
        static_cast<uint16_t>(dt > 65535u ? 65535u : dt),
        sflags,
        0,
    };

    out.soc_cpct = step(rtc_->soc, params_, s);
    out.dq_uah = dq_uah;
    out.updated = true;
    rtc_->last_step_s = now_s;
    rtc_->event_dq_accum_uah += dq_uah;

    EventType ev = EventType::PERIODIC;
    if (done_changed && out.raw.charge_done) {
        ev = EventType::CHG_DONE;
        push_od_event(ev, now_s, out.raw);
        // Offline SOH sanity update from `od soc history` if not recently calibrated by ip-soc.
        if (has_flash_history_ && (params_.flags & PARAM_FLAG_CALIBRATED) == 0u) {
            const uint16_t q_est = history_.estimate_offline_qmax_dmah(params_.q_max_dmah);
            if (q_est >= 500u) {
                params_.q_max_dmah = static_cast<uint16_t>((params_.q_max_dmah * 3u + q_est) / 4u);
                sign_params(params_);
                rtc_->params_dirty = 1u;
            }
        }
    } else if (usb_changed) {
        ev = out.raw.usb_present ? EventType::USB_IN : EventType::USB_OUT;
        push_od_event(ev, now_s, out.raw);
    } else if (chg_changed && out.raw.charging) {
        ev = EventType::CHG_START;
        push_od_event(ev, now_s, out.raw);
    } else if (now_s >= rtc_->last_event_s && (now_s - rtc_->last_event_s) >= 300u) {
        // Periodic low-res anchor every 5 min while active.
        push_od_event(EventType::PERIODIC, now_s, out.raw);
    }

    rtc_->prev_usb_present = out.raw.usb_present ? 1u : 0u;
    rtc_->prev_charging = out.raw.charging ? 1u : 0u;
    rtc_->prev_charge_done = out.raw.charge_done ? 1u : 0u;

    out.ip_record = IpRecord{
        now_s * 1000u,
        out.raw.bat_mv,
        out.raw.bat_ma,
        dq_uah,
        out.raw.cell_est_dc,
        static_cast<uint8_t>((static_cast<uint8_t>(ev) & 0x0Fu) | ((sflags & 0x0Fu) << 4)),
        soc_half_percent(rtc_->soc),
    };
    return true;
}

bool FuelGauge::prepare_sleep(uint32_t now_s, bool usb_sleep) noexcept {
    if (!rtc_) return false;
    PollResult pr{};
    poll(now_s, 0, true, pr);
    int32_t tail_uah = 0;
    sensor_.set_adc_hz_preserving_coulombs(25, rtc_->coulomb, tail_uah);
    rtc_->event_dq_accum_uah += tail_uah;
    push_od_event(usb_sleep ? EventType::SLEEP_USB : EventType::SLEEP_WIFI, now_s, last_raw_);
    return true;
}

bool FuelGauge::wake(uint32_t now_s) noexcept {
    if (!rtc_) return false;
    int32_t sleep_dq_uah = 0;
    sensor_.set_adc_hz_preserving_coulombs(200, rtc_->coulomb, sleep_dq_uah);
    RawReading raw{};
    if (!sensor_.read_telemetry(raw)) return false;
    last_raw_ = raw;
    const uint32_t dt = now_s >= rtc_->last_step_s ? (now_s - rtc_->last_step_s) : 0u;
    const Sample s{
        raw.bat_mv,
        raw.bat_ma,
        sleep_dq_uah,
        static_cast<uint16_t>(dt > 65535u ? 65535u : dt),
        static_cast<uint8_t>(SAMPLE_HAS_DQ |
                             (raw.charge_done ? SAMPLE_CHARGE_DONE : 0u) |
                             (raw.charging ? SAMPLE_CHARGING : 0u)),
        0,
    };
    step(rtc_->soc, params_, s);
    rtc_->last_step_s = now_s;
    rtc_->event_dq_accum_uah += sleep_dq_uah;
    rtc_->prev_usb_present = raw.usb_present ? 1u : 0u;
    rtc_->prev_charging = raw.charging ? 1u : 0u;
    rtc_->prev_charge_done = raw.charge_done ? 1u : 0u;
    push_od_event(EventType::WAKE, now_s, raw);
    return true;
}

void FuelGauge::log_event(EventType type, uint32_t now_s) noexcept {
    if (!rtc_) return;
    push_od_event(type, now_s, last_raw_);
}

bool FuelGauge::apply_remote_params(const CellParams& params, uint32_t now_s) noexcept {
    if (!rtc_ || !validate_params(params)) return false;
    params_ = params;
    rtc_->params_dirty = 1u;
    push_od_event(EventType::CAL_APPLIED, now_s, last_raw_);
    return true;
}

bool FuelGauge::flush_history_if_safe() noexcept {
    if (!rtc_ || !has_flash_history_) return false;
    return history_.flush_rtc(rtc_->events);
}

void FuelGauge::push_od_event(EventType type, uint32_t now_s, const RawReading& raw) noexcept {
    const uint32_t dt = now_s >= rtc_->last_event_s ? (now_s - rtc_->last_event_s) : 0u;
    const int32_t qmah = rtc_->event_dq_accum_uah / 250;  // 0.25 mAh steps
    const UnpackedEvent ev{
        dt,
        type,
        raw.bat_mv,
        raw.bat_ma,
        static_cast<int16_t>(qmah < -512 ? -512 : (qmah > 511 ? 511 : qmah)),
        soc_half_percent(rtc_->soc),
        encode_temp_half_c(static_cast<float>(raw.cell_est_dc) * 0.1f),
    };
    rtc_->events.push(pack_event(ev));
    rtc_->last_event_s = now_s;
    rtc_->event_dq_accum_uah = 0;
}

}  // namespace od_soc::m5stickc_plus
