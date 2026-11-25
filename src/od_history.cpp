#include "od_soc/history.hpp"

namespace od_soc {

namespace {

inline constexpr uint64_t ERASED_SLOT = 0xFFFFFFFFFFFFFFFFULL;

inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi) noexcept {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

}  // namespace

uint16_t encode_dt_log(uint32_t dt_s) noexcept {
    if (dt_s < 60u) {
        return static_cast<uint16_t>(dt_s);  // 0..59 (1 s res)
    }
    if (dt_s < 600u) {
        return static_cast<uint16_t>(60u + (dt_s - 60u) / 10u);  // 60..113 (10 s res)
    }
    if (dt_s < 86640u) {
        return static_cast<uint16_t>(114u + (dt_s - 600u) / 60u);  // 114..1547 (60 s res)
    }
    const uint32_t c = 1548u + (dt_s - 86640u) / 600u;  // 1548..4095 (600 s res, up to 18.7 days)
    return c > 4095u ? 4095u : static_cast<uint16_t>(c);
}

uint32_t decode_dt_log(uint16_t code) noexcept {
    code &= 0x0FFFu;
    if (code < 60u) return code;
    if (code < 114u) return 60u + static_cast<uint32_t>(code - 60u) * 10u;
    if (code < 1548u) return 600u + static_cast<uint32_t>(code - 114u) * 60u;
    return 86640u + static_cast<uint32_t>(code - 1548u) * 600u;
}

PackedEvent pack_event(const UnpackedEvent& ev) noexcept {
    const uint64_t dt_bits = encode_dt_log(ev.dt_s) & 0x0FFFu;
    const uint64_t ev_bits = static_cast<uint64_t>(ev.type) & 0x0Fu;
    const uint64_t v_bits = static_cast<uint64_t>(clamp_i32(static_cast<int32_t>(ev.v_mv) - 2500, 0, 2047)) & 0x7FFu;
    const uint64_t i_bits = static_cast<uint64_t>(clamp_i32(ev.i_ma, -1024, 1023)) & 0x7FFu;
    const uint64_t dq_bits = static_cast<uint64_t>(clamp_i32(ev.dq_qmah, -512, 511)) & 0x3FFu;
    const uint64_t soc_bits = ev.soc_half_pct;
    const uint64_t t_bits = ev.temp_half_c;

    uint64_t raw = dt_bits
                 | (ev_bits << 12)
                 | (v_bits << 16)
                 | (i_bits << 27)
                 | (dq_bits << 38)
                 | (soc_bits << 48)
                 | (t_bits << 56);
    // Avoid colliding with erased flash sentinel (0xFFFFFFFFFFFFFFFFULL).
    if (raw == ERASED_SLOT) {
        raw ^= (1ULL << 56);
    }
    return {raw};
}

UnpackedEvent unpack_event(PackedEvent pe) noexcept {
    const uint64_t r = pe.raw;
    const uint16_t dt_code = static_cast<uint16_t>(r & 0x0FFFu);
    const auto type = static_cast<EventType>((r >> 12) & 0x0Fu);
    const uint16_t v_mv = static_cast<uint16_t>(2500u + ((r >> 16) & 0x7FFu));

    int32_t i_signed = static_cast<int32_t>((r >> 27) & 0x7FFu);
    if ((i_signed & 0x400) != 0) i_signed |= ~0x7FF;

    int32_t dq_signed = static_cast<int32_t>((r >> 38) & 0x3FFu);
    if ((dq_signed & 0x200) != 0) dq_signed |= ~0x3FF;

    return UnpackedEvent{
        decode_dt_log(dt_code),
        type,
        v_mv,
        static_cast<int16_t>(i_signed),
        static_cast<int16_t>(dq_signed),
        static_cast<uint8_t>((r >> 48) & 0xFFu),
        static_cast<uint8_t>((r >> 56) & 0xFFu),
    };
}

void RtcEventRing::reset() noexcept {
    magic = RTC_RING_MAGIC;
    count = 0;
    total_seq = 0;
}

void RtcEventRing::push(PackedEvent ev) noexcept {
    if (!valid()) {
        reset();
    }
    if (count < RTC_RING_CAPACITY) {
        entries[count++] = ev;
    } else {
        // Ring full before flash flush: shift out oldest so newest lifecycle events survive.
        for (size_t i = 1; i < RTC_RING_CAPACITY; ++i) {
            entries[i - 1] = entries[i];
        }
        entries[RTC_RING_CAPACITY - 1] = ev;
    }
    ++total_seq;
}

bool FlashHistoryRing::attach(const FlashOps& ops) noexcept {
    if (!ops.read || !ops.write || !ops.erase_sector || ops.sector_bytes < sizeof(PackedEvent) ||
        ops.total_bytes < ops.sector_bytes || (ops.total_bytes % ops.sector_bytes) != 0) {
        attached_ = false;
        return false;
    }
    ops_ = ops;
    capacity_slots_ = ops.total_bytes / sizeof(PackedEvent);
    attached_ = true;

    // Locate first erased slot (boundary between newest written record and erased space).
    uint32_t first_erased = capacity_slots_;
    for (uint32_t i = 0; i < capacity_slots_; ++i) {
        PackedEvent pe{};
        if (!ops_.read(ops_.ctx, i * sizeof(PackedEvent), &pe, sizeof(pe))) {
            attached_ = false;
            return false;
        }
        if (pe.raw == ERASED_SLOT) {
            first_erased = i;
            break;
        }
    }

    if (first_erased == capacity_slots_) {
        // All sectors full (possible if power lost mid-wrap): wrap to slot 0 and erase sector 0 on next flush.
        write_slot_ = 0;
    } else {
        // If multi-sector and slot 0 is erased while later sector has data, find the transition.
        write_slot_ = first_erased;
        if (first_erased == 0) {
            for (uint32_t i = capacity_slots_; i > 0; --i) {
                PackedEvent pe{};
                if (ops_.read(ops_.ctx, (i - 1) * sizeof(PackedEvent), &pe, sizeof(pe)) &&
                    pe.raw != ERASED_SLOT) {
                    write_slot_ = i % capacity_slots_;
                    break;
                }
            }
        }
    }
    return true;
}

bool FlashHistoryRing::flush_rtc(RtcEventRing& rtc) noexcept {
    if (!attached_ || !rtc.valid() || rtc.count == 0) {
        return attached_;
    }
    const uint32_t slots_per_sector = ops_.sector_bytes / sizeof(PackedEvent);
    for (uint16_t i = 0; i < rtc.count; ++i) {
        // When landing on a sector boundary, check if erasing is required.
        if ((write_slot_ % slots_per_sector) == 0u) {
            PackedEvent probe{};
            const uint32_t sec_off = write_slot_ * sizeof(PackedEvent);
            if (!ops_.read(ops_.ctx, sec_off, &probe, sizeof(probe))) {
                return false;
            }
            if (probe.raw != ERASED_SLOT) {
                if (!ops_.erase_sector(ops_.ctx, sec_off)) {
                    return false;
                }
            }
        }
        const uint32_t off = write_slot_ * sizeof(PackedEvent);
        if (!ops_.write(ops_.ctx, off, &rtc.entries[i], sizeof(PackedEvent))) {
            return false;
        }
        write_slot_ = (write_slot_ + 1u) % capacity_slots_;
    }
    rtc.count = 0;
    return true;
}

bool FlashHistoryRing::read_latest(PackedEvent& out) const noexcept {
    if (!attached_ || capacity_slots_ == 0) return false;
    const uint32_t prev = (write_slot_ + capacity_slots_ - 1u) % capacity_slots_;
    PackedEvent pe{};
    if (!ops_.read(ops_.ctx, prev * sizeof(PackedEvent), &pe, sizeof(pe))) return false;
    if (pe.raw == ERASED_SLOT) return false;
    out = pe;
    return true;
}

size_t FlashHistoryRing::read_chronological(PackedEvent* out, size_t max_out) const noexcept {
    if (!attached_ || !out || max_out == 0) return 0;
    size_t n = 0;
    // Scan from write_slot_ forward (oldest valid sector first), wrapping to write_slot_ - 1 (newest).
    for (uint32_t step = 0; step < capacity_slots_ && n < max_out; ++step) {
        const uint32_t slot = (write_slot_ + step) % capacity_slots_;
        PackedEvent pe{};
        if (!ops_.read(ops_.ctx, slot * sizeof(PackedEvent), &pe, sizeof(pe))) break;
        if (pe.raw != ERASED_SLOT) {
            out[n++] = pe;
        }
    }
    return n;
}

uint16_t FlashHistoryRing::estimate_offline_qmax_dmah(uint16_t nominal_q_max_dmah) const noexcept {
    if (!attached_ || nominal_q_max_dmah < 10) return 0;
    // Scan chronologically for clean discharge/charge arcs ending in CHG_DONE with >= 35% SOC swing.
    int32_t accum_qmah = 0;
    uint8_t start_soc_hp = 255;
    int32_t est_sum = 0;
    int32_t est_cnt = 0;

    for (uint32_t step = 0; step < capacity_slots_; ++step) {
        const uint32_t slot = (write_slot_ + step) % capacity_slots_;
        PackedEvent pe{};
        if (!ops_.read(ops_.ctx, slot * sizeof(PackedEvent), &pe, sizeof(pe))) break;
        if (pe.raw == ERASED_SLOT) continue;
        const UnpackedEvent ev = unpack_event(pe);
        if (ev.type == EventType::CHG_START && ev.soc_half_pct <= 130u) {  // <= 65% SOC
            start_soc_hp = ev.soc_half_pct;
            accum_qmah = 0;
        } else if (start_soc_hp <= 130u) {
            accum_qmah += ev.dq_qmah;
            if (ev.type == EventType::CHG_DONE) {
                const int32_t dsoc_hp = 200 - static_cast<int32_t>(start_soc_hp);
                if (dsoc_hp >= 70 && accum_qmah > 0) {
                    // accum_qmah is in 0.25 mAh; q_dmah = (accum_qmah * 2.5) / (dsoc_hp / 200)
                    //                                    = (accum_qmah * 500) / dsoc_hp
                    const int32_t q_obs = (accum_qmah * 500) / dsoc_hp;
                    const int32_t lo = (static_cast<int32_t>(nominal_q_max_dmah) * 50) / 100;
                    const int32_t hi = (static_cast<int32_t>(nominal_q_max_dmah) * 115) / 100;
                    if (q_obs >= lo && q_obs <= hi) {
                        est_sum += q_obs;
                        ++est_cnt;
                    }
                }
                start_soc_hp = 255;
                accum_qmah = 0;
            }
        }
    }
    return est_cnt > 0 ? static_cast<uint16_t>(est_sum / est_cnt) : 0u;
}

}  // namespace od_soc
