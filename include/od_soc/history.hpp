#pragma once

// `od soc history`: Hypercompressed always-on 8-byte (64-bit) battery event ring.
// Buffered in RTC RAM (128 B = 16 events) and appended to flash ONLY by `od soc code`
// when disarmed (zero flash wear during active operation, ~1 sector erase/month).

#include <cstddef>
#include <cstdint>

namespace od_soc {

enum class EventType : uint8_t {
    PERIODIC    = 0,
    BOOT        = 1,
    SHUTDOWN    = 2,
    USB_IN      = 3,
    USB_OUT     = 4,
    CHG_START   = 5,
    CHG_DONE    = 6,
    SLEEP_WIFI  = 7,
    SLEEP_USB   = 8,
    WAKE        = 9,
    ARM         = 10,
    DISARM      = 11,
    LOW_BAT     = 12,
    CAL_APPLIED = 13,
    THERMAL     = 14,
    FAULT       = 15,
};

struct UnpackedEvent {
    uint32_t dt_s;          // Elapsed seconds since previous event (log-quantized)
    EventType type;
    uint16_t v_mv;          // 2500..4547 mV (1 mV res)
    int16_t i_ma;           // -1024..+1023 mA (1 mA res)
    int16_t dq_qmah;        // -512..+511 in 0.25 mAh steps (+-127.75 mAh)
    uint8_t soc_half_pct;   // 0..200 (0.5% steps), 255 = unknown
    uint8_t temp_half_c;    // (temp_c + 20) * 2 (0.5 C steps), 255 = unknown
};

// Exact 8-byte (64-bit little-endian) packed event record shared bit-for-bit with ip-soc.
struct PackedEvent {
    uint64_t raw;
};
static_assert(sizeof(PackedEvent) == 8, "PackedEvent must be exactly 8 bytes");

// Log-quantized 12-bit delta-time codec: 1 s..18.7 days.
uint16_t encode_dt_log(uint32_t dt_s) noexcept;
uint32_t decode_dt_log(uint16_t code) noexcept;

// Pack/unpack 8-byte event record.
PackedEvent pack_event(const UnpackedEvent& ev) noexcept;
UnpackedEvent unpack_event(PackedEvent pe) noexcept;

inline uint8_t encode_temp_half_c(float temp_c) noexcept {
    const int32_t v = static_cast<int32_t>((temp_c + 20.0f) * 2.0f + (temp_c >= -20.0f ? 0.5f : -0.5f));
    if (v < 0) return 0;
    if (v > 254) return 254;
    return static_cast<uint8_t>(v);
}

inline float decode_temp_half_c(uint8_t code) noexcept {
    return static_cast<float>(code) * 0.5f - 20.0f;
}

// 136-byte RTC-RAM write-coalescing ring (16 x 8 B events + 8 B header).
// Drops directly into RTC_DATA_ATTR; zero flash writes until flushed while disarmed.
inline constexpr uint32_t RTC_RING_MAGIC = 0x4F444831u;  // "ODH1"
inline constexpr size_t RTC_RING_CAPACITY = 16;

struct RtcEventRing {
    uint32_t magic;
    uint16_t count;      // Valid buffered events waiting for flash flush (0..16)
    uint16_t total_seq;  // Wrapping sequence counter
    PackedEvent entries[RTC_RING_CAPACITY];

    void reset() noexcept;
    bool valid() const noexcept { return magic == RTC_RING_MAGIC && count <= RTC_RING_CAPACITY; }
    bool full() const noexcept { return count >= RTC_RING_CAPACITY; }
    void push(PackedEvent ev) noexcept;
};
static_assert(sizeof(RtcEventRing) == 136, "RtcEventRing size mismatch");

// Pluggable flash storage backend for append-only `od soc history` (no ESP-IDF coupling).
// Erased flash reads as 0xFF (raw == 0xFFFFFFFFFFFFFFFFULL).
struct FlashOps {
    bool (*read)(void* ctx, uint32_t offset, void* out, size_t len) noexcept;
    bool (*write)(void* ctx, uint32_t offset, const void* data, size_t len) noexcept;
    bool (*erase_sector)(void* ctx, uint32_t sector_offset) noexcept;
    void* ctx;
    uint32_t total_bytes;   // Must be a multiple of sector_bytes (e.g. 4096 or 8192)
    uint32_t sector_bytes;  // Typically 4096
};

// Append-only flash ring over 1 or 2 flash sectors (512 or 1024 events).
class FlashHistoryRing {
  public:
    bool attach(const FlashOps& ops) noexcept;

    // Appends buffered RTC events to flash (call ONLY when safe/disarmed!).
    // Clears rtc.count on success.
    bool flush_rtc(RtcEventRing& rtc) noexcept;

    // Reads the most recent valid event from flash (for cold-boot recovery after power loss).
    bool read_latest(PackedEvent& out) const noexcept;

    // Reads up to max_out events in chronological order (oldest -> newest) for ip-soc gap sync.
    size_t read_chronological(PackedEvent* out, size_t max_out) const noexcept;

    // Lightweight offline SOH sanity estimate (returns updated q_max_dmah, or 0 if insufficient data).
    // Scans recent clean USB_OUT -> CHG_DONE cycles inside the 8-byte ring.
    uint16_t estimate_offline_qmax_dmah(uint16_t nominal_q_max_dmah) const noexcept;

    uint32_t write_slot() const noexcept { return write_slot_; }
    uint32_t capacity_slots() const noexcept { return capacity_slots_; }

  private:
    FlashOps ops_{};
    uint32_t capacity_slots_ = 0;
    uint32_t write_slot_ = 0;
    bool attached_ = false;
};

}  // namespace od_soc
