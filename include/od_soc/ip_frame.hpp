#pragma once

// `ip soc history` wire frame codec ("SOC1"):
// Compact binary frames streamed over IP (HTTP/WS/ESP-NOW/spillover) to esp32-tools/ip-soc.
// Header (12 B): "SOC1" (4B) | seq (u32 LE) | count (u16 LE) | flags (u8) | reserved (u8)
// Body: `count` x 16-byte IpRecord structs, followed by CRC-16/CCITT-FALSE (2 B LE).

#include <cstddef>
#include <cstdint>

namespace od_soc {

inline constexpr uint8_t IP_FRAME_MAGIC[4] = {'S', 'O', 'C', '1'};
inline constexpr size_t IP_FRAME_HEADER_BYTES = 12;
inline constexpr size_t IP_FRAME_TRAILER_BYTES = 2;

// High-resolution battery record for `ip soc history` (exact 16 bytes, little-endian).
struct IpRecord {
    uint32_t ts_ms;        // Monotonic device timestamp (ms)
    uint16_t v_mv;         // Battery terminal voltage (mV)
    int16_t i_ma;          // Instantaneous current (mA, + charge, - discharge)
    int32_t dq_uah;        // Hardware coulomb delta since previous record (uAh)
    int16_t temp_dc;       // Estimated cell temperature in 0.1 C (e.g. 285 = 28.5 C)
    uint8_t event_flags;   // Low nibble: EventType; high nibble: SAMPLE_* flags
    uint8_t od_soc_hp;     // On-device estimated SOC in 0.5% steps (0..200)
};
static_assert(sizeof(IpRecord) == 16, "IpRecord must be exactly 16 bytes");

size_t encode_ip_frame(uint32_t seq, uint8_t frame_flags, const IpRecord* records, size_t count,
                       uint8_t* out, size_t max_out) noexcept;

size_t decode_ip_frame(const uint8_t* data, size_t len, uint32_t& out_seq, uint8_t& out_flags,
                       IpRecord* out_records, size_t max_records) noexcept;

}  // namespace od_soc
