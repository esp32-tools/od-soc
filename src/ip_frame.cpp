#include "od_soc/ip_frame.hpp"
#include "od_soc/od_soc.hpp"

#include <cstring>

namespace od_soc {

namespace {

inline void put_u16(uint8_t* p, uint16_t v) noexcept {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline void put_u32(uint8_t* p, uint32_t v) noexcept {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

inline uint16_t get_u16(const uint8_t* p) noexcept {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t get_u32(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

size_t encode_ip_frame(uint32_t seq, uint8_t frame_flags, const IpRecord* records, size_t count,
                       uint8_t* out, size_t max_out) noexcept {
    if (!out || (!records && count > 0) || count > 0xFFFFu) return 0;
    const size_t body = count * sizeof(IpRecord);
    const size_t total = IP_FRAME_HEADER_BYTES + body + IP_FRAME_TRAILER_BYTES;
    if (max_out < total) return 0;

    std::memcpy(out, IP_FRAME_MAGIC, 4);
    put_u32(out + 4, seq);
    put_u16(out + 8, static_cast<uint16_t>(count));
    out[10] = frame_flags;
    out[11] = 0;

    uint8_t* w = out + IP_FRAME_HEADER_BYTES;
    for (size_t i = 0; i < count; ++i) {
        put_u32(w + 0, records[i].ts_ms);
        put_u16(w + 4, records[i].v_mv);
        put_u16(w + 6, static_cast<uint16_t>(records[i].i_ma));
        put_u32(w + 8, static_cast<uint32_t>(records[i].dq_uah));
        put_u16(w + 12, static_cast<uint16_t>(records[i].temp_dc));
        w[14] = records[i].event_flags;
        w[15] = records[i].od_soc_hp;
        w += sizeof(IpRecord);
    }

    const uint16_t crc = crc16_ccitt(out, IP_FRAME_HEADER_BYTES + body);
    put_u16(out + IP_FRAME_HEADER_BYTES + body, crc);
    return total;
}

size_t decode_ip_frame(const uint8_t* data, size_t len, uint32_t& out_seq, uint8_t& out_flags,
                       IpRecord* out_records, size_t max_records) noexcept {
    if (!data || len < IP_FRAME_HEADER_BYTES + IP_FRAME_TRAILER_BYTES) return 0;
    if (std::memcmp(data, IP_FRAME_MAGIC, 4) != 0) return 0;
    const uint16_t count = get_u16(data + 8);
    const size_t expected = IP_FRAME_HEADER_BYTES + static_cast<size_t>(count) * sizeof(IpRecord) + IP_FRAME_TRAILER_BYTES;
    if (len < expected || count > max_records) return 0;

    const uint16_t crc = crc16_ccitt(data, expected - IP_FRAME_TRAILER_BYTES);
    if (crc != get_u16(data + expected - IP_FRAME_TRAILER_BYTES)) return 0;

    out_seq = get_u32(data + 4);
    out_flags = data[10];

    const uint8_t* r = data + IP_FRAME_HEADER_BYTES;
    for (size_t i = 0; i < count; ++i) {
        out_records[i].ts_ms = get_u32(r + 0);
        out_records[i].v_mv = get_u16(r + 4);
        out_records[i].i_ma = static_cast<int16_t>(get_u16(r + 6));
        out_records[i].dq_uah = static_cast<int32_t>(get_u32(r + 8));
        out_records[i].temp_dc = static_cast<int16_t>(get_u16(r + 12));
        out_records[i].event_flags = r[14];
        out_records[i].od_soc_hp = r[15];
        r += sizeof(IpRecord);
    }
    return count;
}

}  // namespace od_soc
