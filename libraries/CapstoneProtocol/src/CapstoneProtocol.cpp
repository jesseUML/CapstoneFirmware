// CapstoneProtocol.cpp
//
// Implementation of the explicit byte serialization defined in
// CapstoneProtocol.h. No dynamic allocation, no blocking calls.

#include "CapstoneProtocol.h"

namespace capstone {

namespace {

// Little-endian writers/readers. Keeping these local avoids any dependency on
// board headers so the library stays pure.
inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline uint16_t get_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

// Writes the 5-byte header. Returns pointer just past it.
uint8_t* write_header(uint8_t* p, uint8_t type, uint8_t body_len) {
  put_u16(p, kMagic);
  p[2] = kProtocolVersion;
  p[3] = type;
  p[4] = body_len;
  return p + kHeaderSize;
}

// Validates header + length + CRC. On success, sets body_out/body_len_out to
// point at the body region. Returns false otherwise.
bool validate_frame(const uint8_t* in, size_t in_len, uint8_t expected_type,
                    size_t expected_body_len,
                    const uint8_t** body_out) {
  const size_t expected_frame = kHeaderSize + expected_body_len + kCrcSize;
  if (in == nullptr || in_len < expected_frame) {
    return false;
  }
  if (get_u16(in) != kMagic) {
    return false;
  }
  if (in[2] != kProtocolVersion) {
    return false;
  }
  if (in[3] != expected_type) {
    return false;
  }
  if (in[4] != expected_body_len) {
    return false;
  }
  // CRC covers header + body (everything except the trailing CRC bytes).
  const size_t crc_region = kHeaderSize + expected_body_len;
  const uint16_t computed = crc16(in, crc_region);
  const uint16_t received = get_u16(in + crc_region);
  if (computed != received) {
    return false;
  }
  *body_out = in + kHeaderSize;
  return true;
}

}  // namespace

uint16_t crc16(const uint8_t* data, size_t len) {
  // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

bool seqIsNewer(uint16_t candidate, uint16_t last) {
  // True when candidate is within the "newer" half of the sequence space.
  // Handles wraparound: e.g. last=65534, candidate=1 is newer.
  return static_cast<uint16_t>(candidate - last) != 0 &&
         static_cast<uint16_t>(candidate - last) < 0x8000;
}

uint8_t peekType(const uint8_t* in, size_t in_len) {
  if (in == nullptr || in_len < kHeaderSize) {
    return 0;
  }
  if (get_u16(in) != kMagic) {
    return 0;
  }
  return in[3];
}

size_t packCommand(const CommandPacket& in, uint8_t* out, size_t out_cap) {
  if (out == nullptr || out_cap < kCommandFrameSize) {
    return 0;
  }
  uint8_t* p = write_header(out, kMsgCommand, kCommandBodySize);
  put_u16(p + 0, in.seq);
  p[2] = in.dirs;
  p[3] = in.buttons;
  p[4] = in.status;
  const size_t crc_region = kHeaderSize + kCommandBodySize;
  put_u16(out + crc_region, crc16(out, crc_region));
  return kCommandFrameSize;
}

bool unpackCommand(const uint8_t* in, size_t in_len, CommandPacket& out) {
  const uint8_t* body = nullptr;
  if (!validate_frame(in, in_len, kMsgCommand, kCommandBodySize, &body)) {
    return false;
  }
  out.seq     = get_u16(body + 0);
  out.dirs    = body[2];
  out.buttons = body[3];
  out.status  = body[4];
  return true;
}

size_t packFeedback(const FeedbackPacket& in, uint8_t* out, size_t out_cap) {
  if (out == nullptr || out_cap < kFeedbackFrameSize) {
    return 0;
  }
  uint8_t* p = write_header(out, kMsgFeedback, kFeedbackBodySize);
  put_u16(p + 0, in.seq);
  p[2] = in.eot_state;
  p[3] = in.output_state;
  const size_t crc_region = kHeaderSize + kFeedbackBodySize;
  put_u16(out + crc_region, crc16(out, crc_region));
  return kFeedbackFrameSize;
}

bool unpackFeedback(const uint8_t* in, size_t in_len, FeedbackPacket& out) {
  const uint8_t* body = nullptr;
  if (!validate_frame(in, in_len, kMsgFeedback, kFeedbackBodySize, &body)) {
    return false;
  }
  out.seq          = get_u16(body + 0);
  out.eot_state    = body[2];
  out.output_state = body[3];
  return true;
}

}  // namespace capstone
