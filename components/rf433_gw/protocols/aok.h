#pragma once

#include "esphome/core/log.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome {
namespace rf433_gw {

static const char *const TAG_AOK = "rf433_gw.aok";

// ─── A-OK Command Constants ─────────────────────────────────────────────────
enum AOKCommand : uint8_t {
  AOK_CMD_UP        = 0x0B,
  AOK_CMD_STOP      = 0x23,
  AOK_CMD_DOWN      = 0x43,
  AOK_CMD_PROGRAM   = 0x53,
  AOK_CMD_LIGHT_ON  = 0x12,
  AOK_CMD_LIGHT_OFF = 0x13,
};

// ─── A-OK Packet Data ───────────────────────────────────────────────────────
struct AOKData {
  uint32_t remote_id{0};   // 24 bits
  uint16_t address{0};     // 16 bits (channel bitmask)
  uint8_t  command{0};     //  8 bits

  uint8_t checksum() const {
    uint8_t s = 0;
    s += (remote_id >> 16) & 0xFF;
    s += (remote_id >>  8) & 0xFF;
    s += (remote_id)       & 0xFF;
    s += (address   >>  8) & 0xFF;
    s += (address)         & 0xFF;
    s += command;
    return s;
  }

  uint64_t to_uint64() const {
    uint64_t v = 0;
    v |= (uint64_t) 0xA3                       << 56;
    v |= (uint64_t)(remote_id & 0x00FFFFFF)     << 32;
    v |= (uint64_t) address                     << 16;
    v |= (uint64_t) command                     <<  8;
    v |= (uint64_t) checksum();
    return v;
  }

  bool operator==(const AOKData &rhs) const {
    return remote_id == rhs.remote_id &&
           address   == rhs.address   &&
           command   == rhs.command;
  }
};

// ─── Timing constants (µs) ─────────────────────────────────────────────────
static const uint32_t AOK_BASE_US         = 300;
static const uint32_t AOK_SYNC_HIGH_US    = AOK_BASE_US * 17;  // 5100 µs
static const uint32_t AOK_SYNC_LOW_US     = AOK_BASE_US * 2;   //  600 µs
static const uint32_t AOK_ZERO_HIGH_US    = AOK_BASE_US * 1;   //  300 µs
static const uint32_t AOK_ZERO_LOW_US     = AOK_BASE_US * 2;   //  600 µs
static const uint32_t AOK_ONE_HIGH_US     = AOK_BASE_US * 2;   //  600 µs
static const uint32_t AOK_ONE_LOW_US      = AOK_BASE_US * 1;   //  300 µs
static const uint8_t  AOK_PREAMBLE_BITS   = 7;
static const uint32_t AOK_INTER_FRAME_US  = AOK_BASE_US * 17;  // 5100 µs

// ─── A-OK Protocol (encoder + decoder) ─────────────────────────────────────
class AOKProtocol : public remote_base::RemoteProtocol<AOKData> {
 public:
  void encode(remote_base::RemoteTransmitData *dst, const AOKData &data) override;
  optional<AOKData> decode(remote_base::RemoteReceiveData src) override;
  void dump(const AOKData &data) override;
};

}  // namespace rf433_gw
}  // namespace esphome
