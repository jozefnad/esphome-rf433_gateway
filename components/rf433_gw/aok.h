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
// TX timing — calibrated to match the real A-OK AC114/AC226 remotes
static const uint32_t AOK_TX_SYNC_HIGH_US = 5300;  // AGC mark
static const uint32_t AOK_TX_SYNC_LOW_US  = 530;   // AGC space
static const uint32_t AOK_TX_ONE_HIGH_US  = 565;   // bit 1 mark
static const uint32_t AOK_TX_ONE_LOW_US   = 270;   // bit 1 space
static const uint32_t AOK_TX_ZERO_HIGH_US = 270;   // bit 0 mark
static const uint32_t AOK_TX_ZERO_LOW_US  = 565;   // bit 0 space
static const uint32_t AOK_TX_TRAILING_US  = 5030;  // radio silence after frame

// ─── A-OK Protocol (encoder + decoder) — header-only ───────────────────────
// TX: single frame [SYNC 5300/530µs][64 data bits][trailing '1'][silence 5030µs]
// Use repeat: times: 3 in YAML for multiple transmissions (like the real remote).
// The encode() timing is calibrated to match the real AC114/AC226 remotes.

class AOKProtocol : public remote_base::RemoteProtocol<AOKData> {
 public:
  void encode(remote_base::RemoteTransmitData *dst, const AOKData &data) override {
    uint64_t bits = data.to_uint64();
    // Single frame: sync + 64 data bits + trailing bit + silence = ~135 items
    dst->reserve(135);
    dst->set_carrier_frequency(0);  // OOK — no carrier

    // AGC / Sync pulse
    dst->item(AOK_TX_SYNC_HIGH_US, AOK_TX_SYNC_LOW_US);

    // 64 data bits, MSB first
    for (int i = 63; i >= 0; i--) {
      if ((bits >> i) & 1u) {
        dst->item(AOK_TX_ONE_HIGH_US, AOK_TX_ONE_LOW_US);
      } else {
        dst->item(AOK_TX_ZERO_HIGH_US, AOK_TX_ZERO_LOW_US);
      }
    }

    // Trailing bit (always 1)
    dst->mark(AOK_TX_ONE_HIGH_US);
    dst->space(AOK_TX_ONE_LOW_US);

    // Radio silence
    dst->space(AOK_TX_TRAILING_US);
  }

  optional<AOKData> decode(remote_base::RemoteReceiveData src) override {
    const int32_t n = src.size();
    if (n < 130) return {};

    // Linear scan for AGC/sync mark (4500–6000 µs)
    int idx = 0;
    while (idx < n - 130) {
      if (src[idx] > 4500 && src[idx] < 6000) break;
      idx++;
    }
    if (idx >= n - 130) return {};
    idx++;  // past sync mark

    // Sync space: ~530 µs, accept –300 to –800
    if (idx >= n || src[idx] > -300 || src[idx] < -800) return {};
    idx++;  // past sync space

    // 64 data bits — bit value from mark duration only:
    //   mark > 400 µs → bit 1 (long mark ~565 µs)
    //   mark ≤ 400 µs → bit 0 (short mark ~270 µs)
    uint64_t bits = 0;
    for (int bit = 0; bit < 64; bit++) {
      if (idx >= n) return {};
      int32_t mark = src[idx++];
      if (mark < 150 || mark > 900) return {};
      bits = (bits << 1) | (mark > 400 ? 1 : 0);
      if (idx < n && src[idx] < 0) idx++;  // skip space
    }

    // Validate header byte (0xA3)
    if (((bits >> 56) & 0xFF) != 0xA3) return {};

    // Unpack fields
    AOKData data;
    data.remote_id = (bits >> 32) & 0x00FFFFFF;
    data.address   = (bits >> 16) & 0xFFFF;
    data.command   = (bits >>  8) & 0xFF;
    uint8_t rx_crc =  bits        & 0xFF;

    if (rx_crc != data.checksum()) {
      ESP_LOGW(TAG_AOK, "Checksum mismatch rx=0x%02X calc=0x%02X", rx_crc, data.checksum());
      return {};
    }

    return data;
  }

  void dump(const AOKData &data) override {
    const char *cmd_str = "UNKNOWN";
    switch (data.command) {
      case AOK_CMD_UP:        cmd_str = "UP";        break;
      case AOK_CMD_STOP:      cmd_str = "STOP";      break;
      case AOK_CMD_DOWN:      cmd_str = "DOWN";      break;
      case AOK_CMD_PROGRAM:   cmd_str = "PROGRAM";   break;
      case AOK_CMD_LIGHT_ON:  cmd_str = "LIGHT_ON";  break;
      case AOK_CMD_LIGHT_OFF: cmd_str = "LIGHT_OFF"; break;
    }
    ESP_LOGI(TAG_AOK, "Received A-OK: remote_id=0x%06X address=0x%04X command=%s (0x%02X)",
             data.remote_id, data.address, cmd_str, data.command);
  }

};

}  // namespace rf433_gw
}  // namespace esphome
