#pragma once

#include "esphome/core/log.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome {
namespace rf433_gw {

static const char *const TAG_NEXUS = "rf433_gw.nexus";

// ─── Nexus Packet Data ─────────────────────────────────────────────────────
struct NexusData {
  uint8_t  id{0};           // 8-bit device ID
  uint8_t  channel{0};      // 2-bit channel (1-4)
  bool     battery_ok{true};
  bool     test_mode{false};
  float    temperature{0};  // °C
  uint8_t  humidity{0};     // % (0 = no humidity sensor)

  bool operator==(const NexusData &rhs) const {
    return id == rhs.id && channel == rhs.channel;
  }
};

// ─── Nexus/Solight TE81 Timing ─────────────────────────────────────────────
// PPM modulation: ~500 µs pulse, short gap ~1000 µs (bit 0), long gap ~2000 µs (bit 1)
// Sync: ~500 µs pulse + ~4000 µs gap
// Decoder uses direct raw-array access (no expect_mark / expect_space) for speed.

// ─── Nexus Protocol (RX-only decoder) — header-only ────────────────────────
// Protocol: 36 bits, PPM (distance coding)
// Data: [id:8][flags:4][temp:12][0xF:4][humidity:8]
// Flags: B T CC (Battery, Test, Channel)
// Temperature: 12-bit signed, scaled by 10 (i.e. value/10 = °C)
// Humidity: 0 = no sensor
class NexusProtocol {
 public:
  optional<NexusData> decode(remote_base::RemoteReceiveData src) {
    const int32_t n = src.size();
    if (n < 74) return {};

    // Linear scan for sync: mark 300–700 µs + space –2500 to –6000
    int idx = 0;
    bool sync_found = false;
    while (idx < n - 73) {
      if (src[idx] > 300 && src[idx] < 700 &&
          idx + 1 < n && src[idx + 1] < -2500 && src[idx + 1] > -6000) {
        idx += 2;  // past sync mark + space
        sync_found = true;
        break;
      }
      idx++;
    }
    if (!sync_found) return {};

    // 36 bits — PPM distance coding:
    //   mark ~500 µs + space ~1000 µs = bit 0
    //   mark ~500 µs + space ~2000 µs = bit 1
    //   threshold: |space| > 1500 → bit 1
    uint64_t bits = 0;
    for (int i = 35; i >= 0; i--) {
      if (idx >= n) return {};
      int32_t mark = src[idx++];
      if (mark < 300 || mark > 700) return {};

      if (idx < n && src[idx] < 0) {
        int32_t space = src[idx++];
        if (space < -1500) {
          bits |= (1ULL << i);  // long gap → bit 1
        }
      }
      // No space (last bit or end of buffer) → bit stays 0
    }

    // Extract fields
    uint8_t b[5];
    b[0] = (bits >> 28) & 0xFF;  // id
    b[1] = (bits >> 24) & 0x0F;  // flags
    uint16_t temp_raw = (bits >> 12) & 0xFFF;
    b[2] = (bits >> 8) & 0x0F;   // const nibble (should be 0xF)
    b[3] = (bits >> 4) & 0x0F;   // humidity high nibble
    b[4] = bits & 0x0F;          // humidity low nibble

    // Validate constant nibble = 0xF
    if (b[2] != 0x0F) {
      ESP_LOGV(TAG_NEXUS, "Const nibble mismatch: 0x%X (expected 0xF)", b[2]);
      return {};
    }
    // Reduce false positives: ID must not be 0 or 0xFF
    if (b[0] == 0 || b[0] == 0xFF) return {};

    NexusData data;
    data.id         = b[0];
    data.battery_ok = (b[1] >> 3) & 1;
    data.test_mode  = (b[1] >> 2) & 1;
    data.channel    = (b[1] & 0x03) + 1;  // channel 1-4

    // Temperature: 12-bit signed, divided by 10
    int16_t temp_signed = (int16_t)(temp_raw << 4) >> 4;
    data.temperature = temp_signed / 10.0f;

    // Humidity
    data.humidity = (b[3] << 4) | b[4];

    // Reject unrealistic sensor values (false decode protection)
    if (data.temperature < -40.0f || data.temperature > 80.0f) return {};
    if (data.humidity > 100) return {};

    return data;
  }

  void dump(const NexusData &data) {
    if (data.humidity > 0) {
      ESP_LOGI(TAG_NEXUS, "Received Nexus-TH: id=%d ch=%d battery=%s temp=%.1f°C humidity=%d%%",
               data.id, data.channel,
               data.battery_ok ? "OK" : "LOW",
               data.temperature, data.humidity);
    } else {
      ESP_LOGI(TAG_NEXUS, "Received Nexus-T: id=%d ch=%d battery=%s temp=%.1f°C",
               data.id, data.channel,
               data.battery_ok ? "OK" : "LOW",
               data.temperature);
    }
  }
};

}  // namespace rf433_gw
}  // namespace esphome
