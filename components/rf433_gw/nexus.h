#pragma once

#include <cinttypes>
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
    ESP_LOGD(TAG_NEXUS, "decode attempt: buffer n=%d", n);

    // Retry loop: TE81/Nexus sensors send 36 bits 12× per transmission.
    // With idle:7ms, ESPHome captures all frames in one big buffer (~888 items).
    // If the first frame is corrupted (AGC settling, noise), keep scanning for
    // the next valid sync+frame.  Matches rtl_433's repeated-row approach.
    int scan_pos = 0;

    while (scan_pos <= n - 74) {
      // ── Find sync: mark 300–700 µs + space –2500 to –6000 µs ──────────
      int idx = scan_pos;
      bool sync_found = false;
      while (idx <= n - 74) {
        if (src[idx] > 300 && src[idx] < 700 &&
            idx + 1 < n && src[idx + 1] < -2500 && src[idx + 1] > -6000) {
          idx += 2;  // past sync mark + gap
          sync_found = true;
          break;
        }
        idx++;
      }
      if (!sync_found) {
        ESP_LOGD(TAG_NEXUS, "no sync found (scanned %d of %d items)", idx, n);
        return {};
      }

      // Resume scanning from here if this frame fails
      scan_pos = idx;

      // ── Decode 36 bits (PPM distance coding) ──────────────────────────
      // rtl_433 parameters: short_width=1000, long_width=2000, gap_limit=3000
      // bit 0: mark ~500 µs + space ~1000 µs
      // bit 1: mark ~500 µs + space ~2000 µs
      // gap > 3000 µs = sync/reset (NOT a data bit) → abort this frame
      uint64_t bits = 0;
      bool frame_ok = true;
      for (int i = 35; i >= 0; i--) {
        if (idx >= n) { frame_ok = false; break; }
        int32_t mark = src[idx++];
        if (mark < 200 || mark > 800) { frame_ok = false; break; }

        if (idx < n && src[idx] < 0) {
          int32_t space = src[idx];
          // gap_limit: reject spaces beyond –3000 µs (sync/reset gap, not data)
          if (space < -3000) { frame_ok = false; break; }
          idx++;
          if (space < -1500) {
            bits |= (1ULL << i);  // long gap → bit 1
          }
        }
        // No space at end of buffer → bit stays 0 (last bit of last frame)
      }
      if (!frame_ok) {
        ESP_LOGD(TAG_NEXUS, "frame decode failed at idx=%d of %d", idx, n);
        continue;
      }

      // ── Extract fields ─────────────────────────────────────────────────
      uint8_t b_id    = (bits >> 28) & 0xFF;
      uint8_t b_flags = (bits >> 24) & 0x0F;
      uint16_t temp_raw = (bits >> 12) & 0xFFF;
      uint8_t b_const = (bits >> 8) & 0x0F;
      uint8_t b_hum   = ((bits >> 4) & 0x0F) << 4 | (bits & 0x0F);

      // ── Validate (matching rtl_433 nexus_decode) ───────────────────────
      // Constant nibble must be 0xF
      if (b_const != 0x0F) {
        ESP_LOGD(TAG_NEXUS, "const nibble=0x%X (expected 0xF), bits=0x%09" PRIx64, b_const, bits);
        continue;
      }

      // Reject all-zeros or all-ones patterns (rtl_433 false positive check)
      if (b_id == 0 || b_id == 0xFF) {
        ESP_LOGD(TAG_NEXUS, "rejected id=0x%02X (all-zeros or all-ones)", b_id);
        continue;
      }

      NexusData data;
      data.id         = b_id;
      data.battery_ok = (b_flags >> 3) & 1;
      data.test_mode  = (b_flags >> 2) & 1;
      data.channel    = (b_flags & 0x03) + 1;  // 0→CH1, 1→CH2, 2→CH3

      // Temperature: 12-bit signed, divided by 10
      int16_t temp_signed = (int16_t)(temp_raw << 4) >> 4;
      data.temperature = temp_signed / 10.0f;

      // Humidity
      data.humidity = b_hum;

      // Reject unrealistic sensor values
      if (data.temperature < -40.0f || data.temperature > 80.0f) continue;
      if (data.humidity > 100) continue;

      return data;
    }
    return {};
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
