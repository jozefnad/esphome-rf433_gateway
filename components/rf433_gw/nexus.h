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

// ─── Nexus/Solight TE81 Timing (CC1101 OOK reality) ────────────────────────
// CC1101 OOK demodulation stretches Nexus timing due to AGC/threshold drift:
//   Marks (pulses): ~150–750 µs (variable, OOK threshold dependent)
//   Bit 0 spaces:   ~1600–2500 µs (rtl_433 expects ~1000 µs)
//   Bit 1 spaces:   ~3500–4600 µs (rtl_433 expects ~2000 µs)
//   Inter-frame gap: ~6000–9000 µs → matches idle:7ms so each buffer ≈ 1 frame
// With idle:7ms, each frame arrives in its own buffer (72-82 elements).
// Decoder tries multiple offsets from buffer start since frame boundary may
// include trailing elements from previous frame.
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
    if (n < 72) return {};
    ESP_LOGD(TAG_NEXUS, "decode attempt: buffer n=%d", n);

    // With idle:7ms ≈ inter-frame gap, each buffer holds roughly one Nexus frame.
    // A 36-bit PPM frame = 72 elements (mark+space per bit) + possibly a trailing mark.
    // Buffer may start mid-frame if the inter-frame gap was slightly shorter than idle.
    // Try decoding from multiple even offsets to find the real frame start.
    // Also try after any large gap (>5000µs) inside the buffer.

    // Collect candidate start positions
    int starts[20];
    int num_starts = 0;

    // Always try offset 0
    if (src[0] > 0) {
      starts[num_starts++] = 0;
    }

    // Try even offsets 2, 4, 6, 8 (skip partial leading data)
    for (int off = 2; off <= 8 && off <= n - 72; off += 2) {
      if (src[off] > 0) {
        starts[num_starts++] = off;
      }
    }

    // Also try after any large negative gap (inter-frame separator inside buffer)
    for (int i = 0; i < n - 72 && num_starts < 18; i++) {
      if (src[i] < -5000) {
        int next = i + 1;
        if (next <= n - 72 && src[next] > 0) {
          // Avoid duplicates
          bool dup = false;
          for (int j = 0; j < num_starts; j++) {
            if (starts[j] == next) { dup = true; break; }
          }
          if (!dup) starts[num_starts++] = next;
        }
      }
    }

    for (int si = 0; si < num_starts; si++) {
      int idx = starts[si];

      // ── Decode 36 bits (PPM distance coding) ──────────────────────────
      // CC1101 OOK stretched timing:
      //   bit 0: mark ~150-750 µs + space ~1600-2500 µs
      //   bit 1: mark ~150-750 µs + space ~3500-4600 µs
      //   threshold: |space| > 3000 µs → bit 1, else bit 0
      //   gap_limit: |space| > 5000 µs = inter-frame (NOT a data bit)
      uint64_t bits = 0;
      bool frame_ok = true;
      for (int i = 35; i >= 0; i--) {
        if (idx >= n) { frame_ok = false; break; }
        int32_t mark = src[idx++];
        if (mark < 100 || mark > 900) { frame_ok = false; break; }

        if (idx < n && src[idx] < 0) {
          int32_t space = src[idx];
          // gap_limit: reject spaces beyond –5000 µs (inter-frame gap, not data)
          if (space < -5000) { frame_ok = false; break; }
          idx++;
          if (space < -3000) {
            bits |= (1ULL << i);  // long gap → bit 1
          }
          // else: short gap → bit 0 (already 0)
        }
        // No space at end of buffer → bit stays 0 (last bit of last frame)
      }
      if (!frame_ok) {
        ESP_LOGD(TAG_NEXUS, "frame decode failed at start=%d idx=%d of %d", starts[si], idx, n);
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
