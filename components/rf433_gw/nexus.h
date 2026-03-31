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

// ─── Solight TE81 / Nexus-TH Timing (CC1101 OOK reality) ───────────────────
// Buffer structure from CC1101 OOK:
//   [preamble: 3-4 pairs ~1000µs mark/~1000µs space]
//   [sync mark ~500-700µs] + [sync gap ~7500-8000µs]
//   [40 data bits: mark ~500-670µs + space ~1800µs(0) / ~4000µs(1)]
//   [trailing mark]
// Total buffer: ~90-92 elements for a complete frame.
//
// Data format (40 bits, MSB first):
//   [id:8][flags:4][seq:4][temp:12][humidity_BCD:8][check:4]
//   - id: random on battery change
//   - flags: channel/battery/mode bits (variable between TXes)
//   - seq: sequence/rolling nibble (not data, changes each TX burst)
//   - temp: 12-bit raw ADC value, formula: (raw - 1221) / 18.0 = °C
//   - humidity: BCD encoded (0x72 = 72%)
//   - check: checksum or padding (always 0x1 observed)
// Decoder uses direct raw-array access for speed on single-core ESP32-C3.

// ─── TE81 Protocol (RX-only decoder) — header-only ─────────────────────────
class NexusProtocol {
 public:
  optional<NexusData> decode(remote_base::RemoteReceiveData src) {
    const int32_t n = src.size();
    if (n < 82) return {};  // 40 bits min = 80 elements + preamble/sync overhead

    // Find sync gap: a negative value < -5000 µs followed by data marks 100-900 µs
    // This reliably identifies the start of data after preamble.
    int data_start = -1;
    for (int i = 0; i < n - 80; i++) {
      if (src[i] < -5000) {
        // Found large gap — next element should be first data mark
        // Need at least 79 elements after data_start (40 marks + 39 spaces; last bit may lack space)
        if (i + 1 <= n - 79 && src[i + 1] > 100 && src[i + 1] < 900) {
          data_start = i + 1;
          break;
        }
      }
    }
    if (data_start < 0) return {};

    ESP_LOGD(TAG_NEXUS, "decode: n=%d sync_gap at idx=%d data_start=%d", n, data_start - 1, data_start);

    // ── Decode 40 bits (PPM distance coding) ────────────────────────────
    // CC1101 OOK stretched timing:
    //   bit 0: mark + space ~1700-2500 µs
    //   bit 1: mark + space ~3500-4600 µs
    //   threshold: |space| > 3000 µs → bit 1
    //   gap_limit: |space| > 5500 µs → inter-frame (abort)
    uint64_t bits = 0;
    int idx = data_start;
    bool frame_ok = true;
    for (int i = 39; i >= 0; i--) {
      if (idx >= n) {
        // Allow last bit to have no space (trailing mark at buffer end)
        if (i == 0) break;
        frame_ok = false;
        break;
      }
      int32_t mark = src[idx++];
      if (mark < 100 || mark > 900) { frame_ok = false; break; }

      if (idx < n && src[idx] < 0) {
        int32_t space = src[idx];
        if (space < -5500) { frame_ok = false; break; }
        idx++;
        if (space < -3000) {
          bits |= (1ULL << i);
        }
      }
    }
    if (!frame_ok) {
      ESP_LOGD(TAG_NEXUS, "frame decode failed at idx=%d of %d", idx, n);
      return {};
    }

    ESP_LOGD(TAG_NEXUS, "decoded 40 bits: 0x%010" PRIx64, bits);

    // ── Extract fields from 40-bit frame ─────────────────────────────────
    // [id:8][flags:4][seq:4][temp:12][humidity_BCD:8][check:4]
    uint8_t  b_id    = (bits >> 32) & 0xFF;
    uint8_t  b_flags = (bits >> 28) & 0x0F;
    uint8_t  b_seq   = (bits >> 24) & 0x0F;
    uint16_t temp_raw = (bits >> 12) & 0xFFF;  // 12-bit ADC: bits[23:12]
    uint8_t  hum_raw = (bits >> 4) & 0xFF;
    uint8_t  b_check = bits & 0x0F;

    // Reject all-zeros or all-ones ID
    if (b_id == 0 || b_id == 0xFF) {
      ESP_LOGD(TAG_NEXUS, "rejected id=0x%02X", b_id);
      return {};
    }

    NexusData data;
    data.id = b_id;

    // Channel: lower 2 bits of flags (standard mapping)
    data.channel    = (b_flags & 0x03) + 1;
    data.battery_ok = (b_flags >> 3) & 1;
    data.test_mode  = (b_flags >> 2) & 1;

    // Temperature: 12-bit raw ADC, empirical formula from 7 calibration points
    // (raw - 1221) / 18.0 = °C, R²=0.9999, max error ±0.06°C
    data.temperature = (temp_raw - 1221) / 18.0f;

    // Humidity: try BCD decode first (0x72 → 72), fallback to raw
    uint8_t hum_bcd = ((hum_raw >> 4) & 0x0F) * 10 + (hum_raw & 0x0F);
    if (hum_bcd <= 100 && (hum_raw & 0x0F) <= 9 && ((hum_raw >> 4) & 0x0F) <= 9) {
      data.humidity = hum_bcd;  // Valid BCD
    } else {
      data.humidity = hum_raw;  // Fallback to raw
    }

    // Validate temperature range
    if (data.temperature < -40.0f || data.temperature > 80.0f) {
      ESP_LOGD(TAG_NEXUS, "temp %.1f out of range", data.temperature);
      return {};
    }
    if (data.humidity > 100) {
      ESP_LOGD(TAG_NEXUS, "humidity %d out of range", data.humidity);
      return {};
    }

    ESP_LOGD(TAG_NEXUS, "parsed: id=0x%02X flags=0x%X seq=0x%X temp_raw=%d(0x%03X) hum_raw=0x%02X chk=0x%X -> %.1f°C %d%%",
             b_id, b_flags, b_seq, temp_raw, temp_raw, hum_raw, b_check,
             data.temperature, data.humidity);

    return data;
  }

  void dump(const NexusData &data) {
    ESP_LOGI(TAG_NEXUS, "Received TE81: id=%d ch=%d battery=%s temp=%.1f°C humidity=%d%%",
             data.id, data.channel,
             data.battery_ok ? "OK" : "LOW",
             data.temperature, data.humidity);
  }
};

}  // namespace rf433_gw
}  // namespace esphome
