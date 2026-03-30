#include "nexus.h"

namespace esphome {
namespace rf433_gw {

// ─── Nexus/Solight TE81 Decoder ────────────────────────────────────────────
// Protocol: 36 bits, PPM (distance coding)
// Pulse ~500µs, short gap ~1000µs = bit 0, long gap ~2000µs = bit 1
// Sync: ~500µs pulse + ~4000µs gap
//
// Data: [id:8][flags:4][temp:12][0xF:4][humidity:8]
// Flags: B T CC (Battery, Test, Channel)
// Temperature: 12-bit signed, scaled by 10 (i.e. value/10 = °C)
// Humidity: 0 = no sensor

optional<NexusData> NexusProtocol::decode(remote_base::RemoteReceiveData src) {
  // Nexus sends 36 bits 12 times. We need to find at least one valid frame.
  // Look for sync: pulse ~500µs then gap > 3000µs

  // Scan for sync pulse
  bool sync_found = false;
  for (int attempt = 0; attempt < 10; attempt++) {
    if (src.expect_item(NEXUS_PULSE_US, NEXUS_SYNC_GAP_US)) {
      sync_found = true;
      break;
    }
    // Try to skip one bit and look again
    if (src.expect_item(NEXUS_PULSE_US, NEXUS_SHORT_GAP_US)) continue;
    if (src.expect_item(NEXUS_PULSE_US, NEXUS_LONG_GAP_US)) continue;
    break;
  }

  if (!sync_found) {
    return {};
  }

  // Decode 36 bits (PPM: pulse + gap, short=0, long=1)
  uint64_t bits = 0;
  for (int i = 35; i >= 0; i--) {
    if (src.expect_item(NEXUS_PULSE_US, NEXUS_LONG_GAP_US)) {
      bits |= (1ULL << i);
    } else if (src.expect_item(NEXUS_PULSE_US, NEXUS_SHORT_GAP_US)) {
      // bit stays 0
    } else {
      // Also accept the last bit with a sync gap or longer gap (end of frame)
      if (i == 0 && src.expect_mark(NEXUS_PULSE_US)) {
        // Last bit consumed, trailing space can be anything
      } else {
        return {};
      }
    }
  }

  // Extract bytes
  uint8_t b[5];
  b[0] = (bits >> 28) & 0xFF;  // id
  b[1] = (bits >> 24) & 0x0F;  // flags (only lower nibble)
  // temp in bits 23..12
  uint16_t temp_raw = (bits >> 12) & 0xFFF;
  b[2] = (bits >> 8) & 0x0F;   // const nibble (should be 0xF)
  b[3] = (bits >> 4) & 0x0F;   // humidity high nibble
  b[4] = bits & 0x0F;          // humidity low nibble

  // Validate constant nibble = 0xF
  if (b[2] != 0x0F) {
    ESP_LOGV(TAG_NEXUS, "Const nibble mismatch: 0x%X (expected 0xF)", b[2]);
    return {};
  }

  // Reduce false positives: ID must not be 0
  if (b[0] == 0) {
    return {};
  }

  NexusData data;
  data.id         = b[0];
  data.battery_ok = (b[1] >> 3) & 1;
  data.test_mode  = (b[1] >> 2) & 1;
  data.channel    = (b[1] & 0x03) + 1;  // channel 1-4

  // Temperature: 12-bit signed, divided by 10
  int16_t temp_signed = (int16_t)(temp_raw << 4) >> 4;  // sign-extend 12-bit to 16-bit
  data.temperature = temp_signed / 10.0f;

  // Humidity
  data.humidity = (b[3] << 4) | b[4];

  return data;
}

void NexusProtocol::dump(const NexusData &data) {
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

}  // namespace rf433_gw
}  // namespace esphome
