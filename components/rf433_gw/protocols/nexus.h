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

// ─── Nexus/Solight TE81 Timing constants (µs) ──────────────────────────────
// PPM modulation: ~500µs pulse, short gap ~1000µs (bit 0), long gap ~2000µs (bit 1)
// Sync: ~500µs pulse + ~4000µs gap
// Reset: > 5000µs gap
static const uint32_t NEXUS_PULSE_US      = 500;
static const uint32_t NEXUS_SHORT_GAP_US  = 1000;  // bit 0
static const uint32_t NEXUS_LONG_GAP_US   = 2000;  // bit 1
static const uint32_t NEXUS_SYNC_GAP_US   = 4000;  // sync gap

// ─── Nexus Protocol (RX-only decoder) ──────────────────────────────────────
class NexusProtocol {
 public:
  optional<NexusData> decode(remote_base::RemoteReceiveData src);
  void dump(const NexusData &data);
};

}  // namespace rf433_gw
}  // namespace esphome
