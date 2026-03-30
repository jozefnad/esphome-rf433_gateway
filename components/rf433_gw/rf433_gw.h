#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/remote_base/remote_base.h"
#include "aok.h"
#include "nexus.h"

namespace esphome {
namespace rf433_gw {

static const char *const TAG_GW = "rf433_gw";

// ─── A-OK Trigger ──────────────────────────────────────────────────────────
class AOKTrigger : public Trigger<AOKData> {
 public:
  void process(const AOKData &data) {
    if (has_remote_id_ && data.remote_id != remote_id_) return;
    if (has_address_   && data.address   != address_)   return;
    if (has_command_   && data.command   != command_)   return;
    this->trigger(data);
  }

  void set_remote_id(uint32_t v) { remote_id_ = v; has_remote_id_ = true; }
  void set_address(uint16_t v)   { address_   = v; has_address_   = true; }
  void set_command(uint8_t v)    { command_   = v; has_command_   = true; }

 protected:
  uint32_t remote_id_{0};
  uint16_t address_{0};
  uint8_t  command_{0};
  bool has_remote_id_{false};
  bool has_address_{false};
  bool has_command_{false};
};

// ─── Nexus Trigger ─────────────────────────────────────────────────────────
class NexusTrigger : public Trigger<NexusData> {
 public:
  void process(const NexusData &data) {
    if (has_id_      && data.id      != id_)      return;
    if (has_channel_ && data.channel != channel_) return;
    this->trigger(data);
  }

  void set_id(uint8_t v)      { id_      = v; has_id_      = true; }
  void set_channel(uint8_t v) { channel_ = v; has_channel_ = true; }

 protected:
  uint8_t id_{0};
  uint8_t channel_{0};
  bool has_id_{false};
  bool has_channel_{false};
};

// ─── RF433 Gateway Receiver ────────────────────────────────────────────────
// Registered as a RemoteReceiverListener, decodes all supported protocols.
// Always returns false so built-in listeners (Dooya binary_sensor) can also process.
class RF433GWReceiver : public Component,
                        public remote_base::RemoteReceiverListener {
 public:
  bool on_receive(remote_base::RemoteReceiveData src) override {
    // Self-reception protection
    if (is_transmitting_) return false;

    // Quick size check — skip tiny buffers (noise)
    int32_t buf_size = src.size();
    if (buf_size < 20) return false;

    // Try A-OK decode — needs at least ~130 items (sync + 64 bits × 2)
    if (buf_size >= 130) {
      AOKProtocol aok_proto;
      auto aok_data = aok_proto.decode(src);
      if (aok_data.has_value()) {
        uint32_t now = millis();
        if (aok_data->remote_id != last_aok_remote_id_ ||
            aok_data->command != last_aok_command_ ||
            (now - last_aok_time_) >= debounce_ms_) {
          last_aok_remote_id_ = aok_data->remote_id;
          last_aok_command_ = aok_data->command;
          last_aok_time_ = now;

          aok_proto.dump(*aok_data);
          for (auto *trigger : aok_triggers_)
            trigger->process(*aok_data);
        }
      }
    }

    // Try Nexus decode — needs at least ~73 items (sync + 36 bits × 2)
    if (buf_size >= 36) {
      NexusProtocol nexus_proto;
      auto nexus_data = nexus_proto.decode(src);
      if (nexus_data.has_value()) {
        uint32_t now = millis();
        if (nexus_data->id != last_nexus_id_ ||
            nexus_data->channel != last_nexus_ch_ ||
            (now - last_nexus_time_) >= debounce_ms_) {
          last_nexus_id_ = nexus_data->id;
          last_nexus_ch_ = nexus_data->channel;
          last_nexus_time_ = now;

          nexus_proto.dump(*nexus_data);
          for (auto *trigger : nexus_triggers_)
            trigger->process(*nexus_data);
        }
      }
    }

    // Always return false — allow built-in Dooya listener and dump to proceed
    return false;
  }

  void set_transmitting(bool v) {
    is_transmitting_ = v;
    ESP_LOGD(TAG_GW, "set_transmitting(%s)", v ? "true" : "false");
  }
  void set_debounce(uint32_t ms) { debounce_ms_ = ms; }

  void add_aok_trigger(AOKTrigger *t) { aok_triggers_.push_back(t); }
  void add_nexus_trigger(NexusTrigger *t) { nexus_triggers_.push_back(t); }

 protected:
  std::vector<AOKTrigger *> aok_triggers_;
  std::vector<NexusTrigger *> nexus_triggers_;
  bool is_transmitting_{false};
  uint32_t debounce_ms_{500};

  // A-OK debounce state
  uint32_t last_aok_remote_id_{0};
  uint8_t last_aok_command_{0};
  uint32_t last_aok_time_{0};

  // Nexus debounce state
  uint8_t last_nexus_id_{0};
  uint8_t last_nexus_ch_{0};
  uint32_t last_nexus_time_{0};
};

// ─── A-OK Transmit Action ──────────────────────────────────────────────────
template<typename... Ts>
class AOKTransmitAction : public Action<Ts...> {
 public:
  explicit AOKTransmitAction(remote_base::RemoteTransmitterBase *transmitter,
                             RF433GWReceiver *receiver)
      : transmitter_(transmitter), receiver_(receiver) {}

  TEMPLATABLE_VALUE(uint32_t, remote_id)
  TEMPLATABLE_VALUE(uint16_t, address)
  TEMPLATABLE_VALUE(uint8_t,  command)
  TEMPLATABLE_VALUE(uint32_t, send_times)
  TEMPLATABLE_VALUE(uint32_t, send_wait)

  void play(const Ts &...x) override {
    AOKData data;
    data.remote_id = this->remote_id_.value(x...);
    data.address   = this->address_.value(x...);
    data.command   = this->command_.value(x...);

    ESP_LOGI(TAG_GW, "TX A-OK: remote_id=0x%06X addr=0x%04X cmd=0x%02X",
             data.remote_id, data.address, data.command);

    // Self-reception protection: mark as transmitting
    if (receiver_ != nullptr) {
      receiver_->set_transmitting(true);
    }

    auto call = this->transmitter_->transmit();
    AOKProtocol proto;
    proto.encode(call.get_data(), data);
    if (this->send_times_.has_value()) {
      call.set_send_times(this->send_times_.value(x...));
    }
    if (this->send_wait_.has_value()) {
      call.set_send_wait(this->send_wait_.value(x...));
    }
    call.perform();

    ESP_LOGD(TAG_GW, "TX A-OK: perform() completed");

    // Release self-reception protection
    if (receiver_ != nullptr) {
      receiver_->set_transmitting(false);
    }
  }

 protected:
  remote_base::RemoteTransmitterBase *transmitter_;
  RF433GWReceiver *receiver_;
};

}  // namespace rf433_gw
}  // namespace esphome
