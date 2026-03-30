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
// Registered as a RemoteReceiverListener, decodes all supported protocols
class RF433GWReceiver : public Component,
                        public remote_base::RemoteReceiverListener {
 public:
  bool on_receive(remote_base::RemoteReceiveData src) override {
    // Self-reception protection
    if (is_transmitting_) return true;

    // Try A-OK decode (src is passed by value, each decode gets own copy)
    {
      AOKProtocol aok_proto;
      auto aok_data = aok_proto.decode(src);
      if (aok_data.has_value()) {
        // Debounce: same remote+command within debounce window
        uint32_t now = millis();
        if (aok_data->remote_id == last_aok_remote_id_ &&
            aok_data->command == last_aok_command_ &&
            (now - last_aok_time_) < debounce_ms_) {
          return true;
        }
        last_aok_remote_id_ = aok_data->remote_id;
        last_aok_command_ = aok_data->command;
        last_aok_time_ = now;

        aok_proto.dump(*aok_data);
        for (auto *trigger : aok_triggers_)
          trigger->process(*aok_data);
        return true;
      }
    }

    // Try Nexus decode (fresh copy of src via pass-by-value in decode)
    {
      NexusProtocol nexus_proto;
      auto nexus_data = nexus_proto.decode(src);
      if (nexus_data.has_value()) {
        // Debounce: same sensor within debounce window
        uint32_t now = millis();
        if (nexus_data->id == last_nexus_id_ &&
            nexus_data->channel == last_nexus_ch_ &&
            (now - last_nexus_time_) < debounce_ms_) {
          return true;
        }
        last_nexus_id_ = nexus_data->id;
        last_nexus_ch_ = nexus_data->channel;
        last_nexus_time_ = now;

        nexus_proto.dump(*nexus_data);
        for (auto *trigger : nexus_triggers_)
          trigger->process(*nexus_data);
        return true;
      }
    }

    return false;
  }

  void set_transmitting(bool v) { is_transmitting_ = v; }
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

  void play(Ts... x) override {
    // Self-reception protection: mark as transmitting
    if (receiver_ != nullptr) {
      receiver_->set_transmitting(true);
    }

    AOKData data;
    data.remote_id = this->remote_id_.value(x...);
    data.address   = this->address_.value(x...);
    data.command   = this->command_.value(x...);

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

    // Release self-reception protection after a delay
    if (receiver_ != nullptr) {
      // Delay is handled by ESPHome's remote_transmitter on_complete callback
      // We set it to false; the user should set up on_complete → recv()
      receiver_->set_transmitting(false);
    }
  }

 protected:
  remote_base::RemoteTransmitterBase *transmitter_;
  RF433GWReceiver *receiver_;
};

}  // namespace rf433_gw
}  // namespace esphome
