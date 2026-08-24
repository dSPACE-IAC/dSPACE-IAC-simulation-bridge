#include "npc_controller.hpp"

#include <optional>
#include <string_view>

namespace controller
{

    void ControllerNode::buildMessageLookup()
    {
        message_lookup_.clear();
        message_signal_lookup_.clear();
        message_name_lookup_.clear();
        message_lookup_.reserve(can_message_info.size());
        message_signal_lookup_.reserve(can_message_info.size());
        message_name_lookup_.reserve(can_message_info.size());

        for (auto &message : can_message_info) {
        message_lookup_.emplace(message.id, &message);
        message_name_lookup_.emplace(std::string_view(message.name), &message);
        auto &signal_map = message_signal_lookup_[message.id];
        signal_map.reserve(message.signals.size());
        for (const auto &signal : message.signals) {
            signal_map.emplace(signal.name, &signal);
        }
        }
    }

    const Message* ControllerNode::findMessageByID(uint32_t message_id) const
    {
        const auto iter = message_lookup_.find(message_id);
        if (iter == message_lookup_.end()) {
        return nullptr;
        }
        return iter->second;
    }

    const ControllerNode::SignalLookupMap* ControllerNode::findSignalLookup(uint32_t message_id) const
    {
        const auto iter = message_signal_lookup_.find(message_id);
        if (iter == message_signal_lookup_.end()) {
        return nullptr;
        }
        return &iter->second;
    }

    const Signal* ControllerNode::findSignal(uint32_t message_id, std::string_view signal_name) const
    {
        const auto *signal_map = findSignalLookup(message_id);
        if (!signal_map) {
        return nullptr;
        }

        const auto iter = signal_map->find(signal_name);
        if (iter == signal_map->end()) {
        return nullptr;
        }
        return iter->second;
    }

    const Message* ControllerNode::findMessageByName(std::string_view message_name) const
    {
        const auto iter = message_name_lookup_.find(message_name);
        if (iter == message_name_lookup_.end()) {
        return nullptr;
        }
        return iter->second;
    }

    int32_t ControllerNode::extractBits(const uint8_t* data, Signal signal_information) const
    {
        return unpack_signal_bits(data, signal_information);
    }

    std::optional<double>
    ControllerNode::extractSignalScaled(uint32_t message_id, std::string_view signal_name, const uint8_t* data) const
    {
        const auto *signal = findSignal(message_id, signal_name);
        if (!signal) {
        return std::nullopt;
        }
        const auto raw = extractBits(data, *signal);
        return static_cast<double>(raw) * signal->factor + signal->offset;
    }

} // namespace controller