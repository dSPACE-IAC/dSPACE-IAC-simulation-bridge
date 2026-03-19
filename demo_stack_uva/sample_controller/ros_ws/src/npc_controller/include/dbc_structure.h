#ifndef DBC_STRUCTURE_H
#define DBC_STRUCTURE_H

#include <cstdint>
#include <vector>
#include <cstddef>

struct Signal {
    const char* name;
    uint8_t start_bit;
    uint8_t length;
    uint8_t endian;
    bool is_signed;
    float factor;
    float offset;
    float min_val;
    float max_val;
    const char* unit;
};

struct Message {
    uint32_t id;
    uint8_t dlc;
    const char* name;
    const char* transmitter;
    std::vector<Signal> signals;
    size_t signal_count;
};

extern std::vector<Message> initialize_messages();
#endif // DBC_STRUCTURE_H
