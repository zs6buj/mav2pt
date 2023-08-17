#pragma once

#include "common.h"

#include <stdint.h>

// for SPort X protocol
#define FRAME_HEAD                  0x7E
#define FRAME_DLE                   0x7D
#define FRAME_XOR                   0x20

#define TELEMETRY_RX_BUFFER_SIZE    19U  // 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
#define SPORT_PACKET_SIZE           9U
#define STUFF_MASK                  0x20
#define SPORT_DATA_FRAME            0x10
#define SPORT_UPLINK_FRAME          0x30
#define SPORT_UPLINK_FRAME_RW       0x31
#define SPORT_DOWNLINK_FRAME        0x32

union sport_packet_t {
    struct PACKED {
        uint8_t sensor;
        uint8_t frame;
        uint16_t appid;
        uint32_t data;
    };
    uint8_t raw[8];
};

enum class SPortParseState : uint8_t {
    IDLE,
    START,
    IN_FRAME,
    XOR,
};

typedef struct PACKED {
    uint8_t telemetry_rx_buffer_count;
    uint8_t telemetry_rx_buffer[TELEMETRY_RX_BUFFER_SIZE];
    SPortParseState state;
} sport_parse_state_t;

bool parse_sport_telemetry_data(uint8_t data, sport_parse_state_t &parse_state);
