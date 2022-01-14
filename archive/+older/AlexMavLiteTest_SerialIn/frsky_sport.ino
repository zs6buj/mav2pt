#include "frsky_sport.h"

bool parse_sport_telemetry_data(uint8_t data, sport_parse_state_t &parse_state)
{
    switch (parse_state.state) {
        case SPortParseState::START:
            if (data == FRAME_HEAD) {
                parse_state.state = SPortParseState::IN_FRAME ;
                parse_state.telemetry_rx_buffer_count = 0;
            } else {
                if (parse_state.telemetry_rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
                    parse_state.telemetry_rx_buffer[parse_state.telemetry_rx_buffer_count++] = data;
                }
                parse_state.state = SPortParseState::IN_FRAME;
            }
            break;

        case SPortParseState::IN_FRAME:
            if (data == FRAME_DLE) {
                parse_state.state = SPortParseState::XOR; // XOR next byte
            } else if (data == FRAME_HEAD) {
                parse_state.state = SPortParseState::IN_FRAME ;
                parse_state.telemetry_rx_buffer_count = 0;
                break;
            } else if (parse_state.telemetry_rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
                parse_state.telemetry_rx_buffer[parse_state.telemetry_rx_buffer_count++] = data;
            }
            break;

        case SPortParseState::XOR:
            if (parse_state.telemetry_rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
                parse_state.telemetry_rx_buffer[parse_state.telemetry_rx_buffer_count++] = data ^ STUFF_MASK;
            }
            parse_state.state = SPortParseState::IN_FRAME;
            break;

        case SPortParseState::IDLE:
            if (data == FRAME_HEAD) {
                parse_state.telemetry_rx_buffer_count = 0;
                parse_state.state = SPortParseState::START;
            }
            break;

    } // switch

    if (parse_state.telemetry_rx_buffer_count >= SPORT_PACKET_SIZE) {
        parse_state.telemetry_rx_buffer_count = 0;
        parse_state.state = SPortParseState::IDLE;
        return true;
    }
    return false;
}
