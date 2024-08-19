#pragma once

#include <cstdint>
#include <array>

/**
 * @brief Description of the different diagnostic counters
*/
typedef enum {

    message_sent_count = 5,
    return_bus_message_count,
    bytes_out_count,
    bytes_in_count,
    nothing_0,
    return_server_exception_error_count = 10,
    return_server_NAK_count,
    return_server_busy_count,
    unexpected_responder, //responder has wrong address or response to broadcast
    crc_error_count,
    return_server_no_response_count = 15,
    unexpected_interchar,
    ignoring_state_error, //entered ignoring state
    unhandled_isr,

} diagnostic_counter_t;

static constexpr int diagnostic_counters_array_size = 20;
class DiagnosticsTracker
{
public:
    DiagnosticsTracker()
    {
        diagnostic_counters.fill(0);
    };

    void increment_diagnostic_counter(diagnostic_counter_t diagnostic_number)
    {
        if (diagnostic_number >= diagnostic_counters_array_size || diagnostic_number < 0)
        {
            std::cout << "Writing out of bounds of diagnostic counters.\n";
            return;
        }

        diagnostic_counters[diagnostic_number]++;
    }

    uint16_t Get(int diagnostic_number)
    {
        if (diagnostic_number >= diagnostic_counters_array_size || diagnostic_number < 0)
        {
            std::cout << "Reading out of bounds of diagnostic counters.\n";
            return -1;
        }

        return diagnostic_counters[diagnostic_number];
    }

    uint16_t operator [](int diagnostic_number)
    {
        return Get(diagnostic_number);
    }

private:
    std::array<uint16_t, diagnostic_counters_array_size> diagnostic_counters;
};