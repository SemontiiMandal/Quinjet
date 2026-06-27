#ifndef RADIO_ESB_H
#define RADIO_ESB_H

#include <stdint.h>
#include <stdbool.h>

// Our mock data structure for the test
typedef struct {
    uint32_t packet_id;
    float test_value;
} __attribute__((packed)) test_payload_t;
// compiler extension to disable padding bytes

// Public API Functions
int radio_esb_init(bool is_transmitter);
void radio_esb_send_packet(uint8_t *data, uint8_t length);

#endif // RADIO_ESB_H