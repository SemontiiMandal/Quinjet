#ifndef RADIO_ESB_H
#define RADIO_ESB_H

#include <stdint.h>
#include <stdbool.h>
#include "drone_packet.h" // pull in the shared struct

// globally accessible payload and lock
extern data_packet latest_joystick_state;
extern struct k_mutex joystick_data;

int radio_esb_init(bool is_transmitter);
void radio_esb_send_packet(uint8_t *data, uint8_t length);

#endif 