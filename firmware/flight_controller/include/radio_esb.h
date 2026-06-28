#ifndef RADIO_ESB_H
#define RADIO_ESB_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/spinlock.h>
#include "drone_packet.h" 

// Expose the shared memory and the lock to the flight control thread
extern data_packet latest_rc_command;
extern struct k_spinlock rc_spinlock;

int radio_esb_init(bool is_transmitter);

#endif