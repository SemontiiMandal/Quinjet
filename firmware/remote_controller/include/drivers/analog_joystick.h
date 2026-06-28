#ifndef ANALOG_JOYSTICK_H
#define ANALOG_JOYSTICK_H

#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
} joystick;

// declare semaphore to wake ESB thread
extern struct k_sem wake_esb;

// expose the init function to main
int joystick_init(void);

#endif