#ifndef DECODE_JOYSTICK_H
#define DECODE_JOYSTICK_H

#include <stdint.h>

#define ADC_MAX 4095
#define ADC_CENTER 2048
#define JITTER 100 // needs some tuning

typedef struct {
    float x;
    float y;
} coordinate_normalized; 

// public prototype
coordinate_normalized decode_joystick(uint16_t raw_x, uint16_t raw_y);

#endif