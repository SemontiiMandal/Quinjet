#ifndef DECODE_JOYSTICK_H
#define DECODE_JOYSTICK_H

// 12 bit resolution; So min 0, and max 2^12 - 1

#define ADC_MAX 4095
#define ADC_CENTER 2048
#define JITTER 100 // needs some tuning
// for example, leftmost end should read 0, but now reads 100, so there's some jitter that persists

typedef struct{
    float x;
    float y;
} coordinate_normalized; // between -1.0 to +1.0

static int decode_joystick(uint16_t raw_x, uint16_t raw_y);

#endif