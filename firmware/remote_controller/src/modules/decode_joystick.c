#include <zephyr/kernel.h>
#include "decode_joystick.h"
#include <stdio.h>

// Decode the ADC read into "UP, DOWN, LEFT, RIGHT" takes away all precision!!!
// So, Instead of discrete directions, code should treat the joystick as an (X, Y) coordinate space, where:
// (0, 0) is the dead center, (-1.0, -1.0) is bottom-left, (+1.0, +1.0) is top-right.

// Extreme Left: 0; Physical Center: 2048; Extreme Right: 4095

// decode to normalized floats
coordinate_normalized decode_joystick(uint16_t raw_x, uint16_t raw_y){
    coordinate_normalized output = {0.0f, 0.0f};

    int32_t offset_x = (int32_t)raw_x - ADC_CENTER;
    int32_t offset_y = (int32_t)raw_y - ADC_CENTER;

    if (abs(offset_x) > JITTER){
        if (offset_x > 0)
            output.x = (float)(offset_x - JITTER) / (ADC_CENTER - JITTER);
        else 
            output.x = (float)(offset_x + JITTER) / (ADC_CENTER - JITTER);
    }

    if (abs(offset_y) > JITTER){
        if (offset_y > 0)
            output.y = (float)(offset_y - JITTER) / (ADC_CENTER - JITTER);
        else 
            output.y = (float)(offset_y + JITTER) / (ADC_CENTER - JITTER);
    }

    // clamp limits
    if (output.x > 1.0f) output.x = 1.0f;
    if (output.x < -1.0f) output.x = -1.0f;
    if (output.y > 1.0f) output.y = 1.0f;
    if (output.y < -1.0f) output.y = -1.0f;

    return output;
}