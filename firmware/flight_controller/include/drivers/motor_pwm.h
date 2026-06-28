#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include <stdint.h>

typedef struct {
    uint32_t front_left;  // Motor 1 (CW)
    uint32_t front_right; // Motor 2 (CCW)
    uint32_t rear_left;   // Motor 3 (CCW)
    uint32_t rear_right;  // Motor 4 (CW)
} motor_outputs_t;

int app_pwm_init(void);
void app_pwm_set(motor_outputs_t *output);

#endif