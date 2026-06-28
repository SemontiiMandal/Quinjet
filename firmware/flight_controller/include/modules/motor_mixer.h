#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdint.h>
#include "drivers/motor_pwm.h"

motor_outputs_t mix_motors(float throttle_rc, float pitch_pid, float roll_pid, float yaw_pid, uint8_t arm_state);

#endif