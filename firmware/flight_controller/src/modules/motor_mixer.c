#include <stdint.h>
#include "drivers/motor_pwm.h"

// Direct-Drive PWM Boundaries (0% to 100% Duty Cycle)
#define MAX_DUTY_CYCLE 10000.0f  // 100% power
#define MIN_ARMED_DUTY 500.0f    // 5% power to keep props spinning when armed
#define MOTOR_OFF      0         // 0% power

motor_outputs_t mix_motors(float throttle_rc, float pitch_pid, float roll_pid, float yaw_pid, uint8_t arm_state) {
    motor_outputs_t outputs;

    // 1. Safety Gate: If disarmed or throttle is zero, cut all power
    if (arm_state == 0 || throttle_rc <= 0.05f) {
        outputs.front_left  = MOTOR_OFF;
        outputs.front_right = MOTOR_OFF;
        outputs.rear_left   = MOTOR_OFF;
        outputs.rear_right  = MOTOR_OFF;
        return outputs;
    }

    // 2. Map normalized throttle (0.0 to 1.0) to your Duty Cycle range
    // We leave some headroom (e.g., max base throttle is 80%) so the PID 
    // still has power left to stabilize the drone at full throttle.
    float base_throttle = MIN_ARMED_DUTY + (throttle_rc * (MAX_DUTY_CYCLE * 0.8f)); 

    // 3. The Mixing Matrix
    // The PID outputs now represent raw duty-cycle additions/subtractions
    float m1 = base_throttle - pitch_pid + roll_pid + yaw_pid; // Front Left
    float m2 = base_throttle - pitch_pid - roll_pid - yaw_pid; // Front Right
    float m3 = base_throttle + pitch_pid + roll_pid - yaw_pid; // Rear Left
    float m4 = base_throttle + pitch_pid - roll_pid + yaw_pid; // Rear Right

    // 4. The Output Clamps (Prevent hardware overflow)
    outputs.front_left  = (uint32_t)(m1 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m1 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m1));
    outputs.front_right = (uint32_t)(m2 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m2 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m2));
    outputs.rear_left   = (uint32_t)(m3 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m3 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m3));
    outputs.rear_right  = (uint32_t)(m4 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m4 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m4));

    return outputs;
}