#include <stdint.h>
#include "drivers/motor_pwm.h"

// Direct-Drive PWM Boundaries (0% to 100% Duty Cycle)
#define MAX_DUTY_CYCLE 10000.0f  // 100% power
#define MIN_ARMED_DUTY 500.0f    // 5% power to keep props spinning when armed
#define MOTOR_OFF      0         // 0% power

motor_outputs_t mix_motors(float throttle_rc, float pitch_pid, float roll_pid, float yaw_pid, uint8_t arm_state) {
    motor_outputs_t outputs;

    // Safety Gate: Cut all power if disarmed or throttle is at zero stick
    if (arm_state == 0 || throttle_rc <= 0.05f) {
        outputs.front_left  = MOTOR_OFF;
        outputs.front_right = MOTOR_OFF;
        outputs.rear_left   = MOTOR_OFF;
        outputs.rear_right  = MOTOR_OFF;
        return outputs;
    }

    // Full Throttle
    // Map 0.0 - 1.0 directly to the full usable duty cycle range
    float base_throttle = MIN_ARMED_DUTY + (throttle_rc * (MAX_DUTY_CYCLE - MIN_ARMED_DUTY)); 

    // Mixing Matrix
    float m1 = base_throttle - pitch_pid + roll_pid + yaw_pid; // Front Left
    float m2 = base_throttle - pitch_pid - roll_pid - yaw_pid; // Front Right
    float m3 = base_throttle + pitch_pid + roll_pid - yaw_pid; // Rear Left
    float m4 = base_throttle + pitch_pid - roll_pid + yaw_pid; // Rear Right

    //  Mixer Scaling (Overflow / High-Throttle Desaturation)
    /*
    If a motor overflows past 100%, we calculate the exact overflow amount and subtract it from all four motors equally. This sacrifices a tiny amount of overall altitude, but preserves the differential thrust required to keep the drone from flipping.
    */
    // Find the highest commanded motor value
    float max_motor = m1;
    if (m2 > max_motor) max_motor = m2;
    if (m3 > max_motor) max_motor = m3;
    if (m4 > max_motor) max_motor = m4;

    // If any motor exceeds 100%, pull ALL motors down by the exact overflow amount.
    if (max_motor > MAX_DUTY_CYCLE) {
        float overflow = max_motor - MAX_DUTY_CYCLE;
        m1 -= overflow;
        m2 -= overflow;
        m3 -= overflow;
        m4 -= overflow;
    }

    // AirMode Scaling (Underflow / Zero-Throttle Desaturation)
    /*
    If pilot drops throttle to zero during a dive, the PID controller normally loses authority because the motors can't spin slower than the minimum idle speed. Underflow scaling detects this and slightly revs all motors up just enough to give the PID room to steer, keeping the drone perfectly stabilized even in a free-fall.
    */
    // Find the lowest commanded motor value after the overflow shift
    float min_motor = m1;
    if (m2 < min_motor) min_motor = m2;
    if (m3 < min_motor) min_motor = m3;
    if (m4 < min_motor) min_motor = m4;

    // If any motor drops below the minimum idle speed, push ALL motors up.
    if (min_motor < MIN_ARMED_DUTY) {
        float underflow = MIN_ARMED_DUTY - min_motor;
        m1 += underflow;
        m2 += underflow;
        m3 += underflow;
        m4 += underflow;
    }

    // Hardware Safety Clamp and Assignment
    // Fail-safe to guarantee no illegal values reach PWM hardware
    outputs.front_left  = (uint32_t)(m1 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m1 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m1));
    outputs.front_right = (uint32_t)(m2 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m2 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m2));
    outputs.rear_left   = (uint32_t)(m3 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m3 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m3));
    outputs.rear_right  = (uint32_t)(m4 > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : (m4 < MIN_ARMED_DUTY ? MIN_ARMED_DUTY : m4));

    return outputs;
}