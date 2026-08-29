#ifndef PID_H
#define PID_H

#include <arm_math.h>
#include <dsp/filtering_functions.h>

#define M_PI 3.14159265358979323846f

typedef struct {

    // Tuning Parameters P, I and D
    float Kp;
    float Ki;
    float Kd;
    float Kb; // Back-calculation tracking gain

    // State Memory
    // float previous_error;
    float previous_measurement;
    float integral_sum;

    // Safety Limits
    float integral_limit; // Anti windup (clamps thememory so that if drone hits something and gets stuck the I term doesn't build up to infinity and burns the motors out)
    float output_limit; // Max PWM correction allowed

    // CMSIS-DSP Biquad LPF for D-Term
    arm_biquad_cascade_df2T_instance_f32 dterm_lpf;
    float lpf_coeffs[5];  // {b0, b1, b2, -a1, -a2}
    float lpf_state[2];   // State memory for 1 biquad stage

} pid_controller;

void pid_init(pid_controller* pid, float p, float i, float d, float b, float i_limit, float out_limit, float cutoff_hz, float sample_hz);
float pid_update(pid_controller* pid, float setpoint, float measured, float dt);

#endif