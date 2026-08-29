#include <math.h>
#include "modules/pid.h"
#include <string.h> // Need to keep this for memset

static void init_biquad_lpf_coeffs(pid_controller* pid, float cutoff_hz, float sample_hz) {
    float omega = 2.0f * M_PI * cutoff_hz / sample_hz;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.7071f); // Q = 0.7071 (Butterworth response)

    float b0 = (1.0f - cs) / 2.0f;
    float b1 = 1.0f - cs;
    float b2 = (1.0f - cs) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;

    // CMSIS-DSP expects: {b0/a0, b1/a0, b2/a0, -a1/a0, -a2/a0}
    pid->lpf_coeffs[0] = b0 / a0;
    pid->lpf_coeffs[1] = b1 / a0;
    pid->lpf_coeffs[2] = b2 / a0;
    pid->lpf_coeffs[3] = -a1 / a0; 
    pid->lpf_coeffs[4] = -a2 / a0;

    memset(pid->lpf_state, 0, sizeof(pid->lpf_state));

    // Initialize 1 stage (numStages = 1)
    arm_biquad_cascade_df2T_init_f32(&pid->dterm_lpf, 1, pid->lpf_coeffs, pid->lpf_state);
}

void pid_init(pid_controller* pid, float p, float i, float d, float b, float i_limit, float out_limit, float cutoff_hz, float sample_hz) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->Kb = b;
    pid->previous_measurement = 0.0f;
    pid->integral_sum = 0.0f;
    pid->integral_limit = i_limit;
    pid->output_limit = out_limit;

    // Initialize LPF 
    init_biquad_lpf_coeffs(pid, cutoff_hz, sample_hz);
}

float pid_update(pid_controller* pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // 1. Proportional Term
    float P_out = pid->Kp * error;

    // 2. Integral Term (Standard accumulation)
    pid->integral_sum += (error * dt);
    float I_out = pid->Ki * pid->integral_sum;

    // 3. Filtered Derivative Term (On measurement to prevent kick)
    float raw_derivative = -(measured - pid->previous_measurement) / dt;
    float filtered_derivative = 0.0f;
    arm_biquad_cascade_df2T_f32(&pid->dterm_lpf, &raw_derivative, &filtered_derivative, 1);
    float D_out = pid->Kd * filtered_derivative;

    pid->previous_measurement = measured;

    // 4. Calculate total theoretical output
    float total_output = P_out + I_out + D_out;
    float final_output = total_output;

    // 5. Back-Calculation Anti-Windup
    float excess = 0.0f;
    
    // If output exceeds physical limits, clamp it and calculate the phantom power
    if (total_output > pid->output_limit) {
        final_output = pid->output_limit; 
        excess = total_output - pid->output_limit; 
    } else if (total_output < -pid->output_limit) {
        final_output = -pid->output_limit;
        excess = total_output - (-pid->output_limit);
    }

    // Mathematically drain the phantom power from the integral memory
    if (excess != 0.0f) {
        pid->integral_sum -= (excess * pid->Kb * dt);
    }

    return final_output;
}