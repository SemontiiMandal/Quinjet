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

void pid_init(pid_controller* pid, float p, float i, float d, float i_limit, float out_limit, float cutoff_hz, float sample_hz) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->previous_error = 0.0f;
    pid->integral_sum = 0.0f;
    pid->integral_limit = i_limit;
    pid->output_limit = out_limit;

    // Initialize LPF 
    init_biquad_lpf_coeffs(pid, cutoff_hz, sample_hz);
}

float pid_update(pid_controller* pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // Proportional Term
    float P_out = pid->Kp * error;

    // Integral Term (with Anti-Windup)
    pid->integral_sum += (error * dt);
    if (pid->integral_sum > pid->integral_limit) pid->integral_sum = pid->integral_limit;
    else if (pid->integral_sum < -pid->integral_limit) pid->integral_sum = -pid->integral_limit;
    float I_out = pid->Ki * pid->integral_sum;

    // Filtered Derivative Term 
    float raw_derivative = (error - pid->previous_error) / dt;
    float filtered_derivative = 0.0f;

    // Pass 1 sample through CMSIS-DSP Biquad LPF
    arm_biquad_cascade_df2T_f32(&pid->dterm_lpf, &raw_derivative, &filtered_derivative, 1);

    float D_out = pid->Kd * filtered_derivative;

    pid->previous_error = error;

    // Output Clamp
    float total_output = P_out + I_out + D_out;
    if (total_output > pid->output_limit) total_output = pid->output_limit;
    else if (total_output < -pid->output_limit) total_output = -pid->output_limit;

    return total_output;
}