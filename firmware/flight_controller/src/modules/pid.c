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

    // Proportional Term
    float P_out = pid->Kp * error;

    // Integral Term (Memory)
    pid->integral_sum += (error * dt);
    float I_out = pid->Ki * pid->integral_sum;

    // Filtered Derivative Term (On previous measurement term to prevent derivative kick)
    // Derivative Kick is a massive spike in the control output of a PID controller caused by an abrupt change in the setpoint
    float raw_derivative = -(measured - pid->previous_measurement) / dt;
    float filtered_derivative = 0.0f;
    arm_biquad_cascade_df2T_f32(&pid->dterm_lpf, &raw_derivative, &filtered_derivative, 1);
    float D_out = pid->Kd * filtered_derivative;

    pid->previous_measurement = measured;

    // Calculate total theoretical output
    float total_output = P_out + I_out + D_out;
    float final_output = total_output;

    // Back-Calculation Anti-Windup
    float excess = 0.0f;
    
    // If output exceeds physical limits, clamp it and calculate the phantom power
    if (total_output > pid->output_limit) {
        final_output = pid->output_limit; 
        excess = total_output - pid->output_limit; 
    } else if (total_output < -pid->output_limit) {
        final_output = -pid->output_limit;
        excess = total_output - (-pid->output_limit);
    }

    // Eliminate Overshoot, remove this from I-term
    if (excess != 0.0f) {
        pid->integral_sum -= (excess * pid->Kb * dt);
    }

    return final_output;
}

/*
If drone is blown off course by a gust of wind, the PID math might calculate that it needs 130% motor power to instantly snap back to level. However, a physical motor maxes out at 100%. That extra 30% is controller saturation overflow and causes integral windups if not handled (i.e. I-term keeps rapidly accumulating more and more error)!!

By the time the drone finally reaches a level hover, the I-term has built up a massive, unnecessary memory. That bloated memory will immediately force the drone to violently over-correct in the opposite direction before the integral math has time to drain back to zero. So here we check if the math exceeds reality, calculating the exact amount of overshoot, and eliminating it. 
*/