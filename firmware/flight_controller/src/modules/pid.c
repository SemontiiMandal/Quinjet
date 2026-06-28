#include "modules/pid.h"

void pid_init(pid_controller* pid, float p, float i, float d, float i_limit, float out_limit){
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;

    pid->previous_error = 0.0f;
    pid->integral_sum = 0.0f;
    pid->integral_limit = i_limit;
    pid->output_limit = out_limit;
}

float pid_update(pid_controller* pid, float setpoint, float measured, float dt){
    float error = setpoint - measured;

    // Proportional Term
    float P_out = pid->Kp * error;

    // Integral Term (with Anti-Windup)
    pid->integral_sum += (error * dt);

    // Clamp the integral sum 
    if (pid->integral_sum > pid->integral_limit) pid->integral_sum = pid->integral_limit;
    else if (pid->integral_sum < -pid->integral_limit) pid->integral_sum = -pid->integral_limit;
    
    float I_out = pid->Ki * pid->integral_sum;

    // Derivative Term
    float derivative = (error - pid->previous_error) / dt;
    float D_out = pid->Kd * derivative;

    // Save error for the next loop
    pid->previous_error = error;

    // Calculate Total Output
    float total_output = P_out + I_out + D_out;

    // Clamp the final output to stay within valid PWM
    if (total_output > pid->output_limit) total_output = pid->output_limit;
    else if (total_output < -pid->output_limit) total_output = -pid->output_limit;

    return total_output;
}