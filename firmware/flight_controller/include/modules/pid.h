#ifndef PID_H
#define PID_H

typedef struct {

    // Tuning Parameters P, I and D
    float Kp;
    float Ki;
    float Kd;

    // State Memory
    float previous_error;
    float integral_sum;

    // Safety Limits
    float integral_limit; // Anti windup (clamps thememory so that if drone hits something and gets stuck the I term doesn't build up to infinity and burns the motors out)
    float output_limit; // Max PWM correction allowed
} pid_controller;

void pid_init(pid_controller *pid, float p, float i, float d, float i_limit, float out_limit);
float pid_update(pid_controller *pid, float setpoint, float measured, float dt);

#endif