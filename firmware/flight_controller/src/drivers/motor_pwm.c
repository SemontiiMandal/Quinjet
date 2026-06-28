#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include "drivers/motor_pwm.h"

/*
    mot1 = top-left, mot2 = top-right, mot3 = bottom-left, mot4 = bottom-right
*/

static const struct pwm_dt_spec pwm_mot_1 = PWM_DT_SPEC_GET(DT_NODELABEL(mot_1)); 
static const struct pwm_dt_spec pwm_mot_2 = PWM_DT_SPEC_GET(DT_NODELABEL(mot_2)); 
static const struct pwm_dt_spec pwm_mot_3 = PWM_DT_SPEC_GET(DT_NODELABEL(mot_3)); 
static const struct pwm_dt_spec pwm_mot_4 = PWM_DT_SPEC_GET(DT_NODELABEL(mot_4)); 

// 32 kHz frequency for coreless motors (1,000,000,000 ns / 32,000 Hz = 31250 ns)
#define PWM_PERIOD_NS 31250

int app_pwm_init(void) {
    if (!device_is_ready(pwm_mot_1.dev) || !device_is_ready(pwm_mot_2.dev) || 
        !device_is_ready(pwm_mot_3.dev) || !device_is_ready(pwm_mot_4.dev)) {
        printk("PWMs not initialized.\n");
        return -1;
    }
    return 0;
}

// Call at 1000Hz from  PID thread
void app_pwm_set(motor_outputs_t *output) {
    
    // Convert the 0 - 10000 duty cycle from the mixer into nanosecond pulses
    uint32_t pulse_1 = (PWM_PERIOD_NS * output->front_left)  / 10000;
    uint32_t pulse_2 = (PWM_PERIOD_NS * output->front_right) / 10000;
    uint32_t pulse_3 = (PWM_PERIOD_NS * output->rear_left)   / 10000;
    uint32_t pulse_4 = (PWM_PERIOD_NS * output->rear_right)  / 10000;

    // Blast the new pulses to the hardware registers instantly
    // pwm_set_dt requires (dt_spec, period, pulse)
    pwm_set_dt(&pwm_mot_1, PWM_PERIOD_NS, pulse_1);
    pwm_set_dt(&pwm_mot_2, PWM_PERIOD_NS, pulse_2);
    pwm_set_dt(&pwm_mot_3, PWM_PERIOD_NS, pulse_3);
    pwm_set_dt(&pwm_mot_4, PWM_PERIOD_NS, pulse_4);
    
    // Return immediately so the PID loop can go back to sleep
}