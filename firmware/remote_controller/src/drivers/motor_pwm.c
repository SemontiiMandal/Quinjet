#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_vbmot = PWM_DT_SPEC_GET(DT_NODELABEL(my_pwm_node)); 

// Renamed function to avoid conflict with Zephyr's internal pwm_set API
int app_pwm_set(void){
    if (!device_is_ready(pwm_vbmot.dev)){
        printk("PWM not initialized.\n");
        return -1;
    }
    // Set PWM frequency and duty cycle
    uint32_t period_hz = 1000;
    uint32_t pulse_ns = (1000000000 / period_hz) / 2;
    
    while(1){
        pwm_set_dt(&pwm_vbmot, PWM_HZ(period_hz), pulse_ns);
        k_msleep(1000);
    }

    return 0;
}