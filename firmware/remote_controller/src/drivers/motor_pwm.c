#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_vbmot = PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(my_pwm_node), motor);

int pwm_set(void){
    if (!device_is_ready(pwm_vbmot.dev)){
        printk("PWM not initialized.\n");
        return -1;
    }
    
    uint32_t period_hz = 1000; // PWM frequency
    uint32_t pulse_ns = (1000000000 / period_hz) / 2; // 50% duty cycle
    
    while(1){
        pwm_set_dt(&pwm_vbmot, PWM_HZ(period_hz), pulse_ns);
        k_msleep(1000); 
    }

    return 0;
}