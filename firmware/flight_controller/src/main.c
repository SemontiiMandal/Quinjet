#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "radio_esb.h"
#include "drivers/motor_pwm.h"

// Expose the BMI init here as it's not a generic device call
extern int bmi270_init(void);

int main(void) {
    printk("\n--- Booting Quinjet FC ---\n");

    if (app_pwm_init() != 0) {
        printk("Failed to init PWM.\n");
    }

    if (bmi270_init() != 0) {
        printk("Failed to init IMU.\n");
    }

    if (radio_esb_init(false) != 0) { // false = PRX (Receiver)
        printk("Failed to boot radio.\n");
    }

    printk("System Ready. Flight Controller locked to IMU interrupts.\n");
    
    k_sleep(K_FOREVER);
    return 0;
}