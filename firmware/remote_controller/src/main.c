#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "radio_esb.h"
#include "rc_state_machine.h"
#include "drivers/analog_joystick.h"

int main(void) {
    printk("\n--- Booting Quinjet RC ---\n");

    if (rc_state_init() != 0) {
        printk("Failed to init buttons.\n");
    }

    if (joystick_init() != 0) {
        printk("Failed to init ADC.\n");
    }

    if (radio_esb_init(true) != 0) { // true = PTX
        printk("Failed to boot radio.\n");
    }

    printk("System Ready. Threads running in background.\n");
    
    // Main thread can just sleep forever now, the RTOS threads are doing the work
    k_sleep(K_FOREVER);
    return 0;
}