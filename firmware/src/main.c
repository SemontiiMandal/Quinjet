#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// Import our custom radio module
#include "radio_esb.h"

// Toggle this between flashes
#define I_AM_THE_TRANSMITTER true

test_payload_t my_data = { .packet_id = 0, .test_value = 3.14f };

int main(void) {
    printk("\n--- Starting Modular ESB Test ---\n");

    // Initialize the radio hardware
    if (radio_esb_init(I_AM_THE_TRANSMITTER) != 0) {
        printk("Failed to boot radio module.\n");
        return -1;
    }

    while (1) {
        if (I_AM_THE_TRANSMITTER) {
            my_data.packet_id++;
            my_data.test_value += 0.1f;
            
            // Send data using the clean public API
            radio_esb_send_packet((uint8_t*)&my_data, sizeof(test_payload_t));
            
            k_sleep(K_MSEC(10)); 
        } else {
            // Sleep and let the radio interrupt handle the printing
            k_sleep(K_FOREVER);
        }
    }
    return 0;
}