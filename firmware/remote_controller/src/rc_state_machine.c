#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "rc_state_machine.h"

volatile bool pending_arm_request = false; 
rc_states current_rc_state = RC_STATE_IDLE;

// get pins from overlay
static const struct gpio_dt_spec dev1 = GPIO_DT_SPEC_GET(DT_NODELABEL(sel_1), gpios);
static const struct gpio_dt_spec dev2 = GPIO_DT_SPEC_GET(DT_NODELABEL(sel_2), gpios);

static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;

// fast isr
void button_press_handler(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins){
    pending_arm_request = true; // just flip the flag
}

int rc_state_init(void){
    if (!gpio_is_ready_dt(&dev1) || !gpio_is_ready_dt(&dev2)) return -1;

    gpio_pin_configure_dt(&dev1, GPIO_INPUT);
    gpio_pin_configure_dt(&dev2, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&dev1, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&dev2, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button1_cb_data, button_press_handler, BIT(dev1.pin));
    gpio_add_callback(dev1.port, &button1_cb_data);

    gpio_init_callback(&button2_cb_data, button_press_handler, BIT(dev2.pin));
    gpio_add_callback(dev2.port, &button2_cb_data);

    return 0;
}

bool rc_check_arm(void){
    if (pending_arm_request){
        pending_arm_request = false;
        return true;
    }
    return false;
}