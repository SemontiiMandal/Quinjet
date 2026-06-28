#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include "drivers/analog_joystick.h"
#include "drivers/decode_joystick.h"
#include "rc_state_machine.h"
#include "radio_esb.h"

// memory allocation belongs in the source file
K_SEM_DEFINE(wake_esb, 0, 1);
K_MUTEX_DEFINE(joystick_data);
data_packet latest_joystick_state;

static const struct adc_dt_spec adc_1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0); // index starts at 0!
static const struct adc_dt_spec adc_2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec adc_3 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);
static const struct adc_dt_spec adc_4 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3);

joystick joystick1, joystick2;
static int16_t sample_buffer;

int joystick_init(void) {
    if (!adc_is_ready_dt(&adc_1) || !adc_is_ready_dt(&adc_2) || 
        !adc_is_ready_dt(&adc_3) || !adc_is_ready_dt(&adc_4)){
        printk("ADCs not ready yet!\n");
        return -1;
    }

    adc_channel_setup_dt(&adc_1);
    adc_channel_setup_dt(&adc_2);
    adc_channel_setup_dt(&adc_3);
    adc_channel_setup_dt(&adc_4);
    
    return 0;
}

// 100hz continuous thread
void adc_thread(void* arg1, void* arg2, void* arg3) {
    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
    };

    while(1) {
        // read hardware
        adc_sequence_init_dt(&adc_1, &sequence);
        adc_read(adc_1.dev, &sequence);
        joystick1.x = sample_buffer;

        adc_sequence_init_dt(&adc_2, &sequence);
        adc_read(adc_2.dev, &sequence);
        joystick1.y = sample_buffer;

        adc_sequence_init_dt(&adc_3, &sequence);
        adc_read(adc_3.dev, &sequence);
        joystick2.x = sample_buffer;

        adc_sequence_init_dt(&adc_4, &sequence); // fixed copy paste error here
        adc_read(adc_4.dev, &sequence);
        joystick2.y = sample_buffer;

        // normalize
        coordinate_normalized output_left = decode_joystick(joystick1.x, joystick1.y);
        coordinate_normalized output_right = decode_joystick(joystick2.x, joystick2.y);

        // check if user clicked the stick to arm
        if (rc_check_arm()) {
            if (current_rc_state == RC_STATE_IDLE && output_left.y <= -0.95f) { // throttle low gate
                current_rc_state = RC_STATE_ARMED;
                printk("ARMED!\n");
            } else if (current_rc_state == RC_STATE_ARMED && output_left.y <= -0.95f) {
                current_rc_state = RC_STATE_IDLE;
                printk("DISARMED!\n");
            }
        }

        // safely write to shared memory
        k_mutex_lock(&joystick_data, K_FOREVER);
        latest_joystick_state.yaw = output_left.x;
        latest_joystick_state.throttle = output_left.y; // note: may want to map -1 to 1 into 0 to 1 later!
        latest_joystick_state.roll = output_right.x;
        latest_joystick_state.pitch = output_right.y;
        latest_joystick_state.status = (uint8_t)current_rc_state;
        latest_joystick_state.mode = 0; 
        k_mutex_unlock(&joystick_data);

        // wake radio
        k_sem_give(&wake_esb);

        k_msleep(10); // 100hz loop
    }
}

// zephyr macro for threads
K_THREAD_DEFINE(adc_thread_id, 2048, adc_thread, NULL, NULL, NULL, 5, 0, 0);