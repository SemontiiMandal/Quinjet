#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <hal/nrf_saadc.h>

// Fetch the ADC channel from overlay

// X1
static const struct adc_dt_spec adc_1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user),1);
// Y1
static const struct adc_dt_spec adc_2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user),2);
// X2
static const struct adc_dt_spec adc_3 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user),3);
// Y2
static const struct adc_dt_spec adc_4 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user),4);

joystick joystick1, joystick2;

static int joystick_init(){

    if (!adc_is_ready_dt(adc_1) | !adc_is_ready_dt(adc_2) | !adc_is_ready_dt(adc_3) | !adc_is_ready_dt(adc_4)){
        printk("ADCs not ready yet! \n");
        return -1;
    }

    // configure adc channels
    int err = adc_channel_setup_dt(&adc_1);
    if (err < 0){
        printk("Could not setup channel 1; Error Code %d.\n", err);
    }

    err = adc_channel_setup_dt(&adc_2);
    if (err < 0){
        printk("Could not setup channel 2; Error Code %d.\n", err);
    }

    err = adc_channel_setup_dt(&adc_3);
    if (err < 0){
        printk("Could not setup channel 3; Error Code %d.\n", err);
    }

    err = adc_channel_setup_dt(&adc_4);
    if (err < 0){
        printk("Could not setup channel 4; Error Code %d.\n", err);
    }

}

static int read_data_thread (){
    err = adc_sequence_init_dt(&adc_1, &sequence);

    if (err < 0){
        printk ("Sequence not initialized (%d)\n", err);
        continue;
    }

    err = adc_read(adc_1.dev, &sequence);
    int32_t val_mv = 0;

    if (err < 0){
        printk("ADC read failed; Error code %d \n", err);
    }
    else{
        // Raw conversion value
        val_mv = sample_buffer;
        joystick1.x = sample_buffer;
        // Convert raw value to millivolts
        adc_raw_to_millivolts_dt(&adc_1, &val_mv);
        printk("Raw Value (X1): %d | Voltage: %d mV\n", sample_buffer, val_mv);
    }

    err = adc_read(adc_2.dev, &sequence);

    if (err < 0){
        printk("ADC read failed; Error code %d \n", err);
    }
    else{
        // Raw conversion value
        val_mv = sample_buffer;
        joystick1.y = sample_buffer;
        // Convert raw value to millivolts
        adc_raw_to_millivolts_dt(&adc_2, &val_mv);
        printk("Raw Value (Y1): %d | Voltage: %d mV\n", sample_buffer, val_mv);
    }

    err = adc_read(adc_3.dev, &sequence);

    if (err < 0){
        printk("ADC read failed; Error code %d \n", err);
    }
    else{
        // Raw conversion value
        val_mv = sample_buffer;
        joystick2.x = sample_buffer;
        // Convert raw value to millivolts
        adc_raw_to_millivolts_dt(&adc_3, &val_mv);
        printk("Raw Value (X2): %d | Voltage: %d mV\n", sample_buffer, val_mv);
    }

    err = adc_read(adc_3.dev, &sequence);

    if (err < 0){
        printk("ADC read failed; Error code %d \n", err);
    }
    else{
        // Raw conversion value
        val_mv = sample_buffer;
        joystick2.x = sample_buffer;
        // Convert raw value to millivolts
        adc_raw_to_millivolts_dt(&adc_4, &val_mv);
        printk("Raw Value (Y2): %d | Voltage: %d mV\n", sample_buffer, val_mv);
    }

    joystick combined[2] = {joystick1, joystick2};

    uint32_t write = ring_buf_put(&ringbuf, combined, sizeof(combined));

    k_sem_give(&wake_eb);

    k_msleep(1000);
    return 0;

}