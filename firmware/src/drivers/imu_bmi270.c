#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

K_SEM_DEFINE(imu_data_ready_sem, 0, 1);

static void bmi270_trigger_handler(const struct device *dev, struct sensor_trigger *trig){
    k_sem_give(&imu_data_ready_sem);
}

static void bmi270_init(){
    const struct device *const bmi270 = DEVICE_DT_GET(DT_NODELABEL(bmi270));

     struct sensor_trigger trig{
        .type = SENSOR_TRIG_DATA_READY;
        .chan = SENSOR_CHAN_ALL; 
     }

    if (!device_is_ready(bmi270)) {
        printk("Device BMI270 is not ready.\n");
        return 0;
    }

    if (!bmi270_init_interrupts(bmi270)){
        printk("Interrupt Trigger Enabled for BMI270. \n")
    }   

     if (!bmi270_trigger_set(dev, &trig, bmi270_trigger_handler)){
         printf("Trigger set successfully!\n");
     }

    while (1) {
      k_sleep_ms(K_FOREVER);
    }
}

/*
DO NOT READ INSIDE THE CALLBACK; GIVE SEMAPHORE TO FLIGHT CONTROL THREAD AND EXIT (DONE ABOVE).

static void bmi270_trigger_handler(const struct device *dev, struct sensor_trigger *trig){
    struct sensor_value val;

    if (trig->type == SENSOR_TRIG_DATA_READY){

         printk("BMI270 data arrived, fetching now...\n");

        // Fetch sensor samples 
        sensor_sample_fetch(bmi270);

        // Get Accelerometer Data
        sensor_channel_get(bmi270, SENSOR_CHAN_ACCEL_XYZ, accel);
        
        // Get Gyroscope Data
        sensor_channel_get(bmi270, SENSOR_CHAN_GYRO_XYZ, gyro);

        printk("Accel X: %d.%06d m/s^2\n", accel.val1, accel.val2);
        printk("Gyro X: %d.%06d rad/s\n", gyro.val1, gyro.val2);
    }

}

*/