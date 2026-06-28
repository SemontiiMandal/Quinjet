#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

K_SEM_DEFINE(imu_data_ready_sem, 0, 1);

static void bmi270_trigger_handler(const struct device *dev, const struct sensor_trigger *trig){
    k_sem_give(&imu_data_ready_sem); // Wakes the flight control thread
}

int bmi270_init(void){
    const struct device *bmi270 = DEVICE_DT_GET(DT_NODELABEL(bmi270));

    // Sstruct initialization
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL 
    };

    if (!device_is_ready(bmi270)) {
        printk("Device BMI270 is not ready.\n");
        return -1;
    }

    if (sensor_trigger_set(bmi270, &trig, bmi270_trigger_handler) < 0){
        printk("Failed to set trigger!\n");
        return -1;
    }

    printk("Trigger set successfully!\n");
    return 0; 
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