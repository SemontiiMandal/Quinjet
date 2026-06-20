#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

int main(void) {
    const struct device *const bmi270 = DEVICE_DT_GET(DT_NODELABEL(bmi270));
    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    if (!device_is_ready(bmi270)) {
        printk("Device BMI270 is not ready.\n");
        return 0;
    }

    printk("BMI270 found, fetching data...\n");

    while (1) {
        // Fetch sensor samples 
        sensor_sample_fetch(bmi270);

        // Get Accelerometer Data
        sensor_channel_get(bmi270, SENSOR_CHAN_ACCEL_XYZ, accel);
        
        // Get Gyroscope Data
        sensor_channel_get(bmi270, SENSOR_CHAN_GYRO_XYZ, gyro);

        printk("Accel X: %d.%06d m/s^2\n", accel[0].val1, accel[0].val2);
        printk("Gyro X: %d.%06d rad/s\n", gyro[0].val1, gyro[0].val2);

        k_sleep(K_MSEC(100));
    }
}
