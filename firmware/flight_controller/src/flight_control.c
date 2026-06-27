#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

euler_angles_t current_angle = {0.0f, 0.0f, 0.0f};

void flight_controller_thread(void* p1, void* p2, void* p3){

    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    while(1){
        // Kept printk-s for debugging; Will comment out during actual flight

        k_sem_take(&imu_data_ready_sem, K_FOREVER);

            printk("BMI270 data arrived, fetching now...\n");

            // Fetch sensor samples 
            sensor_sample_fetch(bmi270);

            // Get Accelerometer Data
            sensor_channel_get(bmi270, SENSOR_CHAN_ACCEL_XYZ, accel);
            
            // Get Gyroscope Data
            sensor_channel_get(bmi270, SENSOR_CHAN_GYRO_XYZ, gyro);

            // Only printing X values for now, might change later
            printk("Accel X: %d.%06d m/s^2\n", accel[0].val1, accel[0].val2);
            printk("Gyro X: %d.%06d rad/s\n", gyro[0].val1, gyro[0].val2);

            // Compute current orientation (Euler Angles)
            sensor_fusion_compute(accel, gyro, &current_angle);

            // Get the latest stick positions from the radio thread
            radio_triple_buffer(&remote_commands);

            // Run the PID Controller
            pid_control(&current_angle, &remote_commands, &pid_corrections);

            // Calculate outputs after running computations with current throttle requested and PID corrections
            motor_calculate(remote_commands.throttle, &pid_corrections, &motor_powers);

            // Send the duty cycle to motors
            pwm_write_channels(&motor_powers);            

    }
}
