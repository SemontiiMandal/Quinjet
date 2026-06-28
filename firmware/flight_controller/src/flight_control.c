#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "flight_control.h"
#include "radio_esb.h"
#include "modules/pid.h"
#include "modules/sensor_fusion.h"
#include "modules/motor_mixer.h"
#include "drivers/motor_pwm.h"

// Access the semaphore from the IMU driver
extern struct k_sem imu_data_ready_sem;

euler_angles_t current_angle = {0.0f, 0.0f, 0.0f};

// Allocate PIDs globally or static so they hold state across loops
static pid_controller pitch_pid;
static pid_controller roll_pid;
static pid_controller yaw_pid;

void flight_controller_thread(void* p1, void* p2, void* p3){
    
    const struct device *bmi270 = DEVICE_DT_GET(DT_NODELABEL(bmi270));
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    data_packet local_rc_command;

    // Initialize PIDs ONCE before the flight loop begins
    // Kp=1.5, Ki=0.01, Kd=0.5, I-limit=100.0, Output-limit=10000.0 (PWM range for direct drive)
    pid_init(&pitch_pid, 1.5f, 0.01f, 0.5f, 100.0f, 10000.0f); 
    pid_init(&roll_pid,  1.5f, 0.01f, 0.5f, 100.0f, 10000.0f); 
    pid_init(&yaw_pid,   1.5f, 0.01f, 0.5f, 100.0f, 10000.0f); 

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
            k_spinlock_key_t key = k_spin_lock(&rc_spinlock);
            memcpy(&local_rc_command, &latest_rc_command, sizeof(data_packet));
            k_spin_unlock(&rc_spinlock, key);

            // Run the PID Controller

            float pitch_correction = pid_update(&pitch_pid, local_rc_command.pitch, current_angle.pitch, 0.001f);
            float roll_correction  = pid_update(&roll_pid, local_rc_command.roll, current_angle.roll, 0.001f);
            float yaw_correction   = pid_update(&yaw_pid, local_rc_command.yaw, current_angle.yaw, 0.001f);

            // Calculate outputs after running computations with current throttle requested and PID corrections

            motor_outputs_t output = mix_motors(local_rc_command.throttle, pitch_correction, roll_correction, yaw_correction, local_rc_command.status);

            // Send the duty cycle to motors
            app_pwm_set(&output);   
            
            // k_msleep(1) not needed asthe thread naturally sleeps via the semaphore

    }
}

// Spawns the thread dynamically in the RTOS background
K_THREAD_DEFINE(fc_thread_id, 2048, flight_controller_thread, NULL, NULL, NULL, -1, 0, 0); // -1 priority = highly critical, and cooperative (runs interrupted until it itself yields voluntarily to the scheduler, no time-slicing)