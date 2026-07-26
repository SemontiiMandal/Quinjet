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
#include "modules/pid.h"
#include "modules/dynamic_notch.h"

// Access the semaphore from the IMU driver
extern struct k_sem imu_data_ready_sem;

euler_angles_t current_angle = {0.0f, 0.0f, 0.0f};

// Allocate PIDs globally or static so they hold state across loops
static pid_controller pitch_pid;
static pid_controller roll_pid;
static pid_controller yaw_pid;

static dynamic_notch_t pitch_notch;
static dynamic_notch_t roll_notch;

void flight_controller_thread(void* p1, void* p2, void* p3){
    
    const struct device *bmi270 = DEVICE_DT_GET(DT_NODELABEL(bmi270));
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    data_packet local_rc_command;

    // Initialize PIDs with LPF Cutoff = 30Hz, Loop Rate = 1000Hz
    pid_init(&pitch_pid, 1.5f, 0.01f, 0.5f, 100.0f, 10000.0f, 30.0f, 1000.0f);
    pid_init(&roll_pid,  1.5f, 0.01f, 0.5f, 100.0f, 10000.0f, 30.0f, 1000.0f);
    pid_init(&yaw_pid,   1.5f, 0.01f, 0.5f, 100.0f, 10000.0f, 30.0f, 1000.0f); 

    // Initialize Dynamic Notch Filters for Gyro Channels
    dynamic_notch_init(&pitch_notch);
    dynamic_notch_init(&roll_notch);

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

            // Convert Gyro to float rad/s
            float raw_gx = sensor_value_to_double(&gyro[0]);
            float raw_gy = sensor_value_to_double(&gyro[1]);

            // Only printing X values for now, might change later
            printk("Accel X: %d.%06d m/s^2\n", accel[0].val1, accel[0].val2);
            printk("Gyro X: %d.%06d rad/s\n", gyro[0].val1, gyro[0].val2);

            // Apply Dynamic Notch Filter to strip motor resonance
            float clean_gx = dynamic_notch_process(&roll_notch, raw_gx);
            float clean_gy = dynamic_notch_process(&pitch_notch, raw_gy);

            // Convert floats back into the Zephyr sensor_value structs
            gyro[0].val1 = (int32_t)clean_gx;
            gyro[0].val2 = (int32_t)((clean_gx - (int32_t)clean_gx) * 1000000);
        
            gyro[1].val1 = (int32_t)clean_gy;
            gyro[1].val2 = (int32_t)((clean_gy - (int32_t)clean_gy) * 1000000);

            // Compute current orientation (Euler Angles)
            sensor_fusion_compute(accel, gyro, &current_angle);

            // Get the latest stick positions from the radio thread
            k_spinlock_key_t key = k_spin_lock(&rc_spinlock);
            memcpy(&local_rc_command, &latest_rc_command, sizeof(data_packet));
            k_spin_unlock(&rc_spinlock, key);

            //  Update PID loops (D-term is automatically LPF-filtered inside pid_update)
            float pitch_correction = pid_update(&pitch_pid, local_rc_command.pitch, current_angle.pitch, 0.001f);
            float roll_correction  = pid_update(&roll_pid,  local_rc_command.roll,  current_angle.roll,  0.001f);
            float yaw_correction   = pid_update(&yaw_pid,   local_rc_command.yaw,   current_angle.yaw,   0.001f);

            // Calculate outputs after running computations with current throttle requested and PID corrections

            motor_outputs_t output = mix_motors(local_rc_command.throttle, pitch_correction, roll_correction, yaw_correction, local_rc_command.status);

            // Send the duty cycle to motors
            app_pwm_set(&output);   
            
            // k_msleep(1) not needed asthe thread naturally sleeps via the semaphore

    }
}

// Spawns the thread dynamically in the RTOS background
K_THREAD_DEFINE(fc_thread_id, 2048, flight_controller_thread, NULL, NULL, NULL, -1, 0, 0); // -1 priority = highly critical, and cooperative (runs interrupted until it itself yields voluntarily to the scheduler, no time-slicing)