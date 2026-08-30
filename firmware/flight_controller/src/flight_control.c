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
#include "modules/dynamic_notch.h"

#define RADIO_TIMEOUT_MS 500 

// Access the semaphore from the IMU driver
extern struct k_sem imu_data_ready_sem;

// This variable exists in radio_esb.c; declared and initialized there already, so use extern
extern int64_t last_packet_time; 

euler_angles_t current_angle = {0.0f, 0.0f, 0.0f};

// Making them static so they hold state across loops
static pid_controller pitch_pid;
static pid_controller roll_pid;
static pid_controller yaw_pid;

static dynamic_notch_t pitch_notch;
static dynamic_notch_t roll_notch;

void flight_controller_thread(void* p1, void* p2, void* p3){
    int64_t local_last_packet_time = 0;
    const struct device *bmi270 = DEVICE_DT_GET(DT_NODELABEL(bmi270));
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    data_packet local_rc_command;

    // Initialize PIDs.
    // Update to PID functions - Added 1.0f for the new Kb (anti-windup) parameter

    // Arguments in this order: p, i, d, b, i_limit, out_limit, cutoff_hz, sample_hz
    pid_init(&pitch_pid, 1.5f, 0.01f, 0.5f, 1.0f, 100.0f, 10000.0f, 30.0f, 1000.0f);
    pid_init(&roll_pid,  1.5f, 0.01f, 0.5f, 1.0f, 100.0f, 10000.0f, 30.0f, 1000.0f);
    pid_init(&yaw_pid,   1.5f, 0.01f, 0.5f, 1.0f, 100.0f, 10000.0f, 30.0f, 1000.0f); 

    // Initialize Dynamic Notch Filters for Gyro Channels to strip out motor vibration frequency
    dynamic_notch_init(&pitch_notch);
    dynamic_notch_init(&roll_notch);

    while(1){
        // Runs when IMU triggers data ready interrupt and gives semaphore to this fc thread
        k_sem_take(&imu_data_ready_sem, K_FOREVER);

        // get values from IMU
        sensor_sample_fetch(bmi270);
        sensor_channel_get(bmi270, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(bmi270, SENSOR_CHAN_GYRO_XYZ, gyro);

        // Convert Accel and Gyro to hardware floats because we do floating point math later 

        /*
            The nRF52840 uses an ARM Cortex-M4F processor ('F' stands for a hardware Floating-Point Unit (FPU)), making floating-point math extremely fast. Multiplying two floats takes 1 to 3 clock cycles, so practically as fast as integer math.

            It is single precision FPU, so need 32 bit float; Can't support 64 bit double.
            
        */
        float ax = sensor_value_to_double(&accel[0]);
        float ay = sensor_value_to_double(&accel[1]);
        float az = sensor_value_to_double(&accel[2]);
        
        float raw_gx = sensor_value_to_double(&gyro[0]);
        float raw_gy = sensor_value_to_double(&gyro[1]);
        float raw_gz = sensor_value_to_double(&gyro[2]);

        // Apply Dynamic Notch Filter to strip motor resonance
        float clean_gx = dynamic_notch_process(&roll_notch, raw_gx);
        float clean_gy = dynamic_notch_process(&pitch_notch, raw_gy);

        // Compute current orientation of drone
        sensor_fusion_compute(ax, ay, az, clean_gx, clean_gy, raw_gz, &current_angle, 0.001f);

        // Access shared memory
        // Get latest joystick positions from the radio thread
        k_spinlock_key_t key = k_spin_lock(&rc_spinlock);
        memcpy(&local_rc_command, &latest_rc_command, sizeof(data_packet));
        local_last_packet_time = last_packet_time; // Pull the timestamp
        k_spin_unlock(&rc_spinlock, key);

        // Failsafe - Did radio timeout, somehow rc disconnected, out of range etc?
        if ((k_uptime_get() - local_last_packet_time) > RADIO_TIMEOUT_MS) {
            motor_outputs_t kill_output = {0, 0, 0, 0};
            app_pwm_set(&kill_output);
            continue; // Skip PID math 
        }

        // Update PID loops 
        float pitch_correction = pid_update(&pitch_pid, local_rc_command.pitch, current_angle.pitch, 0.001f);
        float roll_correction  = pid_update(&roll_pid,  local_rc_command.roll,  current_angle.roll,  0.001f);
        float yaw_correction   = pid_update(&yaw_pid,   local_rc_command.yaw,   current_angle.yaw,   0.001f);

        // Mix outputs and send to motors
        motor_outputs_t output = mix_motors(local_rc_command.throttle, pitch_correction, roll_correction, yaw_correction, local_rc_command.status);
        app_pwm_set(&output);   
    }
}

K_THREAD_DEFINE(fc_thread_id, 2048, flight_controller_thread, NULL, NULL, NULL, -1, 0, 0);

/*
A preemptive thread can be interrupted by the OS scheduler if a higher-priority task wakes up. If FC thread is halfway through calculating the PID loop and kernel pauses it to run the ESB thread, motor updates will be delayed. This causes jitter and messes up the DSP pipeline.
*/