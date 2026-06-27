#include <stdio.h>
// Assuming delta_t is fixed (e.g., 0.001 seconds for a 1kHz loop)
#define DELTA_T 0.001f
#define ALPHA 0.98f

void sensor_fusion_compute(struct sensor_value *accel, struct sensor_value *gyro, euler_angles_t *angles) {
    
    // Convert Zephyr's fixed-point structs (val1 int and val 2 decimal part but given as int) to standard floats (rad/s and m/s^2)
    // Returns double, which we're implicitly typecasted to float as nRF's Cortex-M4F processor has a 32-bit Hardware FPU
    float ax = sensor_value_to_double(&accel[0]);
    float ay = sensor_value_to_double(&accel[1]);
    float az = sensor_value_to_double(&accel[2]);
    
    float gx = sensor_value_to_double(&gyro[0]);
    float gy = sensor_value_to_double(&gyro[1]);
    float gz = sensor_value_to_double(&gyro[2]);

    // Calculate raw pitch and roll angles using trigonometry (in radians)
    // atan2 is highly optimized in standard math libraries
    float roll_ang  = atan2f(ay, sqrtf(ax * ax + az * az));
    float pitch_ang = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Apply the Complementary Filter
    angles->roll  = ALPHA * (angles->roll + (gx * DELTA_T)) + ((1.0f - ALPHA) * roll_ang);
    angles->pitch = ALPHA * (angles->pitch + (gy * DELTA_T)) + ((1.0f - ALPHA) * pitch_ang);
    
    // Yaw is purely Gyro Integration (No accelerometer reference)
    angles->yaw = angles->yaw + (gz * DELTA_T);
}