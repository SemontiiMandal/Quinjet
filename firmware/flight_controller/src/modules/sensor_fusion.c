#include <stdio.h>
#include <math.h> // Required for atan2f and sqrtf
#include "modules/sensor_fusion.h"
#define DELTA_T 0.001f
#define ALPHA 0.98f

// Do some math to get pitch, yaw and roll from acc and gyro values
void sensor_fusion_compute(float ax, float ay, float az, 
                           float gx, float gy, float gz, 
                           euler_angles_t *angles, float dt) {
    
    float roll_ang  = atan2f(ay, sqrtf(ax * ax + az * az));
    float pitch_ang = atan2f(-ax, sqrtf(ay * ay + az * az));

    angles->roll  = ALPHA * (angles->roll + (gx * dt)) + ((1.0f - ALPHA) * roll_ang);
    angles->pitch = ALPHA * (angles->pitch + (gy * dt)) + ((1.0f - ALPHA) * pitch_ang);
    angles->yaw   = angles->yaw + (gz * dt);
}