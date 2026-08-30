#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <zephyr/drivers/sensor.h>
#include "flight_control.h"

void sensor_fusion_compute(float ax, float ay, float az, float gx, float gy, float gz, euler_angles_t *angles, float dt);

#endif