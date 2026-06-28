#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <zephyr/drivers/sensor.h>
#include "flight_control.h"

void sensor_fusion_compute(struct sensor_value *accel, struct sensor_value *gyro, euler_angles_t *angles);

#endif