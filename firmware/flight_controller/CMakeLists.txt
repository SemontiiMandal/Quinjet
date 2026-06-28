cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(quinjet_drone)

# Added all FC source files
target_sources(app PRIVATE 
    src/main.c 
    src/radio_esb.c
    src/flight_control.c
    src/drivers/imu_bmi270.c
    src/drivers/motor_pwm.c
    src/modules/motor_mixer.c
    src/modules/pid.c
    src/modules/sensor_fusion.c
)

# Added shared include for the data packet
target_include_directories(app PRIVATE include ../shared/include)