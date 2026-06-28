cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(quinjet_drone)

# Added all new source files after changed architecture
target_sources(app PRIVATE 
    src/main.c 
    src/radio_esb.c
    src/rc_state_machine.c
    src/modules/decode_joystick.c
    src/drivers/analog_joystick.c
    src/drivers/motor_pwm.c
)

# Tell CMake to look in the local include folder
target_include_directories(app PRIVATE include)

# Tell CMake to look in the shared include folder
target_include_directories(app PRIVATE ../shared/include)