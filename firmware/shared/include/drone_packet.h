#ifndef DRONE_PACKET_H
#define DRONE_PACKET_H

#include <stdint.h>
#include <stdbool.h>

// Using the Mode 2 controller configuration
typedef struct __attribute__((packed)){
    // Send normalized floats (16 bytes total)
    float throttle; // 0.0f to 1.0f (Left Joystick Y)
    float pitch; // -1.0f to 1.0f (Right Joystick Y)
    float roll; // -1.0f to 1.0f (Right Stick X)
    float yaw; // -1.0f to 1.0f (Left Joystick X)

    uint8_t status; // 0 = Motors Off. 1 = Ready to fly
    uint8_t mode; // 0 = Angled, but self-levelling. 1 = Manual levelling needed

    uint16_t packet_id; // Wrap around at 65535, FC uses to detect dropped packets 

} data_packet; // Payload is 20 bytes, below the 32 bytes max limit for ESB

#endif // DRONE_PACKET_H