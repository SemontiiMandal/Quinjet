# ifndef ANALOG_JOYSTICK_H
# define ANALOG_JOYSTICK_H

// C struct defining measurement parameters for ADC samples
int16_t sample_buffer;

K_SEM_DEFINE(wake_esb, 0, 1);

struct adc_sequence sequence = {
    .buffer = &sample_buffer;
    .buffer_size = sizeof(sample_buffer);
}

typedef struct {
 uint16_t x;
 uint16_t y;
} joystick;

#define RING_BUF_SIZE 128
uint8_t ring_buffer_mem[RING_BUFF_SIZE];
struct ring_buf ringbuf;

# endif