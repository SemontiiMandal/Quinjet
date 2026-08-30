#ifndef DYNAMIC_NOTCH_H
#define DYNAMIC_NOTCH_H

#include <arm_math.h>

#define FFT_SIZE 64 // 64 samples at 1000Hz = ~15.6Hz bin resolution

#define M_PI 3.14159265358979323846f

typedef struct {
    arm_rfft_fast_instance_f32 fft_instance;
    float sample_buffer[FFT_SIZE];
    float fft_output[FFT_SIZE];
    float fft_mag[FFT_SIZE / 2];
    float window[FFT_SIZE];
    uint16_t buffer_index;

    // Notch Filter State
    arm_biquad_cascade_df2T_instance_f32 notch_filter;
    float notch_coeffs[5];
    float notch_state[2];
    float current_notch_freq;
} dynamic_notch_t;

void dynamic_notch_init(dynamic_notch_t *dn);
float dynamic_notch_process(dynamic_notch_t *dn, float in_sample);

#endif