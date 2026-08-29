#include <string.h>
#include <math.h>
#include "modules/dynamic_notch.h"

#define SAMPLE_RATE 1000.0f
#define MIN_SEARCH_HZ 100.0f
#define MAX_SEARCH_HZ 400.0f
#define NOTCH_Q 3.0f // Quality factor (higher = narrower notch)

static void update_notch_coeffs(dynamic_notch_t *dn, float center_freq) {
    if (center_freq < MIN_SEARCH_HZ) center_freq = MIN_SEARCH_HZ;
    if (center_freq > MAX_SEARCH_HZ) center_freq = MAX_SEARCH_HZ;

    float omega = 2.0f * M_PI * center_freq / SAMPLE_RATE;
    float alpha = sinf(omega) / (2.0f * NOTCH_Q);
    float cs = cosf(omega);

    float b0 = 1.0f;
    float b1 = -2.0f * cs;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;

    dn->notch_coeffs[0] = b0 / a0;
    dn->notch_coeffs[1] = b1 / a0;
    dn->notch_coeffs[2] = b2 / a0;
    dn->notch_coeffs[3] = -a1 / a0;
    dn->notch_coeffs[4] = -a2 / a0;

    dn->current_notch_freq = center_freq;
    arm_biquad_cascade_df2T_init_f32(&dn->notch_filter, 1, dn->notch_coeffs, dn->notch_state);
}

void dynamic_notch_init(dynamic_notch_t *dn) {
    dn->buffer_index = 0;
    memset(dn->sample_buffer, 0, sizeof(dn->sample_buffer));
    memset(dn->notch_state, 0, sizeof(dn->notch_state));

    arm_rfft_fast_init_f32(&dn->fft_instance, FFT_SIZE);
    update_notch_coeffs(dn, 200.0f); // Default initial notch at 200Hz
}
/*
static void update_fft_and_recalculate_notch(dynamic_notch_t *dn) {
    // Run FFT using CMSIS-DSP
    arm_rfft_fast_f32(&dn->fft_instance, dn->sample_buffer, dn->fft_output, 0);

    // Compute magnitudes of complex bins
    arm_cmplx_mag_f32(dn->fft_output, dn->fft_mag, FFT_SIZE / 2);

    // Search for peak frequency in motor noise band (100Hz - 400Hz)
    uint32_t min_bin = (uint32_t)(MIN_SEARCH_HZ / (SAMPLE_RATE / FFT_SIZE));
    uint32_t max_bin = (uint32_t)(MAX_SEARCH_HZ / (SAMPLE_RATE / FFT_SIZE));

    float max_val = 0.0f;
    uint32_t max_index = min_bin;

    for (uint32_t i = min_bin; i <= max_bin; i++) {
        if (dn->fft_mag[i] > max_val) {
            max_val = dn->fft_mag[i];
            max_index = i;
        }
    }

    float detected_peak_hz = (float)max_index * (SAMPLE_RATE / (float)FFT_SIZE);

    // Update Notch Filter Coefficients if peak shifted significantly (> 10Hz)
    if (fabsf(detected_peak_hz - dn->current_notch_freq) > 10.0f) {
        update_notch_coeffs(dn, detected_peak_hz);
    }
}
*/

// Add this to dynamic_notch_t struct: float window[FFT_SIZE];
// Initialize it in dynamic_notch_init() using:
// for(int i=0; i<FFT_SIZE; i++) dn->window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));

static void update_fft_and_recalculate_notch(dynamic_notch_t *dn) {
    float windowed_buffer[FFT_SIZE];
    
    // 1. Apply Hann Window
    arm_mult_f32(dn->sample_buffer, dn->window, windowed_buffer, FFT_SIZE);

    // 2. Run FFT
    arm_rfft_fast_f32(&dn->fft_instance, windowed_buffer, dn->fft_output, 0);

    // 3. Handle CMSIS-DSP Packed Output (Fixing the bug)
    // dn->fft_output[0] is DC, dn->fft_output[1] is Nyquist
    dn->fft_mag[0] = fabsf(dn->fft_output[0]); // DC

    for (uint32_t k = 1; k < FFT_SIZE / 2; k++) {
        float real = dn->fft_output[2 * k];
        float imag = dn->fft_output[2 * k + 1];
        dn->fft_mag[k] = sqrtf(real * real + imag * imag);
    }

    // 4. Search for peak in motor noise band (100Hz - 400Hz)
    uint32_t min_bin = (uint32_t)(MIN_SEARCH_HZ / (SAMPLE_RATE / FFT_SIZE));
    uint32_t max_bin = (uint32_t)(MAX_SEARCH_HZ / (SAMPLE_RATE / FFT_SIZE));

    float max_val = 0.0f;
    uint32_t max_index = min_bin;

    for (uint32_t i = min_bin; i <= max_bin; i++) {
        if (dn->fft_mag[i] > max_val) {
            max_val = dn->fft_mag[i];
            max_index = i;
        }
    }

    float detected_peak_hz = (float)max_index * (SAMPLE_RATE / (float)FFT_SIZE);

    if (fabsf(detected_peak_hz - dn->current_notch_freq) > 10.0f) {
        update_notch_coeffs(dn, detected_peak_hz);
    }
}

float dynamic_notch_process(dynamic_notch_t *dn, float in_sample) {
    // Store sample in circular FFT buffer
    dn->sample_buffer[dn->buffer_index++] = in_sample;

    // When buffer fills (every 64ms at 1000Hz), run FFT update
    if (dn->buffer_index >= FFT_SIZE) {
        dn->buffer_index = 0;
        update_fft_and_recalculate_notch(dn);
    }

    // Pass incoming sample through the notch filter
    float out_sample = 0.0f;
    arm_biquad_cascade_df2T_f32(&dn->notch_filter, &in_sample, &out_sample, 1);

    return out_sample;
}