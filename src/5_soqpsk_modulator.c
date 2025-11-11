#include "soqpsk.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define M_PI 3.14159265358979323846
#define FLOAT_EPSILON 1e-10

SOQPSK_Modulator* SOQPSK_Modulator_Create(float carrier_freq, float sample_rate, 
                                          int samples_per_symbol)
{
    SOQPSK_Modulator *mod = malloc(sizeof(SOQPSK_Modulator));
    if (!mod) return NULL;
    
    mod->carrier_freq = carrier_freq;
    mod->sample_rate = sample_rate;
    mod->samples_per_symbol = samples_per_symbol;
    mod->phase_accum = 0.0f;
    
    mod->pulse_length = 8 * samples_per_symbol;
    mod->frequency_pulse = malloc(mod->pulse_length * sizeof(float));
    
    float Ts = samples_per_symbol / sample_rate;
    create_frequency_pulse(mod->frequency_pulse, mod->pulse_length, Ts);
    
    return mod;
}

void SOQPSK_Modulator_Destroy(SOQPSK_Modulator *mod)
{
    if (mod) {
        free(mod->frequency_pulse);
        free(mod);
    }
}

void differential_precoder(const uint8_t *input_bits, int8_t *output_ternary, int nbits)
{
    if (!input_bits || !output_ternary) return;
    
    int8_t *d = malloc(nbits * sizeof(int8_t));
    for (int i = 0; i < nbits; i++) {
        d[i] = (input_bits[i] == 0) ? -1 : 1;
    }
    
    int8_t d_prev1 = 1;
    int8_t d_prev2 = 1;
    
    for (int i = 0; i < nbits; i++) {
        int8_t d_i = d[i];
        int8_t d_i_minus_2 = (i >= 2) ? d[i-2] : d_prev2;
        int8_t d_i_minus_1 = (i >= 1) ? d[i-1] : d_prev1;
        
        int sign = ((i + 1) % 2 == 0) ? 1 : -1;
        int8_t diff = d_i - d_i_minus_2;
        
        if (diff == 0) {
            output_ternary[i] = 0;
        } else if (diff > 0) {
            output_ternary[i] = sign * d_i_minus_1;
        } else {
            output_ternary[i] = -sign * d_i_minus_1;
        }
    }
    
    free(d);
}

void create_frequency_pulse(float *pulse, int length, float Ts)
{
    if (!pulse) return;
    
    float rho = SOQPSK_RHO;
    float B = SOQPSK_B;
    
    int center = length / 2;
    
    for (int i = 0; i < length; i++) {
        float t = (float)(i - center) * Ts / length;
        
        float numerator_arg = M_PI * rho * B * t / Ts;
        float numerator = cosf(numerator_arg) * sinf(M_PI * B * t / Ts);
        
        float denom_arg = M_PI * B * t / Ts;
        float denom = (1.0f - 4.0f * (rho * B * t / Ts) * (rho * B * t / Ts)) * denom_arg;
        
        if (fabsf(denom) > FLOAT_EPSILON) {
            pulse[i] = numerator / denom;
        } else {
            pulse[i] = 0.0f;
        }
    }
}

void SOQPSK_Modulate(SOQPSK_Modulator *mod, const uint8_t *input_bits, 
                     int num_bits, float complex *output_signal)
{
    if (!mod || !input_bits || !output_signal) return;
    
    int8_t *ternary_data = malloc(num_bits * sizeof(int8_t));
    differential_precoder(input_bits, ternary_data, num_bits);
    
    int output_length = num_bits * mod->samples_per_symbol;
    float *freq_impulses = calloc(output_length, sizeof(float));
    
    for (int i = 0; i < num_bits; i++) {
        freq_impulses[i * mod->samples_per_symbol] = (float)ternary_data[i];
    }
    
    float *instantaneous_freq = calloc(output_length, sizeof(float));
    
    for (int i = 0; i < output_length; i++) {
        for (int j = 0; j < mod->pulse_length; j++) {
            int idx = i - mod->pulse_length/2 + j;
            if (idx >= 0 && idx < output_length) {
                instantaneous_freq[idx] += 
                    freq_impulses[i] * mod->frequency_pulse[j];
            }
        }
    }
    
    float *phase = malloc(output_length * sizeof(float));
    phase = mod->phase_accum;
    
    for (int i = 1; i < output_length; i++) {
        phase[i] = phase[i-1] + 
                   2.0f * M_PI * instantaneous_freq[i] / mod->sample_rate;
    }
    
    mod->phase_accum = phase[output_length - 1];
    
    for (int i = 0; i < output_length; i++) {
        float t = (float)i / mod->sample_rate;
        float carrier_phase = 2.0f * M_PI * mod->carrier_freq * t;
        float total_phase = carrier_phase + phase[i];
        
        float I = cosf(total_phase);
        float Q = sinf(total_phase);
        output_signal[i] = I + Q * _Complex_I;
    }
    
    free(ternary_data);
    free(freq_impulses);
    free(instantaneous_freq);
    free(phase);
}
