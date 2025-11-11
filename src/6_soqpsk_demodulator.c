#include "soqpsk.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define M_PI 3.14159265358979323846

SOQPSK_Demodulator* SOQPSK_Demodulator_Create(float carrier_freq, float sample_rate,
                                              int samples_per_symbol)
{
    SOQPSK_Demodulator *demod = malloc(sizeof(SOQPSK_Demodulator));
    if (!demod) return NULL;
    
    demod->carrier_freq = carrier_freq;
    demod->sample_rate = sample_rate;
    demod->samples_per_symbol = samples_per_symbol;
    
    float symbol_rate = sample_rate / samples_per_symbol;
    demod->loop_bw = symbol_rate * 0.01f;
    demod->damping = 0.707f;
    demod->pll_phase = 0.0f;
    demod->pll_freq = 0.0f;
    
    demod->timing_mu = 0.0f;
    demod->timing_error = 0.0f;
    
    demod->current_state = 0;
    for (int i = 0; i < 8; i++) {
        demod->path_metrics[i] = 1e9f;
    }
    demod->path_metrics = 0.0f;
    
    return demod;
}

void SOQPSK_Demodulator_Destroy(SOQPSK_Demodulator *demod)
{
    if (demod) free(demod);
}

void carrier_recovery_pll(const float complex *received_signal, int length,
                          SOQPSK_Demodulator *demod, float complex *recovered_carrier)
{
    if (!demod) return;
    
    float Kp = 4.0f * demod->damping * demod->loop_bw / demod->sample_rate;
    float Ki = 4.0f * (demod->loop_bw / demod->sample_rate) * 
               (demod->loop_bw / demod->sample_rate);
    
    for (int i = 0; i < length; i++) {
        float cos_phase = cosf(demod->pll_phase);
        float sin_phase = sinf(demod->pll_phase);
        float complex local_osc = cos_phase + sin_phase * _Complex_I;
        
        float complex mixed = received_signal[i] * conj(local_osc);
        
        float error = cimagf(mixed) * (crealf(mixed) > 0 ? 1.0f : -1.0f);
        
        demod->pll_freq += Ki * error;
        demod->pll_phase += Kp * error + demod->pll_freq;
        
        while (demod->pll_phase > M_PI) {
            demod->pll_phase -= 2.0f * M_PI;
        }
        while (demod->pll_phase < -M_PI) {
            demod->pll_phase += 2.0f * M_PI;
        }
        
        recovered_carrier[i] = local_osc;
    }
}

void symbol_timing_recovery(const float complex *baseband_signal, int length,
                            int samples_per_symbol, float complex *symbol_samples)
{
    if (!baseband_signal || !symbol_samples) return;
    
    float K = 0.1f;
    float mu = 0.0f;
    int out_idx = 0;
    
    int i = samples_per_symbol;
    
    while (i < length - samples_per_symbol && 
           out_idx < length / samples_per_symbol) {
        
        int idx = (int)i;
        symbol_samples[out_idx] = baseband_signal[idx];
        out_idx++;
        
        float complex early = baseband_signal[idx - samples_per_symbol/2];
        float complex mid = baseband_signal[idx];
        float complex late = baseband_signal[idx + samples_per_symbol/2];
        
        float complex diff = late - early;
        float error = crealf(diff * conj(mid));
        
        mu += K * error;
        
        i += samples_per_symbol + mu;
        mu = mu - floorf(mu);
    }
}

void viterbi_detector(const float complex *symbols, int num_symbols,
                      uint8_t *decoded_bits)
{
    if (!symbols || !decoded_bits) return;
    
    for (int i = 0; i < num_symbols; i++) {
        float real_part = crealf(symbols[i]);
        float imag_part = cimagf(symbols[i]);
        
        if (real_part > 0 && imag_part > 0) {
            decoded_bits[2*i] = 0;
            decoded_bits[2*i+1] = 0;
        } else if (real_part < 0 && imag_part > 0) {
            decoded_bits[2*i] = 1;
            decoded_bits[2*i+1] = 0;
        } else if (real_part < 0 && imag_part < 0) {
            decoded_bits[2*i] = 1;
            decoded_bits[2*i+1] = 1;
        } else {
            decoded_bits[2*i] = 0;
            decoded_bits[2*i+1] = 1;
        }
    }
}

void SOQPSK_Demodulate(SOQPSK_Demodulator *demod, const float complex *received_signal,
                       int length, uint8_t *output_bits)
{
    if (!demod || !received_signal || !output_bits) return;
    
    float complex *carrier = malloc(length * sizeof(float complex));
    carrier_recovery_pll(received_signal, length, demod, carrier);
    
    float complex *baseband = malloc(length * sizeof(float complex));
    for (int i = 0; i < length; i++) {
        baseband[i] = received_signal[i] * conj(carrier[i]);
    }
    
    int num_symbols = length / demod->samples_per_symbol;
    float complex *symbol_samples = malloc(num_symbols * sizeof(float complex));
    symbol_timing_recovery(baseband, length, demod->samples_per_symbol, symbol_samples);
    
    viterbi_detector(symbol_samples, num_symbols, output_bits);
    
    free(carrier);
    free(baseband);
    free(symbol_samples);
}
