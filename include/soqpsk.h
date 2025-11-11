#ifndef SOQPSK_H
#define SOQPSK_H

#include <stdint.h>


#define SOQPSK_RHO 0.70
#define SOQPSK_B 1.25
#define SOQPSK_T1 1.5
#define SOQPSK_T2 0.50

typedef struct {
    float carrier_freq;
    float sample_rate;
    int samples_per_symbol;
    float *frequency_pulse;
    int pulse_length;
    float phase_accum;
} SOQPSK_Modulator;

typedef struct {
    float carrier_freq;
    float sample_rate;
    int samples_per_symbol;
    
    float pll_phase, pll_freq;
    float loop_bw, damping;
    
    float timing_mu, timing_error;
    
    uint8_t current_state;
    float path_metrics;
} SOQPSK_Demodulator;

SOQPSK_Modulator* SOQPSK_Modulator_Create(float fc, float fs, int sps);
void SOQPSK_Modulator_Destroy(SOQPSK_Modulator *mod);
void SOQPSK_Modulate(SOQPSK_Modulator *mod, const uint8_t *bits, 
                     int nbits, float complex *output);

SOQPSK_Demodulator* SOQPSK_Demodulator_Create(float fc, float fs, int sps);
void SOQPSK_Demodulator_Destroy(SOQPSK_Demodulator *demod);
void SOQPSK_Demodulate(SOQPSK_Demodulator *demod, const float complex *rx, 
                       int len, uint8_t *bits);

void differential_precoder(const uint8_t *in, int8_t *out, int n);
void create_frequency_pulse(float *pulse, int len, float Ts);

#endif
