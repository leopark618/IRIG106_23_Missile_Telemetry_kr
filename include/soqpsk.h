#ifndef SOQPSK_H
#define SOQPSK_H

#include <stdint.h>

/* ============================================================
 * IRIG 106 Appendix M: SOQPSK-TG 변조
 * ============================================================ */

/* IRIGFIX_: 고정 상수 (변경 금지) */
#define IRIGFIX_CPM_RHO 0.70                /* CPM 스무딩 인자 */
#define IRIGFIX_CPM_B 1.25                  /* CPM 대역폭 인자 */
#define IRIGFIX_CPM_T1 1.5                  /* CPM 시간 파라미터 1 */
#define IRIGFIX_CPM_T2 0.50                 /* CPM 시간 파라미터 2 */

typedef struct {
    float real;
    float imag;
} float_complex;

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
    float path_metrics[8];
} SOQPSK_Demodulator;

SOQPSK_Modulator* SOQPSK_Modulator_Create(float fc, float fs, int sps);
void SOQPSK_Modulator_Destroy(SOQPSK_Modulator *mod);
void SOQPSK_Modulate(SOQPSK_Modulator *mod, const uint8_t *bits, 
                     int nbits, float_complex *output);

SOQPSK_Demodulator* SOQPSK_Demodulator_Create(float fc, float fs, int sps);
void SOQPSK_Demodulator_Destroy(SOQPSK_Demodulator *demod);
void SOQPSK_Demodulate(SOQPSK_Demodulator *demod, const float_complex *rx, 
                       int len, uint8_t *bits);

void differential_precoder(const uint8_t *in, int8_t *out, int n);
void create_frequency_pulse(float *pulse, int len, float Ts);

#endif
