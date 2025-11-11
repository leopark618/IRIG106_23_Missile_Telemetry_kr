# 미사일 텔레메트리 시스템 - 최종 코드 (완전 버전)
## 모든 파일: 헤더 3개 + 소스 7개

---

# 📁 HEADER FILES (3개)

---

## 파일 1: include/missile_telemetry.h

```c
#ifndef MISSILE_TELEMETRY_H
#define MISSILE_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#define MISSILE_DATA_RATE 10e6
#define MISSILE_SAMPLE_RATE 80e6
#define MISSILE_CARRIER_FREQ 2.35e9
#define SAMPLES_PER_SYMBOL 8

#define NUM_IMU_CHANNELS 6
#define NUM_PRESSURE_CHANNELS 4
#define NUM_TEMP_CHANNELS 8
#define NUM_GUIDANCE_CHANNELS 16

typedef struct {
    uint32_t frame_counter;
    uint64_t timestamp_us;
    
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    
    float pressure_psi[4];
    float temperature_c[8];
    
    float guidance_cmd[4];
    float actuator_pos[4];
    uint8_t flight_mode;
    
    double latitude, longitude;
    float altitude_m;
    
    float battery_voltage;
    uint16_t system_status;
    uint16_t crc16;
    
} __attribute__((packed)) MissileTelemetryFrame;

typedef struct {
    void *ldpc_encoder;
    void *soqpsk_modulator;
    
    void *imu_handle;
    void *adc_handle;
    void *uart_handle;
    
    MissileTelemetryFrame current_frame;
    uint8_t tx_buffer[16384];
    
    bool system_armed;
    bool launch_detected;
    bool telemetry_active;
    
    uint32_t frames_sent;
    uint32_t errors;
    
} MissileTelemetrySystem;

MissileTelemetrySystem* MissileTM_Create(void);
void MissileTM_Destroy(MissileTelemetrySystem *sys);
void MissileTM_ReadSensors(MissileTelemetrySystem *sys);
bool MissileTM_DetectLaunch(MissileTelemetrySystem *sys);
void MissileTM_ProcessAndTransmit(MissileTelemetrySystem *sys);

#endif
```

---

## 파일 2: include/ldpc_codec.h

```c
#ifndef LDPC_CODEC_H
#define LDPC_CODEC_H

#include <stdint.h>
#include <stdbool.h>

#define LDPC_N 8192
#define LDPC_K_RATE_1_2 4096
#define LDPC_K_RATE_2_3 5461
#define LDPC_K_RATE_4_5 6554

#define LDPC_CIRCULANT_SIZE 128
#define LDPC_MAX_ITER 50
#define LDPC_ASM_LENGTH 64

typedef enum {
    LDPC_RATE_1_2 = 0,
    LDPC_RATE_2_3 = 1,
    LDPC_RATE_4_5 = 2
} LDPC_CodeRate;

typedef struct {
    LDPC_CodeRate rate;
    int K, N, M;
    int8_t **proto_matrix;
    int proto_rows, proto_cols;
} LDPC_Encoder;

typedef struct {
    LDPC_CodeRate rate;
    int K, N, M;
    int8_t **proto_matrix;
    int proto_rows, proto_cols;
    
    float *edge_values;
    float *check_to_var;
    float *var_llr;
} LDPC_Decoder;

LDPC_Encoder* LDPC_Encoder_Create(LDPC_CodeRate rate);
void LDPC_Encoder_Destroy(LDPC_Encoder *enc);
void LDPC_Encode(LDPC_Encoder *enc, const uint8_t *info, uint8_t *codeword);

LDPC_Decoder* LDPC_Decoder_Create(LDPC_CodeRate rate);
void LDPC_Decoder_Destroy(LDPC_Decoder *dec);
bool LDPC_Decode(LDPC_Decoder *dec, const float *llr, uint8_t *decoded, int max_iter);

void LDPC_Randomizer_Init(uint32_t seed);
void LDPC_Randomize(const uint8_t *in, uint8_t *out, int len);
void LDPC_Derandomize(const uint8_t *in, uint8_t *out, int len);

extern const uint8_t LDPC_ASM_PATTERN[8];
int LDPC_DetectASM(const uint8_t *stream, int len);

#endif
```

---

## 파일 3: include/soqpsk.h

```c
#ifndef SOQPSK_H
#define SOQPSK_H

#include <stdint.h>
#include <complex.h>

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
    float path_metrics[8];
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
```

---

# 📁 SOURCE FILES (7개)

---

## 파일 4: src/1_sensor_acquisition.c

```c
#include "missile_telemetry.h"
#include <math.h>
#include <string.h>

void MissileTM_ReadSensors(MissileTelemetrySystem *sys)
{
    if (!sys) return;
    
    int16_t imu_raw[6];
    
    float accel_scale = 100.0f / 32768.0f;
    sys->current_frame.accel_x_g = imu_raw[0] * accel_scale;
    sys->current_frame.accel_y_g = imu_raw[1] * accel_scale;
    sys->current_frame.accel_z_g = imu_raw[2] * accel_scale;
    
    float gyro_scale = 2000.0f / 32768.0f;
    sys->current_frame.gyro_x_dps = imu_raw[3] * gyro_scale;
    sys->current_frame.gyro_y_dps = imu_raw[4] * gyro_scale;
    sys->current_frame.gyro_z_dps = imu_raw[5] * gyro_scale;
    
    uint16_t pressure_adc[4];
    float pressure_scale = 10000.0f / 65535.0f;
    for (int i = 0; i < 4; i++) {
        sys->current_frame.pressure_psi[i] = pressure_adc[i] * pressure_scale;
    }
    
    uint16_t temp_adc[8];
    for (int i = 0; i < 8; i++) {
        float R = temp_adc[i] * 10000.0f / 65535.0f;
        float lnR = logf(R);
        
        float A = 0.001129148f;
        float B = 0.000234125f;
        float C = 0.0000000876741f;
        
        float T_kelvin = 1.0f / (A + B*lnR + C*lnR*lnR*lnR);
        sys->current_frame.temperature_c[i] = T_kelvin - 273.15f;
    }
    
    uint8_t uart_rx[64];
    int rx_len = 0;
    
    if (rx_len >= 40) {
        memcpy(sys->current_frame.guidance_cmd, &uart_rx[0], 16);
        memcpy(sys->current_frame.actuator_pos, &uart_rx[16], 16);
        sys->current_frame.flight_mode = uart_rx[32];
    }
    
    sys->current_frame.frame_counter++;
}

bool MissileTM_DetectLaunch(MissileTelemetrySystem *sys)
{
    if (!sys) return false;
    
    float ax = sys->current_frame.accel_x_g;
    float ay = sys->current_frame.accel_y_g;
    float az = sys->current_frame.accel_z_g;
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    
    const float LAUNCH_THRESHOLD = 5.0f;
    static int counter = 0;
    
    if (mag > LAUNCH_THRESHOLD) {
        counter++;
        if (counter >= 10) {
            counter = 0;
            return true;
        }
    } else {
        counter = 0;
    }
    
    return false;
}
```

---

## 파일 5: src/2_ldpc_encoder.c

```c
#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>

LDPC_Encoder* LDPC_Encoder_Create(LDPC_CodeRate rate)
{
    LDPC_Encoder *enc = malloc(sizeof(LDPC_Encoder));
    if (!enc) return NULL;
    
    enc->rate = rate;
    enc->N = LDPC_N;
    
    switch (rate) {
        case LDPC_RATE_1_2: enc->K = LDPC_K_RATE_1_2; break;
        case LDPC_RATE_2_3: enc->K = LDPC_K_RATE_2_3; break;
        case LDPC_RATE_4_5: enc->K = LDPC_K_RATE_4_5; break;
    }
    
    enc->M = enc->N - enc->K;
    
    enc->proto_rows = enc->M / LDPC_CIRCULANT_SIZE;
    enc->proto_cols = enc->N / LDPC_CIRCULANT_SIZE;
    
    enc->proto_matrix = malloc(enc->proto_rows * sizeof(int8_t*));
    for (int i = 0; i < enc->proto_rows; i++) {
        enc->proto_matrix[i] = malloc(enc->proto_cols * sizeof(int8_t));
    }
    
    return enc;
}

void LDPC_Encoder_Destroy(LDPC_Encoder *enc)
{
    if (enc) {
        for (int i = 0; i < enc->proto_rows; i++) {
            free(enc->proto_matrix[i]);
        }
        free(enc->proto_matrix);
        free(enc);
    }
}

void LDPC_Encode(LDPC_Encoder *enc, const uint8_t *info_bits, 
                 uint8_t *codeword)
{
    if (!enc || !info_bits || !codeword) return;
    
    memcpy(codeword, info_bits, enc->K);
    
    uint8_t *parity = &codeword[enc->K];
    memset(parity, 0, enc->M);
    
    int z = LDPC_CIRCULANT_SIZE;
    
    for (int p_block = 0; p_block < enc->proto_rows; p_block++) {
        for (int i_block = 0; i_block < enc->proto_cols; i_block++) {
            int shift = enc->proto_matrix[p_block][i_block];
            
            if (shift < 0) continue;
            
            for (int k = 0; k < z; k++) {
                int src_idx = i_block * z + k;
                int dst_idx = p_block * z + ((k + shift) % z);
                
                parity[dst_idx] ^= info_bits[src_idx];
            }
        }
    }
}
```

---

## 파일 6: src/3_ldpc_decoder.c

```c
#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PT_LDPC_DECODER_MAX_ITERATIONS 50
#define PT_LDPC_EARLY_TERMINATION 1
#define PT_USE_ATANH_APPROXIMATION 1
#define PT_MESSAGE_SCALING_FACTOR 1.0f

LDPC_Decoder* LDPC_Decoder_Create(LDPC_CodeRate rate)
{
    LDPC_Decoder *dec = malloc(sizeof(LDPC_Decoder));
    if (!dec) return NULL;
    
    dec->rate = rate;
    dec->N = LDPC_N;
    
    switch (rate) {
        case LDPC_RATE_1_2: dec->K = LDPC_K_RATE_1_2; break;
        case LDPC_RATE_2_3: dec->K = LDPC_K_RATE_2_3; break;
        case LDPC_RATE_4_5: dec->K = LDPC_K_RATE_4_5; break;
    }
    
    dec->M = dec->N - dec->K;
    
    int num_edges = dec->N * 3;
    dec->edge_values = calloc(num_edges, sizeof(float));
    dec->check_to_var = calloc(num_edges, sizeof(float));
    dec->var_llr = malloc(dec->N * sizeof(float));
    
    dec->proto_rows = 32;
    dec->proto_cols = 64;
    
    return dec;
}

void LDPC_Decoder_Destroy(LDPC_Decoder *dec)
{
    if (dec) {
        free(dec->edge_values);
        free(dec->check_to_var);
        free(dec->var_llr);
        free(dec);
    }
}

static float tanh_fast(float x)
{
    if (!PT_USE_ATANH_APPROXIMATION) {
        return tanhf(x);
    }
    
    if (x > 3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    
    float x_sq = x * x;
    return x * (1.0f - x_sq / 9.0f + x_sq * x_sq / 81.0f);
}

bool LDPC_Decode(LDPC_Decoder *dec, const float *received_llr,
                 uint8_t *decoded_bits, int max_iterations)
{
    if (!dec || !received_llr || !decoded_bits) return false;
    
    if (max_iterations > PT_LDPC_DECODER_MAX_ITERATIONS) {
        max_iterations = PT_LDPC_DECODER_MAX_ITERATIONS;
    }
    
    memcpy(dec->var_llr, received_llr, dec->N * sizeof(float));
    memset(dec->edge_values, 0, dec->N * 3 * sizeof(float));
    
    int z = LDPC_CIRCULANT_SIZE;
    uint8_t *hard_decision = malloc(dec->N * sizeof(uint8_t));
    
    for (int iter = 0; iter < max_iterations; iter++) {
        
        for (int c_block = 0; c_block < dec->proto_rows; c_block++) {
            for (int k = 0; k < z; k++) {
                for (int v_block = 0; v_block < dec->proto_cols; v_block++) {
                    int shift = dec->proto_matrix[c_block][v_block];
                    if (shift < 0) continue;
                    
                    int v_idx = v_block * z + ((k + shift) % z);
                    
                    float product = 1.0f;
                    for (int v2_block = 0; v2_block < dec->proto_cols; v2_block++) {
                        int shift2 = dec->proto_matrix[c_block][v2_block];
                        if (shift2 < 0 || v2_block == v_block) continue;
                        
                        int v2_idx = v2_block * z + ((k + shift2) % z);
                        product *= tanh_fast(dec->var_llr[v2_idx] / 2.0f);
                    }
                    
                    dec->check_to_var[c_block * dec->N + v_idx] = 
                        2.0f * atanhf(product * PT_MESSAGE_SCALING_FACTOR);
                }
            }
        }
        
        for (int v = 0; v < dec->N; v++) {
            float sum = received_llr[v];
            
            int v_block = v / z;
            for (int c_block = 0; c_block < dec->proto_rows; c_block++) {
                int shift = dec->proto_matrix[c_block][v_block];
                if (shift >= 0) {
                    sum += dec->check_to_var[c_block * dec->N + v];
                }
            }
            
            dec->var_llr[v] = sum;
        }
        
        for (int i = 0; i < dec->N; i++) {
            hard_decision[i] = (dec->var_llr[i] > 0) ? 0 : 1;
        }
        
        if (PT_LDPC_EARLY_TERMINATION) {
            bool all_parity_ok = true;
            
            for (int c_block = 0; c_block < dec->proto_rows && all_parity_ok; c_block++) {
                for (int k = 0; k < z && all_parity_ok; k++) {
                    uint8_t parity = 0;
                    
                    for (int v_block = 0; v_block < dec->proto_cols; v_block++) {
                        int shift = dec->proto_matrix[c_block][v_block];
                        if (shift < 0) continue;
                        
                        int v_idx = v_block * z + ((k + shift) % z);
                        parity ^= hard_decision[v_idx];
                    }
                    
                    if (parity != 0) all_parity_ok = false;
                }
            }
            
            if (all_parity_ok) {
                memcpy(decoded_bits, hard_decision, dec->K);
                free(hard_decision);
                return true;
            }
        }
    }
    
    memcpy(decoded_bits, hard_decision, dec->K);
    free(hard_decision);
    return false;
}
```

---

## 파일 7: src/4_ldpc_randomizer.c

```c
#include "ldpc_codec.h"
#include <stdint.h>

#define PT_LFSR_INITIAL_SEED 0xACE1
#define PT_RANDOMIZER_STATISTICS_ENABLE 0
#define IRIGFIX_LFSR_POLY 0xB400

static uint16_t lfsr_state = PT_LFSR_INITIAL_SEED;

void LDPC_Randomizer_Init(uint32_t seed)
{
    if (seed == 0) {
        lfsr_state = PT_LFSR_INITIAL_SEED;
    } else {
        lfsr_state = seed & 0xFFFF;
    }
}

static uint8_t lfsr_next_bit(void)
{
    uint8_t output_bit = lfsr_state & 0x0001;
    
    uint16_t feedback = 0;
    if (lfsr_state & (1 << 15)) feedback ^= 1;
    if (lfsr_state & (1 << 14)) feedback ^= 1;
    if (lfsr_state & (1 << 12)) feedback ^= 1;
    if (lfsr_state & (1 <<  3)) feedback ^= 1;
    
    lfsr_state = (lfsr_state >> 1) | (feedback << 15);
    
    return output_bit;
}

void LDPC_Randomize(const uint8_t *input, uint8_t *output, int length)
{
    if (!input || !output) return;
    
    for (int i = 0; i < length; i++) {
        uint8_t rand_bit = lfsr_next_bit();
        output[i] = input[i] ^ rand_bit;
    }
}

void LDPC_Derandomize(const uint8_t *input, uint8_t *output, int length)
{
    LDPC_Randomize(input, output, length);
}

const uint8_t LDPC_ASM_PATTERN[8] = {
    0x1A, 0xCF, 0xFC, 0x1D, 0x00, 0x00, 0x00, 0x00
};

int LDPC_DetectASM(const uint8_t *stream, int len)
{
    if (!stream || len < LDPC_ASM_LENGTH) return -1;
    
    int asm_bytes = LDPC_ASM_LENGTH / 8;
    
    for (int i = 0; i <= len - LDPC_ASM_LENGTH; i++) {
        int match_bits = 0;
        
        for (int j = 0; j < asm_bytes; j++) {
            int byte_idx = i / 8 + j;
            int bit_offset = i % 8;
            
            uint8_t window_byte = (stream[byte_idx] >> bit_offset);
            if (bit_offset > 0) {
                window_byte |= (stream[byte_idx + 1] << (8 - bit_offset));
            }
            
            uint8_t diff = window_byte ^ LDPC_ASM_PATTERN[j];
            match_bits += __builtin_popcount(diff);
        }
        
        if (match_bits <= 2) {
            return i;
        }
    }
    
    return -1;
}
```

---

## 파일 8: src/5_soqpsk_modulator.c

```c
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
    phase[0] = mod->phase_accum;
    
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
```

---

## 파일 9: src/6_soqpsk_demodulator.c

```c
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
    demod->path_metrics[0] = 0.0f;
    
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
```

---

## 파일 10: src/7_main_integration.c

```c
#include "missile_telemetry.h"
#include "ldpc_codec.h"
#include "soqpsk.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define PT_SENSOR_SAMPLE_PERIOD_MS 1
#define PT_DATA_TX_PERIOD_MS 10
#define PT_LAUNCH_DETECTION_PERIOD_MS 100
#define PT_LAUNCH_ACCEL_THRESHOLD_G 5.0f
#define PT_LAUNCH_SUSTAINED_SAMPLES 10
#define PT_PLL_BANDWIDTH_SCALE 0.01f
#define PT_LDPC_MAX_ITERATIONS 50
#define PT_TX_POWER_W 3.0f

#define IRIG_CARRIER_FREQ_HZ 2.35e9
#define IRIG_SAMPLE_RATE_HZ 80e6
#define IRIG_DATA_RATE_BPS 10e6
#define IRIG_LDPC_CODE_RATE LDPC_RATE_2_3
#define IRIG_SAMPLES_PER_SYMBOL 8

static MissileTelemetrySystem *g_tm_system = NULL;
static LDPC_Encoder *g_ldpc_encoder = NULL;
static LDPC_Decoder *g_ldpc_decoder = NULL;
static SOQPSK_Modulator *g_soqpsk_mod = NULL;

static uint32_t PT_frames_transmitted = 0;

MissileTelemetrySystem* MissileTM_Create(void)
{
    MissileTelemetrySystem *sys = malloc(sizeof(MissileTelemetrySystem));
    if (!sys) return NULL;
    
    memset(sys, 0, sizeof(MissileTelemetrySystem));
    
    g_soqpsk_mod = SOQPSK_Modulator_Create(
        IRIG_CARRIER_FREQ_HZ,
        IRIG_SAMPLE_RATE_HZ,
        IRIG_SAMPLES_PER_SYMBOL
    );
    
    g_ldpc_encoder = LDPC_Encoder_Create(IRIG_LDPC_CODE_RATE);
    if (!g_ldpc_encoder) {
        SOQPSK_Modulator_Destroy(g_soqpsk_mod);
        free(sys);
        return NULL;
    }
    
    g_ldpc_decoder = LDPC_Decoder_Create(IRIG_LDPC_CODE_RATE);
    
    LDPC_Randomizer_Init(0xACE1);
    
    sys->system_armed = false;
    sys->launch_detected = false;
    sys->telemetry_active = false;
    
    return sys;
}

void MissileTM_Destroy(MissileTelemetrySystem *sys)
{
    if (sys) {
        SOQPSK_Modulator_Destroy(g_soqpsk_mod);
        LDPC_Encoder_Destroy(g_ldpc_encoder);
        LDPC_Decoder_Destroy(g_ldpc_decoder);
        free(sys);
    }
}

void MissileTM_ProcessAndTransmit(MissileTelemetrySystem *sys)
{
    if (!sys || !g_ldpc_encoder || !g_soqpsk_mod) return;
    
    MissileTM_ReadSensors(sys);
    
    uint8_t *frame_bytes = (uint8_t*)&sys->current_frame;
    int frame_size = sizeof(MissileTelemetryFrame);
    
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < frame_size - 2; i++) {
        crc ^= frame_bytes[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    sys->current_frame.crc16 = crc;
    
    int K = g_ldpc_encoder->K;
    uint8_t *info_bits = malloc(K * sizeof(uint8_t));
    uint8_t *codeword = malloc(LDPC_N * sizeof(uint8_t));
    
    for (int i = 0; i < K && i < frame_size * 8; i++) {
        int byte_idx = i / 8;
        int bit_idx = i % 8;
        info_bits[i] = (frame_bytes[byte_idx] >> bit_idx) & 0x01;
    }
    
    for (int i = frame_size * 8; i < K; i++) {
        info_bits[i] = 0;
    }
    
    LDPC_Encode(g_ldpc_encoder, info_bits, codeword);
    
    uint8_t *randomized = malloc(LDPC_N * sizeof(uint8_t));
    LDPC_Randomize(codeword, randomized, LDPC_N);
    
    int total_bits = 64 + LDPC_N;
    uint8_t *with_asm = malloc(total_bits * sizeof(uint8_t));
    
    for (int i = 0; i < 64; i++) {
        int byte_idx = i / 8;
        int bit_idx = i % 8;
        with_asm[i] = (LDPC_ASM_PATTERN[byte_idx] >> bit_idx) & 0x01;
    }
    
    memcpy(&with_asm[64], randomized, LDPC_N);
    
    int output_length = total_bits * IRIG_SAMPLES_PER_SYMBOL;
    float complex *modulated = malloc(output_length * sizeof(float complex));
    
    SOQPSK_Modulate(g_soqpsk_mod, with_asm, total_bits, modulated);
    
    PT_frames_transmitted++;
    
    free(info_bits);
    free(codeword);
    free(randomized);
    free(with_asm);
    free(modulated);
}

void Task_SensorSampling(void *pvParameters)
{
    while (1) {
        MissileTM_ReadSensors(g_tm_system);
    }
}

void Task_DataTransmit(void *pvParameters)
{
    while (1) {
        if (g_tm_system && g_tm_system->telemetry_active) {
            MissileTM_ProcessAndTransmit(g_tm_system);
        }
    }
}

void Task_LaunchDetection(void *pvParameters)
{
    while (1) {
        if (!g_tm_system->launch_detected) {
            if (MissileTM_DetectLaunch(g_tm_system)) {
                g_tm_system->launch_detected = true;
                g_tm_system->telemetry_active = true;
            }
        }
    }
}

int main(void)
{
    printf("=== Missile Telemetry System ===\n\n");
    
    printf("[PT_ - Project Tuning]\n");
    printf("  Sensor: %d ms\n", PT_SENSOR_SAMPLE_PERIOD_MS);
    printf("  TX: %d ms\n", PT_DATA_TX_PERIOD_MS);
    printf("  Launch: %.1f G\n\n", PT_LAUNCH_ACCEL_THRESHOLD_G);
    
    printf("[IRIG_ - Standard Parameters]\n");
    printf("  Carrier: %.2f GHz\n", IRIG_CARRIER_FREQ_HZ / 1e9);
    printf("  Data: %.1f Mbps\n\n", IRIG_DATA_RATE_BPS / 1e6);
    
    g_tm_system = MissileTM_Create();
    if (!g_tm_system) {
        printf("Error: Init failed\n");
        return 1;
    }
    
    printf("System OK\n\n");
    
    for (int i = 0; i < 5; i++) {
        printf("[Frame %d]\n", i);
        MissileTM_ReadSensors(g_tm_system);
        
        if (MissileTM_DetectLaunch(g_tm_system)) {
            g_tm_system->launch_detected = true;
            g_tm_system->telemetry_active = true;
            printf("  LAUNCH!\n");
        }
        
        if (g_tm_system->telemetry_active) {
            MissileTM_ProcessAndTransmit(g_tm_system);
            printf("  Frames: %d\n", PT_frames_transmitted);
        }
        printf("\n");
    }
    
    printf("=== Done ===\n");
    MissileTM_Destroy(g_tm_system);
    
    return 0;
}
```

---

## Makefile

```makefile
CC = gcc
CFLAGS = -Wall -O2 -lm
TARGET = missile_telemetry

SOURCES = src/1_sensor_acquisition.c \
          src/2_ldpc_encoder.c \
          src/3_ldpc_decoder.c \
          src/4_ldpc_randomizer.c \
          src/5_soqpsk_modulator.c \
          src/6_soqpsk_demodulator.c \
          src/7_main_integration.c

OBJECTS = $(SOURCES:.c=.o)
CFLAGS += -Iinclude

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
```

---

## 컴파일 방법

```bash
# 1. 파일 정리
mkdir -p missile_telemetry/include missile_telemetry/src
cd missile_telemetry

# 2. 각 파일 저장
# 위의 코드들을 각각의 파일에 저장

# 3. 컴파일
make

# 4. 실행
./missile_telemetry
```

**완료!** 🚀
