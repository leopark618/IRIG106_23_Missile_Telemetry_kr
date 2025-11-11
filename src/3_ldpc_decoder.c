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
    dec->K = LDPC_K_RATE_2_3;

    
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
