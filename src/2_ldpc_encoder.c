#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>

LDPC_Encoder* LDPC_Encoder_Create(LDPC_CodeRate rate)
{
    LDPC_Encoder *enc = malloc(sizeof(LDPC_Encoder));
    if (!enc) return NULL;
    
    enc->rate = rate;
    enc->N = LDPC_N;
    enc->K = LDPC_K_RATE_2_3
    
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
