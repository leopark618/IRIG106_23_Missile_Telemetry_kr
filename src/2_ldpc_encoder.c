#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>

LDPC_Encoder* LDPC_Encoder_Create(LDPC_CodeRate rate)
{
    LDPC_Encoder *enc = malloc(sizeof(LDPC_Encoder));
    if (!enc) return NULL;
    
    enc->rate = rate;
    enc->N = IRIGFIX_LDPC_N;  /* ✅ LDPC_N → IRIGFIX_LDPC_N */
    
    /* ✅ LDPC_K_RATE_2_3 → 직접 값 지정 */
    switch (rate) {
        case LDPC_RATE_1_2: enc->K = 4096; break;  /* K = N/2 */
        case LDPC_RATE_2_3: enc->K = 5461; break;  /* K ≈ 2N/3 */
        case LDPC_RATE_4_5: enc->K = 6554; break;  /* K ≈ 4N/5 */
    }
    
    enc->M = enc->N - enc->K;
    enc->proto_rows = enc->M / IRIGFIX_LDPC_CIRCULANT_SIZE;  /* ✅ LDPC_CIRCULANT_SIZE → IRIGFIX_LDPC_CIRCULANT_SIZE */
    enc->proto_cols = enc->N / IRIGFIX_LDPC_CIRCULANT_SIZE;
    
    enc->proto_matrix = malloc(enc->proto_rows * sizeof(int8_t*));
    for (int i = 0; i < enc->proto_rows; i++) {
        enc->proto_matrix[i] = calloc(enc->proto_cols, sizeof(int8_t));
    }
    
    return enc;
}

void LDPC_Encoder_Destroy(LDPC_Encoder *enc)
{
    if (enc) {
        if (enc->proto_matrix) {
            for (int i = 0; i < enc->proto_rows; i++) {
                free(enc->proto_matrix[i]);
            }
            free(enc->proto_matrix);
        }
        free(enc);
    }
}

void LDPC_Encode(LDPC_Encoder *enc, const uint8_t *info, uint8_t *codeword)
{
    if (!enc || !info || !codeword) return;
    
    memcpy(codeword, info, enc->K);
    
    int z = IRIGFIX_LDPC_CIRCULANT_SIZE;  /* ✅ LDPC_CIRCULANT_SIZE → IRIGFIX_LDPC_CIRCULANT_SIZE */
    
    for (int i = 0; i < enc->M; i++) {
        codeword[enc->K + i] = 0;
    }
    
    for (int row = 0; row < enc->proto_rows; row++) {
        for (int col = 0; col < enc->proto_cols; col++) {
            int shift = enc->proto_matrix[row][col];
            if (shift < 0) continue;
            
            for (int i = 0; i < z; i++) {
                int info_idx = col * z + i;
                int parity_idx = row * z + ((i + shift) % z);
                
                if (info_idx < enc->K) {
                    codeword[enc->K + parity_idx] ^= codeword[info_idx];
                }
            }
        }
    }
}

