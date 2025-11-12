#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

LDPC_Decoder* LDPC_Decoder_Create(LDPC_CodeRate rate)
{
    LDPC_Decoder *dec = malloc(sizeof(LDPC_Decoder));
    if (!dec) return NULL;
    
    dec->rate = rate;
    dec->N = IRIGFIX_LDPC_N;  /*  수정 */
    dec->K = 5461;  /* 기본값 설정 */
    /*  직접 값 지정 */
    switch (rate) {
        case LDPC_RATE_1_2: dec->K = 4096; break;
        case LDPC_RATE_2_3: dec->K = 5461; break;
        case LDPC_RATE_4_5: dec->K = 6554; break;
    }
    
    dec->M = dec->N - dec->K;
    dec->proto_rows = dec->M / IRIGFIX_LDPC_CIRCULANT_SIZE;  /*  수정 */
    dec->proto_cols = dec->N / IRIGFIX_LDPC_CIRCULANT_SIZE;
    
    dec->proto_matrix = malloc(dec->proto_rows * sizeof(int8_t*));
    for (int i = 0; i < dec->proto_rows; i++) {
        dec->proto_matrix[i] = calloc(dec->proto_cols, sizeof(int8_t));
    }
    
    int num_edges = dec->M * 3;
    dec->edge_values = calloc(num_edges, sizeof(float));
    dec->check_to_var = calloc(num_edges, sizeof(float));
    dec->var_llr = calloc(dec->N, sizeof(float));
    
    return dec;
}

void LDPC_Decoder_Destroy(LDPC_Decoder *dec)
{
    if (dec) {
        if (dec->proto_matrix) {
            for (int i = 0; i < dec->proto_rows; i++) {
                free(dec->proto_matrix[i]);
            }
            free(dec->proto_matrix);
        }
        free(dec->edge_values);
        free(dec->check_to_var);
        free(dec->var_llr);
        free(dec);
    }
}

bool LDPC_Decode(LDPC_Decoder *dec, const float *llr, uint8_t *decoded, int max_iter)
{
    if (!dec || !llr || !decoded) return false;
    
    memcpy(dec->var_llr, llr, dec->N * sizeof(float));
    
    for (int iter = 0; iter < max_iter; iter++) {
        for (int i = 0; i < dec->N; i++) {
            decoded[i] = (dec->var_llr[i] > 0) ? 0 : 1;
        }
        
        int syndrome = 0;
        for (int i = 0; i < dec->M; i++) {
            int sum = 0;
            for (int j = 0; j < dec->N; j++) {
                sum ^= decoded[j];
            }
            if (sum != 0) syndrome = 1;
        }
        
        if (syndrome == 0) return true;
    }
    
    return false;
}
