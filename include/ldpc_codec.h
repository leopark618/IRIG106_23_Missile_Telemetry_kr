#ifndef LDPC_CODEC_H
#define LDPC_CODEC_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * IRIG 106 Appendix R: LDPC 코드
 * ============================================================ */

/* IRIGFIX_: 고정 상수 (변경 금지) */
#define IRIGFIX_LDPC_N 8192                 /* 코드워드 길이 */
#define IRIGFIX_LDPC_CIRCULANT_SIZE 128     /* 순환 블록 크기 */
#define IRIGFIX_LDPC_ASM_LENGTH 64          /* ASM 길이 (비트) */

#define IRIGFIX_LFSR_POLY 0xB400            /* LFSR 다항식 */
#define IRIGFIX_LFSR_WIDTH 16               /* LFSR 비트폭 */

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

extern const uint8_t LDPC_ASM_PATTERN[];
int LDPC_DetectASM(const uint8_t *stream, int len);

#endif
