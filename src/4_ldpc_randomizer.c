#include "ldpc_codec.h"
#include <stdint.h>

#define PT_LFSR_INITIAL_SEED 0xACE1
#define PT_RANDOMIZER_STATISTICS_ENABLE 0
#define IRIGFIX_LFSR_POLY 0xB400

static uint16_t lfsr_state = PT_LFSR_INITIAL_SEED;

/* ASM 패턴 정의 (여기에 추가!) */
const uint8_t LDPC_ASM_PATTERN[] = {
    0x1A, 0xCF, 0xFC, 0x1D, 0x00, 0x00, 0x00, 0x00
};

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

int LDPC_DetectASM(const uint8_t *stream, int len)
{
    if (!stream || len < 64) return -1;
    
    int asm_size = sizeof(LDPC_ASM_PATTERN);
    
    for (int i = 0; i <= len - 64; i++) {
        int match_count = 0;
        
        for (int j = 0; j < asm_size && j < 8; j++) {
            if (i + j < len) {
                uint8_t diff = stream[i + j] ^ LDPC_ASM_PATTERN[j];
                int bits = 0;
                for (int k = 0; k < 8; k++) {
                    if (diff & (1 << k)) bits++;
                }
                match_count += bits;
            }
        }
        
        if (match_count <= 2) {
            return i;
        }
    }
    
    return -1;
}
