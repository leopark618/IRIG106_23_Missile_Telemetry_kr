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
