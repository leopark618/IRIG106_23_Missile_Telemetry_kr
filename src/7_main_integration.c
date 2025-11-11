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
    
    memcpy(&with_asm, randomized, LDPC_N);
    
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
