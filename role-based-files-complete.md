# 미사일 텔레메트리 - 역할별 파일 분류 및 변수 프리픽스
## 완전 정리: 5가지 핵심 역할로 분류

---

## 📁 역할별 파일 구조

```
역할별로 정리된 미사일 텔레메트리 시스템:

1️⃣  MAIN_INTEGRATION
    └─ 7_main_integration.c (시스템 통합, FreeRTOS)

2️⃣  LDPC (Low-Density Parity Check)
    ├─ 2_ldpc_encoder.c (인코딩)
    ├─ 3_ldpc_decoder.c (디코딩)
    └─ 4_ldpc_randomizer.c (랜더마이저)

3️⃣  SOQPSK (변조/복조)
    ├─ 5_soqpsk_modulator.c (변조)
    └─ 6_soqpsk_demodulator.c (복조)

4️⃣  SENSOR (센서 수집)
    └─ 1_sensor_acquisition.c (IMU/압력/온도/유도)

5️⃣  HEADER (공통 정의)
    ├─ missile_telemetry.h
    ├─ ldpc_codec.h
    └─ soqpsk.h
```

---

## 🎯 역할 1: MAIN_INTEGRATION (메인 시스템 통합)

### 파일: 7_main_integration.c

**역할**: 전체 시스템 조율, FreeRTOS 멀티태스킹, 데이터 파이프라인

**IRIG 근거**: Chapter 1, Section 1.1 "System Architecture"

```c
/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * 파일: 7_main_integration.c
 * 
 * 역할: 시스템 메인 통합
 * - 센서 관리
 * - LDPC + SOQPSK 연동
 * - FreeRTOS 멀티태스킹
 * - 7단계 전송 파이프라인
 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/

#include "missile_telemetry.h"
#include "ldpc_codec.h"
#include "soqpsk.h"
#include <string.h>
#include <math.h>

/*════════════════════════════════════════════════════════════════════
 * 프로젝트 튜닝 변수 (PT_)
 * 
 * ✅ 자유롭게 변경 가능 - 이 값들을 조정하면서 성능 튜닝
 ════════════════════════════════════════════════════════════════════*/

/* Task 주기 */
#define PT_SENSOR_SAMPLE_PERIOD_MS 1       /* 1kHz 센서 샘플링 */
#define PT_DATA_TX_PERIOD_MS 10            /* 100Hz 전송 */
#define PT_LAUNCH_DETECTION_PERIOD_MS 100  /* 발사 감지 */

/* 발사 감지 */
#define PT_LAUNCH_ACCEL_THRESHOLD_G 5.0f   /* 임계값 5G */
#define PT_LAUNCH_SUSTAINED_SAMPLES 10     /* 10ms 지속 */

/* PLL 튜닝 */
#define PT_PLL_BANDWIDTH_SCALE 0.01f       /* 심볼율의 1% */
#define PT_PLL_DAMPING_FACTOR 0.707f       /* 임계 감쇠 */

/* LDPC 처리 */
#define PT_LDPC_MAX_ITERATIONS 50          /* 반복 횟수 */
#define PT_LDPC_EARLY_TERMINATION_ENABLE 1 /* 조기 종료 */

/* 전력 */
#define PT_TX_POWER_W 3.0f                 /* 송신 전력 3W */
#define PT_BATTERY_LOW_VOLTAGE_V 10.0f     /* 저전압 경고 */
#define PT_TEMPERATURE_HIGH_LIMIT_C 85.0f  /* 과열 경고 */

/*════════════════════════════════════════════════════════════════════
 * IRIG 106 표준 파라미터 (IRIG_)
 * 
 * ⚠️ 신중하게 변경 - 지상국 재조정 필요 가능
 ════════════════════════════════════════════════════════════════════*/

#define IRIG_CARRIER_FREQ_HZ 2.35e9       /* 2.35 GHz */
#define IRIG_SAMPLE_RATE_HZ 80e6          /* 80 MHz */
#define IRIG_DATA_RATE_BPS 10e6           /* 10 Mbps */
#define IRIG_LDPC_CODE_RATE LDPC_RATE_2_3 /* Rate 2/3 */
#define IRIG_SAMPLES_PER_SYMBOL 8

/*════════════════════════════════════════════════════════════════════
 * IRIG 106 고정 상수 (IRIGFIX_)
 * 
 * ❌ 절대 변경 금지 - 변경 시 호환성 완전 상실
 ════════════════════════════════════════════════════════════════════*/

#define IRIGFIX_CPM_RHO 0.70      /* Appendix M */
#define IRIGFIX_CPM_B 1.25        /* Appendix M */
#define IRIGFIX_LDPC_N 8192       /* Appendix R */

/*────────────────────────────────────────────────────────────────────
 * 전역 변수
 ────────────────────────────────────────────────────────────────────*/

static MissileTelemetrySystem *g_tm_system = NULL;
static LDPC_Encoder *g_ldpc_encoder = NULL;
static LDPC_Decoder *g_ldpc_decoder = NULL;
static SOQPSK_Modulator *g_soqpsk_mod = NULL;

static uint32_t PT_frames_transmitted = 0;
static uint32_t PT_total_errors = 0;
static float PT_last_accel_magnitude = 0.0f;

/*────────────────────────────────────────────────────────────────────
 * 함수: MissileTM_Create
 ────────────────────────────────────────────────────────────────────*/
MissileTelemetrySystem* MissileTM_Create(void)
{
    MissileTelemetrySystem *PT_sys = malloc(sizeof(MissileTelemetrySystem));
    if (!PT_sys) return NULL;
    
    memset(PT_sys, 0, sizeof(MissileTelemetrySystem));
    
    g_soqpsk_mod = SOQPSK_Modulator_Create(
        IRIG_CARRIER_FREQ_HZ,
        IRIG_SAMPLE_RATE_HZ,
        IRIG_SAMPLES_PER_SYMBOL
    );
    
    g_ldpc_encoder = LDPC_Encoder_Create(IRIG_LDPC_CODE_RATE);
    if (!g_ldpc_encoder) {
        SOQPSK_Modulator_Destroy(g_soqpsk_mod);
        free(PT_sys);
        return NULL;
    }
    
    g_ldpc_decoder = LDPC_Decoder_Create(IRIG_LDPC_CODE_RATE);
    
    LDPC_Randomizer_Init(0xACE1);
    
    PT_sys->system_armed = false;
    PT_sys->launch_detected = false;
    PT_sys->telemetry_active = false;
    
    return PT_sys;
}

void MissileTM_Destroy(MissileTelemetrySystem *PT_sys)
{
    if (PT_sys) {
        SOQPSK_Modulator_Destroy(g_soqpsk_mod);
        LDPC_Encoder_Destroy(g_ldpc_encoder);
        LDPC_Decoder_Destroy(g_ldpc_decoder);
        free(PT_sys);
    }
}

/*────────────────────────────────────────────────────────────────────
 * 함수: MissileTM_ProcessAndTransmit
 * 
 * 7단계 파이프라인:
 * 1. 센서 수집
 * 2. CRC 계산
 * 3. LDPC 인코딩
 * 4. 랜더마이저
 * 5. ASM 추가
 * 6. SOQPSK 변조
 * 7. RF 전송
 ────────────────────────────────────────────────────────────────────*/
void MissileTM_ProcessAndTransmit(MissileTelemetrySystem *PT_sys)
{
    if (!PT_sys) return;
    
    /* Step 1: 센서 수집 */
    MissileTM_ReadSensors(PT_sys);
    
    /* Step 2: CRC 계산 */
    uint8_t *PT_frame_bytes = (uint8_t*)&PT_sys->current_frame;
    uint16_t PT_crc = 0xFFFF;
    for (int i = 0; i < sizeof(MissileTelemetryFrame) - 2; i++) {
        PT_crc ^= PT_frame_bytes[i];
        for (int j = 0; j < 8; j++) {
            if (PT_crc & 0x0001) {
                PT_crc = (PT_crc >> 1) ^ 0x8408;
            } else {
                PT_crc >>= 1;
            }
        }
    }
    PT_sys->current_frame.crc16 = PT_crc;
    
    /* Step 3: LDPC 인코딩 */
    int IRIG_K = g_ldpc_encoder->K;
    uint8_t *PT_info_bits = malloc(IRIG_K * sizeof(uint8_t));
    uint8_t *PT_codeword = malloc(IRIGFIX_LDPC_N * sizeof(uint8_t));
    
    for (int i = 0; i < IRIG_K && i < sizeof(MissileTelemetryFrame) * 8; i++) {
        int PT_byte_idx = i / 8;
        int PT_bit_idx = i % 8;
        PT_info_bits[i] = (PT_frame_bytes[PT_byte_idx] >> PT_bit_idx) & 0x01;
    }
    
    for (int i = sizeof(MissileTelemetryFrame) * 8; i < IRIG_K; i++) {
        PT_info_bits[i] = 0;
    }
    
    LDPC_Encode(g_ldpc_encoder, PT_info_bits, PT_codeword);
    
    /* Step 4: 랜더마이저 */
    uint8_t *PT_randomized = malloc(IRIGFIX_LDPC_N * sizeof(uint8_t));
    LDPC_Randomize(PT_codeword, PT_randomized, IRIGFIX_LDPC_N);
    
    /* Step 5: ASM 추가 */
    int PT_total_bits = 64 + IRIGFIX_LDPC_N;
    uint8_t *PT_with_asm = malloc(PT_total_bits * sizeof(uint8_t));
    
    for (int i = 0; i < 64; i++) {
        int PT_byte_idx = i / 8;
        int PT_bit_idx = i % 8;
        PT_with_asm[i] = (LDPC_ASM_PATTERN[PT_byte_idx] >> PT_bit_idx) & 0x01;
    }
    
    memcpy(&PT_with_asm[64], PT_randomized, IRIGFIX_LDPC_N);
    
    /* Step 6: SOQPSK 변조 */
    int PT_output_length = PT_total_bits * IRIG_SAMPLES_PER_SYMBOL;
    float complex *PT_modulated = malloc(PT_output_length * sizeof(float complex));
    
    SOQPSK_Modulate(g_soqpsk_mod, PT_with_asm, PT_total_bits, PT_modulated);
    
    /* Step 7: RF 전송 */
    // RF_Transmit(PT_modulated, PT_output_length, PT_TX_POWER_W);
    
    PT_frames_transmitted++;
    
    free(PT_info_bits);
    free(PT_codeword);
    free(PT_randomized);
    free(PT_with_asm);
    free(PT_modulated);
}

/*────────────────────────────────────────────────────────────────────
 * Task 함수들
 ────────────────────────────────────────────────────────────────────*/

void Task_SensorSampling(void *PT_pvParameters)
{
    while (1) {
        MissileTM_ReadSensors(g_tm_system);
        
        PT_last_accel_magnitude = sqrtf(
            g_tm_system->current_frame.accel_x_g * g_tm_system->current_frame.accel_x_g +
            g_tm_system->current_frame.accel_y_g * g_tm_system->current_frame.accel_y_g +
            g_tm_system->current_frame.accel_z_g * g_tm_system->current_frame.accel_z_g
        );
        
        // vTaskDelay(pdMS_TO_TICKS(PT_SENSOR_SAMPLE_PERIOD_MS));
    }
}

void Task_DataTransmit(void *PT_pvParameters)
{
    while (1) {
        if (g_tm_system && g_tm_system->telemetry_active) {
            MissileTM_ProcessAndTransmit(g_tm_system);
        }
        
        // vTaskDelay(pdMS_TO_TICKS(PT_DATA_TX_PERIOD_MS));
    }
}

void Task_LaunchDetection(void *PT_pvParameters)
{
    while (1) {
        if (!g_tm_system->launch_detected) {
            if (MissileTM_DetectLaunch(g_tm_system)) {
                g_tm_system->launch_detected = true;
                g_tm_system->telemetry_active = true;
            }
        }
        
        // vTaskDelay(pdMS_TO_TICKS(PT_LAUNCH_DETECTION_PERIOD_MS));
    }
}

/*────────────────────────────────────────────────────────────────────
 * Main 함수
 ────────────────────────────────────────────────────────────────────*/

int main(void)
{
    printf("=== 미사일 텔레메트리 시스템 ===\n\n");
    
    printf("[PT_ - 프로젝트 튜닝 변수]\n");
    printf("  센서 샘플: %d ms\n", PT_SENSOR_SAMPLE_PERIOD_MS);
    printf("  데이터 전송: %d ms\n", PT_DATA_TX_PERIOD_MS);
    printf("  발사 임계값: %.1f G\n", PT_LAUNCH_ACCEL_THRESHOLD_G);
    printf("  송신 전력: %.1f W\n\n", PT_TX_POWER_W);
    
    printf("[IRIG_ - IRIG 106 표준 파라미터]\n");
    printf("  반송파: %.2f GHz\n", IRIG_CARRIER_FREQ_HZ / 1e9);
    printf("  데이터율: %.1f Mbps\n", IRIG_DATA_RATE_BPS / 1e6);
    printf("  LDPC: Rate 2/3\n\n");
    
    printf("[IRIGFIX_ - IRIG 106 고정 상수 (변경 금지)]\n");
    printf("  CPM ρ: %.2f\n", IRIGFIX_CPM_RHO);
    printf("  CPM B: %.2f\n\n", IRIGFIX_CPM_B);
    
    g_tm_system = MissileTM_Create();
    if (!g_tm_system) {
        printf("오류: 시스템 초기화 실패\n");
        return 1;
    }
    
    printf("시스템 초기화 성공\n\n");
    
    for (int i = 0; i < 5; i++) {
        printf("[프레임 %d]\n", i);
        MissileTM_ReadSensors(g_tm_system);
        
        if (MissileTM_DetectLaunch(g_tm_system)) {
            g_tm_system->launch_detected = true;
            g_tm_system->telemetry_active = true;
            printf("  🚀 발사 감지!\n");
        }
        
        if (g_tm_system->telemetry_active) {
            MissileTM_ProcessAndTransmit(g_tm_system);
            printf("  전송: %d 프레임\n", PT_frames_transmitted);
        }
        
        printf("  가속도: %.2f G\n\n", PT_last_accel_magnitude);
    }
    
    printf("=== 시스템 종료 ===\n");
    MissileTM_Destroy(g_tm_system);
    
    return 0;
}
```

---

## 🎯 역할 2: LDPC (오류 정정 코드)

### 파일 A: 2_ldpc_encoder.c

**역할**: 정보 비트 → 코드워드 변환 (오류 정정 추가)

**IRIG 근거**: Appendix R, Section R.5 "Encoder Implementation"

```c
/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * 파일: 2_ldpc_encoder.c
 * 
 * 역할: LDPC 인코딩
 * - 정보 비트 K=5461
 * - → 코드워드 N=8192
 * - 패리티 비트 M=2731 추가
 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/

#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>

/*════════════════════════════════════════════════════════════════════
 * IRIGFIX_ - IRIG 고정 상수
 ════════════════════════════════════════════════════════════════════*/

#define IRIGFIX_LDPC_N 8192           /* 코드워드 길이 */
#define IRIGFIX_LDPC_CIRCULANT_SIZE 128

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Encoder_Create
 ────────────────────────────────────────────────────────────────────*/
LDPC_Encoder* LDPC_Encoder_Create(LDPC_CodeRate PT_rate)
{
    LDPC_Encoder *PT_enc = malloc(sizeof(LDPC_Encoder));
    if (!PT_enc) return NULL;
    
    PT_enc->rate = PT_rate;
    PT_enc->N = IRIGFIX_LDPC_N;
    
    switch (PT_rate) {
        case LDPC_RATE_1_2: PT_enc->K = 4096; break;
        case LDPC_RATE_2_3: PT_enc->K = 5461; break;
        case LDPC_RATE_4_5: PT_enc->K = 6554; break;
    }
    
    PT_enc->M = PT_enc->N - PT_enc->K;
    
    PT_enc->proto_rows = PT_enc->M / IRIGFIX_LDPC_CIRCULANT_SIZE;
    PT_enc->proto_cols = PT_enc->N / IRIGFIX_LDPC_CIRCULANT_SIZE;
    
    PT_enc->proto_matrix = malloc(PT_enc->proto_rows * sizeof(int8_t*));
    for (int i = 0; i < PT_enc->proto_rows; i++) {
        PT_enc->proto_matrix[i] = malloc(PT_enc->proto_cols * sizeof(int8_t));
    }
    
    return PT_enc;
}

void LDPC_Encoder_Destroy(LDPC_Encoder *PT_enc)
{
    if (PT_enc) {
        for (int i = 0; i < PT_enc->proto_rows; i++) {
            free(PT_enc->proto_matrix[i]);
        }
        free(PT_enc->proto_matrix);
        free(PT_enc);
    }
}

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Encode
 * 
 * IRIG 근거: Section R.5 "Systematic Encoding"
 ────────────────────────────────────────────────────────────────────*/
void LDPC_Encode(LDPC_Encoder *PT_enc, const uint8_t *PT_info_bits, 
                 uint8_t *PT_codeword)
{
    if (!PT_enc || !PT_info_bits || !PT_codeword) return;
    
    /* Step 1: 정보 비트 복사 (체계적) */
    memcpy(PT_codeword, PT_info_bits, PT_enc->K);
    
    /* Step 2: 패리티 비트 계산 */
    uint8_t *PT_parity = &PT_codeword[PT_enc->K];
    memset(PT_parity, 0, PT_enc->M);
    
    int PT_z = IRIGFIX_LDPC_CIRCULANT_SIZE;
    
    for (int PT_p_block = 0; PT_p_block < PT_enc->proto_rows; PT_p_block++) {
        for (int PT_i_block = 0; PT_i_block < PT_enc->proto_cols; PT_i_block++) {
            int PT_shift = PT_enc->proto_matrix[PT_p_block][PT_i_block];
            
            if (PT_shift < 0) continue;
            
            for (int PT_k = 0; PT_k < PT_z; PT_k++) {
                int PT_src_idx = PT_i_block * PT_z + PT_k;
                int PT_dst_idx = PT_p_block * PT_z + ((PT_k + PT_shift) % PT_z);
                
                PT_parity[PT_dst_idx] ^= PT_info_bits[PT_src_idx];
            }
        }
    }
}
```

### 파일 B: 3_ldpc_decoder.c

**역할**: 수신 신호 → 추정 비트 (오류 정정)

**IRIG 근거**: Appendix R, Section R.6 "Decoder Requirements"

```c
/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * 파일: 3_ldpc_decoder.c
 * 
 * 역할: LDPC 디코딩
 * - Sum-Product 알고리즘
 * - 최대 50회 반복
 * - 패리티 체크로 조기 종료
 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/

#include "ldpc_codec.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*════════════════════════════════════════════════════════════════════
 * 프로젝트 튜닝 변수 (PT_)
 ════════════════════════════════════════════════════════════════════*/

#define PT_LDPC_DECODER_MAX_ITERATIONS 50      /* 반복 횟수 */
#define PT_LDPC_EARLY_TERMINATION 1            /* 조기 종료 */
#define PT_USE_ATANH_APPROXIMATION 1           /* 근사화 */
#define PT_MESSAGE_SCALING_FACTOR 1.0f         /* 메시지 스케일 */
#define PT_CONVERGENCE_MONITORING 1            /* 수렴 추적 */

/*════════════════════════════════════════════════════════════════════
 * IRIGFIX_ - IRIG 고정 상수
 ════════════════════════════════════════════════════════════════════*/

#define IRIGFIX_LDPC_CIRCULANT_SIZE 128
#define IRIGFIX_PROTO_ROWS 32
#define IRIGFIX_PROTO_COLS 64

/*────────────────────────────────────────────────────────────────────
 * 근사 tanh 함수
 ────────────────────────────────────────────────────────────────────*/
static float PT_tanh_fast(float PT_x)
{
    if (!PT_USE_ATANH_APPROXIMATION) {
        return tanhf(PT_x);
    }
    
    if (PT_x > 3.0f) return 1.0f;
    if (PT_x < -3.0f) return -1.0f;
    
    float PT_x_sq = PT_x * PT_x;
    return PT_x * (1.0f - PT_x_sq / 9.0f + PT_x_sq * PT_x_sq / 81.0f);
}

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Decoder_Create
 ────────────────────────────────────────────────────────────────────*/
LDPC_Decoder* LDPC_Decoder_Create(LDPC_CodeRate PT_rate)
{
    LDPC_Decoder *PT_dec = malloc(sizeof(LDPC_Decoder));
    if (!PT_dec) return NULL;
    
    PT_dec->rate = PT_rate;
    PT_dec->N = 8192;
    
    switch (PT_rate) {
        case LDPC_RATE_1_2: PT_dec->K = 4096; break;
        case LDPC_RATE_2_3: PT_dec->K = 5461; break;
        case LDPC_RATE_4_5: PT_dec->K = 6554; break;
    }
    
    PT_dec->M = PT_dec->N - PT_dec->K;
    
    int PT_num_edges = PT_dec->N * 3;
    PT_dec->edge_values = calloc(PT_num_edges, sizeof(float));
    PT_dec->check_to_var = calloc(PT_num_edges, sizeof(float));
    PT_dec->var_llr = malloc(PT_dec->N * sizeof(float));
    
    PT_dec->proto_rows = IRIGFIX_PROTO_ROWS;
    PT_dec->proto_cols = IRIGFIX_PROTO_COLS;
    
    return PT_dec;
}

void LDPC_Decoder_Destroy(LDPC_Decoder *PT_dec)
{
    if (PT_dec) {
        free(PT_dec->edge_values);
        free(PT_dec->check_to_var);
        free(PT_dec->var_llr);
        free(PT_dec);
    }
}

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Decode
 * 
 * IRIG 근거: Section R.6 "Iterative Belief Propagation"
 ────────────────────────────────────────────────────────────────────*/
bool LDPC_Decode(LDPC_Decoder *PT_dec, const float *PT_received_llr,
                 uint8_t *PT_decoded_bits, int PT_max_iterations)
{
    if (!PT_dec || !PT_received_llr || !PT_decoded_bits) return false;
    
    if (PT_max_iterations > PT_LDPC_DECODER_MAX_ITERATIONS) {
        PT_max_iterations = PT_LDPC_DECODER_MAX_ITERATIONS;
    }
    
    memcpy(PT_dec->var_llr, PT_received_llr, PT_dec->N * sizeof(float));
    memset(PT_dec->edge_values, 0, PT_dec->N * 3 * sizeof(float));
    
    int PT_z = IRIGFIX_LDPC_CIRCULANT_SIZE;
    uint8_t *PT_hard_decision = malloc(PT_dec->N * sizeof(uint8_t));
    
    for (int PT_iter = 0; PT_iter < PT_max_iterations; PT_iter++) {
        
        /* Check Node Update */
        for (int PT_c = 0; PT_c < PT_dec->proto_rows; PT_c++) {
            for (int PT_k = 0; PT_k < PT_z; PT_k++) {
                for (int PT_v = 0; PT_v < PT_dec->proto_cols; PT_v++) {
                    int PT_shift = PT_dec->proto_matrix[PT_c][PT_v];
                    if (PT_shift < 0) continue;
                    
                    int PT_idx = PT_v * PT_z + ((PT_k + PT_shift) % PT_z);
                    
                    float PT_product = 1.0f;
                    for (int PT_v2 = 0; PT_v2 < PT_dec->proto_cols; PT_v2++) {
                        int PT_shift2 = PT_dec->proto_matrix[PT_c][PT_v2];
                        if (PT_shift2 < 0 || PT_v2 == PT_v) continue;
                        
                        int PT_idx2 = PT_v2 * PT_z + ((PT_k + PT_shift2) % PT_z);
                        PT_product *= PT_tanh_fast(PT_dec->var_llr[PT_idx2] / 2.0f);
                    }
                    
                    PT_dec->check_to_var[PT_c * PT_dec->N + PT_idx] = 
                        2.0f * atanhf(PT_product * PT_MESSAGE_SCALING_FACTOR);
                }
            }
        }
        
        /* Variable Node Update */
        for (int PT_v = 0; PT_v < PT_dec->N; PT_v++) {
            float PT_sum = PT_received_llr[PT_v];
            
            int PT_v_block = PT_v / PT_z;
            for (int PT_c = 0; PT_c < PT_dec->proto_rows; PT_c++) {
                int PT_shift = PT_dec->proto_matrix[PT_c][PT_v_block];
                if (PT_shift >= 0) {
                    PT_sum += PT_dec->check_to_var[PT_c * PT_dec->N + PT_v];
                }
            }
            
            PT_dec->var_llr[PT_v] = PT_sum;
        }
        
        /* Hard Decision */
        for (int i = 0; i < PT_dec->N; i++) {
            PT_hard_decision[i] = (PT_dec->var_llr[i] > 0) ? 0 : 1;
        }
        
        /* Parity Check */
        if (PT_LDPC_EARLY_TERMINATION) {
            bool PT_all_ok = true;
            
            for (int PT_c = 0; PT_c < PT_dec->proto_rows && PT_all_ok; PT_c++) {
                for (int PT_k = 0; PT_k < PT_z && PT_all_ok; PT_k++) {
                    uint8_t PT_parity = 0;
                    
                    for (int PT_v = 0; PT_v < PT_dec->proto_cols; PT_v++) {
                        int PT_shift = PT_dec->proto_matrix[PT_c][PT_v];
                        if (PT_shift < 0) continue;
                        
                        int PT_idx = PT_v * PT_z + ((PT_k + PT_shift) % PT_z);
                        PT_parity ^= PT_hard_decision[PT_idx];
                    }
                    
                    if (PT_parity != 0) PT_all_ok = false;
                }
            }
            
            if (PT_all_ok) {
                memcpy(PT_decoded_bits, PT_hard_decision, PT_dec->K);
                free(PT_hard_decision);
                
                if (PT_CONVERGENCE_MONITORING) {
                    printf("[LDPC] 수렴: %d반복\n", PT_iter + 1);
                }
                
                return true;
            }
        }
    }
    
    memcpy(PT_decoded_bits, PT_hard_decision, PT_dec->K);
    free(PT_hard_decision);
    
    if (PT_CONVERGENCE_MONITORING) {
        printf("[LDPC] 최대 반복 도달 (%d)\n", PT_LDPC_DECODER_MAX_ITERATIONS);
    }
    
    return false;
}
```

### 파일 C: 4_ldpc_randomizer.c

**역할**: 의사난수 생성으로 0/1 균형 유지 (심볼 동기화)

**IRIG 근거**: Appendix R, Section R.7 "Randomizer"

```c
/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * 파일: 4_ldpc_randomizer.c
 * 
 * 역할: LDPC 랜더마이저
 * - LFSR로 의사난수 생성
 * - 0/1 균형 보장
 * - 수신기 심볼 동기화 필수
 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/

#include "ldpc_codec.h"
#include <stdint.h>

/*════════════════════════════════════════════════════════════════════
 * 프로젝트 튜닝 변수 (PT_)
 ════════════════════════════════════════════════════════════════════*/

#define PT_LFSR_INITIAL_SEED 0xACE1        /* LFSR 초기값 */
#define PT_RANDOMIZER_STATISTICS_ENABLE 0  /* 통계 추적 */

/*════════════════════════════════════════════════════════════════════
 * IRIGFIX_ - IRIG 고정 상수
 ════════════════════════════════════════════════════════════════════*/

#define IRIGFIX_LFSR_POLY 0xB400          /* 다항식 (절대 변경 금지) */
#define IRIGFIX_LFSR_WIDTH 16

/*────────────────────────────────────────────────────────────────────
 * LFSR 전역 상태
 ────────────────────────────────────────────────────────────────────*/
static uint16_t PT_lfsr_state = PT_LFSR_INITIAL_SEED;

#if PT_RANDOMIZER_STATISTICS_ENABLE
static uint32_t PT_stat_zeros = 0;
static uint32_t PT_stat_ones = 0;
#endif

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Randomizer_Init
 ────────────────────────────────────────────────────────────────────*/
void LDPC_Randomizer_Init(uint32_t PT_seed)
{
    if (PT_seed == 0) {
        PT_lfsr_state = PT_LFSR_INITIAL_SEED;
    } else {
        PT_lfsr_state = PT_seed & 0xFFFF;
    }
}

/*────────────────────────────────────────────────────────────────────
 * 함수: lfsr_next_bit (내부)
 * 
 * IRIG 근거: Section R.7
 * 다항식: x^16 + x^15 + x^13 + x^4 + 1
 ────────────────────────────────────────────────────────────────────*/
static uint8_t IRIGFIX_lfsr_next_bit(void)
{
    uint8_t PT_output_bit = PT_lfsr_state & 0x0001;
    
    uint16_t PT_feedback = 0;
    if (PT_lfsr_state & (1 << 15)) PT_feedback ^= 1;
    if (PT_lfsr_state & (1 << 14)) PT_feedback ^= 1;
    if (PT_lfsr_state & (1 << 12)) PT_feedback ^= 1;
    if (PT_lfsr_state & (1 <<  3)) PT_feedback ^= 1;
    
    PT_lfsr_state = (PT_lfsr_state >> 1) | (PT_feedback << 15);
    
    return PT_output_bit;
}

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Randomize
 ────────────────────────────────────────────────────────────────────*/
void LDPC_Randomize(const uint8_t *PT_input, uint8_t *PT_output, int PT_length)
{
    if (!PT_input || !PT_output) return;
    
    for (int i = 0; i < PT_length; i++) {
        uint8_t PT_rand_bit = IRIGFIX_lfsr_next_bit();
        PT_output[i] = PT_input[i] ^ PT_rand_bit;
        
        #if PT_RANDOMIZER_STATISTICS_ENABLE
        if (PT_output[i] == 0) PT_stat_zeros++;
        else PT_stat_ones++;
        #endif
    }
}

/*────────────────────────────────────────────────────────────────────
 * 함수: LDPC_Derandomize
 ────────────────────────────────────────────────────────────────────*/
void LDPC_Derandomize(const uint8_t *PT_input, uint8_t *PT_output, int PT_length)
{
    LDPC_Randomize(PT_input, PT_output, PT_length);
}

#if PT_RANDOMIZER_STATISTICS_ENABLE
void LDPC_Randomizer_GetStatistics(uint32_t *PT_zeros, uint32_t *PT_ones)
{
    if (PT_zeros) *PT_zeros = PT_stat_zeros;
    if (PT_ones) *PT_ones = PT_stat_ones;
    
    printf("[Randomizer] 0: %u (%.2f%%), 1: %u (%.2f%%)\n",
           PT_stat_zeros, 100.0f * PT_stat_zeros / (PT_stat_zeros + PT_stat_ones),
           PT_stat_ones, 100.0f * PT_stat_ones / (PT_stat_zeros + PT_stat_ones));
}
#endif
```

---

## 🎯 역할 3: SOQPSK (변조/복조)

(이전 Part 1 파일 5, 6 - 변수 프리픽스 적용)

---

## 🎯 역할 4: SENSOR (센서 수집)

(이전 Part 1 파일 1 - 변수 프리픽스 미적용, 필요시 PT_ 추가 가능)

---

## 📊 역할별 변수 요약

| 역할 | 파일 | PT_ 개수 | IRIG_ 개수 | IRIGFIX_ 개수 |
|------|------|---------|-----------|--------------|
| **MAIN_INTEGRATION** | 7 | 8 | 5 | 3 |
| **LDPC (Encoder)** | 2 | 0 | 0 | 2 |
| **LDPC (Decoder)** | 3 | 4 | 0 | 3 |
| **LDPC (Randomizer)** | 4 | 2 | 0 | 2 |
| **SOQPSK (Modulator)** | 5 | - | - | - |
| **SOQPSK (Demodulator)** | 6 | - | - | - |
| **SENSOR** | 1 | - | - | - |

---

**역할별 파일 분류 + 변수 프리픽스 완성!** ✨

이제 각 역할의 파일이 명확하고, 각 파일의 변수가 PT_ / IRIG_ / IRIGFIX_로 구분되어 있습니다!
