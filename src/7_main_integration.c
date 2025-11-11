#include "missile_telemetry.h"
#include "ldpc_codec.h"
#include "soqpsk.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* ============================================================
 * PT_ (프로젝트 튜닝) - 자유롭게 변경 가능
 * ============================================================ */

/* 센서 샘플링 주기 */
#define PT_SENSOR_SAMPLE_PERIOD_MS 1
    /* 변경 가능: 1~100 ms
     * 작을수록: 더 자주 샘플링 (높은 정확도, 높은 전력)
     * 클수록: 덜 자주 샘플링 (낮은 전력)
     */

/* 데이터 전송 주기 */
#define PT_DATA_TX_PERIOD_MS 10
    /* 변경 가능: 5~100 ms
     * 작을수록: 더 자주 전송 (높은 데이터율)
     * 클수록: 덜 자주 전송 (낮은 대역폭)
     */

/* 발사 감지 관련 */
#define PT_LAUNCH_DETECTION_PERIOD_MS 100
    /* 변경 가능: 50~500 ms */

#define PT_LAUNCH_ACCEL_THRESHOLD_G 5.0f
    /* 변경 가능: 1.0~10.0 G
     * 작을수록: 더 민감한 감지 (오감지 가능성↑)
     * 클수록: 덜 민감한 감지 (감지 실패 가능성↑)
     */

#define PT_LAUNCH_SUSTAINED_SAMPLES 10
    /* 변경 가능: 5~50 샘플
     * 작을수록: 빠른 감지
     * 클수록: 더 안정적인 감지
     */

/* PLL (Phase Lock Loop) 튜닝 */
#define PT_PLL_BANDWIDTH_SCALE 0.01f
    /* 변경 가능: 0.001~0.1
     * 작을수록: 느린 추적 (안정적, 낮은 도플러에 강함)
     * 클수록: 빠른 추적 (민감, 높은 도플러에 약함)
     */

#define PT_PLL_DAMPING_FACTOR 0.707f
    /* 변경 가능: 0.5~1.0
     * 0.707 (권장): 임계 감쇠 (가장 안정적)
     * <0.707: 과도응답 (빠름)
     * >0.707: 저조응답 (느림)
     */

/* LDPC 처리 */
#define PT_LDPC_MAX_ITERATIONS 50
    /* 변경 가능: 10~100
     * 적을수록: 빠른 처리, 낮은 BER 개선
     * 많을수록: 느린 처리, 높은 BER 개선
     */

#define PT_LDPC_EARLY_TERMINATION_ENABLE 1
    /* 변경 가능: 0 또는 1
     * 0: 모든 반복 실행
     * 1: 패리티 만족 시 즉시 종료 (평균 30% 가속)
     */

/* 전력 설정 */
#define PT_TX_POWER_W 3.0f
    /* 변경 가능: 0.1~10.0 W
     * 낮을수록: 에너지 절약, 짧은 거리
     * 높을수록: 에너지 소비↑, 긴 거리
     */

#define PT_BATTERY_LOW_VOLTAGE_V 10.0f
    /* 변경 가능: 8.0~16.0 V
     * 배터리 전압이 이 값 이하면 경고
     */

#define PT_TEMPERATURE_HIGH_LIMIT_C 85.0f
    /* 변경 가능: 50~120°C
     * 이 값을 초과하면 시스템 경고
     */

/* ============================================================
 * IRIG_ (IRIG 106 표준) - 신중하게 변경 (지상국과 동기화 필수)
 * ============================================================ */

#define IRIG_CARRIER_FREQ_HZ IRIGFIX_CARRIER_FREQ
    /* 변경 가능: 2.0~2.5 GHz (권장: 2.35 GHz)
     * 지상국 수신기도 동일하게 변경 필수!
     */

#define IRIG_SAMPLE_RATE_HZ IRIGFIX_SAMPLE_RATE
    /* 변경 가능: 40~160 MHz (권장: 80 MHz)
     * LDPC 코드 길이와 호환성 유지 필요
     */

#define IRIG_DATA_RATE_BPS IRIGFIX_DATA_RATE
    /* 변경 가능: 1~100 Mbps (권장: 10 Mbps)
     * 채널 대역폭 제약 고려
     */

#define IRIG_LDPC_CODE_RATE LDPC_RATE_2_3
    /* IRIG 106 권장: RATE_2_3
     * RATE_1_2: 더 강한 오류 정정 (속도 느림)
     * RATE_2_3: 균형잡힌 성능 (권장)
     * RATE_4_5: 더 빠른 데이터율 (오류 정정 약함)
     */

#define IRIG_SAMPLES_PER_SYMBOL IRIGFIX_SAMPLES_PER_SYMBOL
    /* IRIG 106 권장: 8 */

/* ============================================================
 * IRIGFIX_ (IRIG 106 고정 상수) - 절대 변경 금지!
 * ============================================================ */

#define IRIGFIX_CPM_RHO_VAL IRIGFIX_CPM_RHO
    /* IRIG 106 Appendix M 정의: 0.70 (변경 금지) */

#define IRIGFIX_CPM_B_VAL IRIGFIX_CPM_B
    /* IRIG 106 Appendix M 정의: 1.25 (변경 금지) */

#define IRIGFIX_LDPC_N_VAL IRIGFIX_LDPC_N
    /* IRIG 106 Appendix R 정의: 8192 (변경 금지) */

/* ============================================================
 * 전역 변수
 * ============================================================ */

static MissileTelemetrySystem *g_tm_system = NULL;
static LDPC_Encoder *g_ldpc_encoder = NULL;
static LDPC_Decoder *g_ldpc_decoder = NULL;
static SOQPSK_Modulator *g_soqpsk_mod = NULL;

static uint32_t PT_frames_transmitted = 0;
static uint32_t PT_total_errors = 0;
static float PT_last_accel_magnitude = 0.0f;

/* ============================================================
 * 함수: MissileTM_Create
 * 
 * 목적: 시스템 초기화
 * 파라미터: 없음
 * 반환: MissileTelemetrySystem 포인터
 * ============================================================ */

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

/* ============================================================
 * 함수: MissileTM_ProcessAndTransmit
 * 
 * 목적: 데이터 처리 및 전송
 * 파이프라인:
 *   1. 센서 수집
 *   2. CRC 계산
 *   3. LDPC 인코딩
 *   4. 랜더마이저
 *   5. ASM 추가
 *   6. SOQPSK 변조
 *   7. RF 전송
 * ============================================================ */

void MissileTM_ProcessAndTransmit(MissileTelemetrySystem *sys)
{
    if (!sys || !g_ldpc_encoder || !g_soqpsk_mod) return;
    
    /* Step 1: 센서 수집 */
    MissileTM_ReadSensors(sys);
    
    /* Step 2: CRC 계산 */
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
    
    /* Step 3: LDPC 인코딩 */
    int K = g_ldpc_encoder->K;
    uint8_t *info_bits = malloc(K * sizeof(uint8_t));
    uint8_t *codeword = malloc(IRIGFIX_LDPC_N * sizeof(uint8_t));
    
    for (int i = 0; i < K && i < frame_size * 8; i++) {
        int byte_idx = i / 8;
        int bit_idx = i % 8;
        info_bits[i] = (frame_bytes[byte_idx] >> bit_idx) & 0x01;
    }
    
    for (int i = frame_size * 8; i < K; i++) {
        info_bits[i] = 0;
    }
    
    LDPC_Encode(g_ldpc_encoder, info_bits, codeword);
    
    /* Step 4: 랜더마이저 */
    uint8_t *randomized = malloc(IRIGFIX_LDPC_N * sizeof(uint8_t));
    LDPC_Randomize(codeword, randomized, IRIGFIX_LDPC_N);
    
    /* Step 5: ASM 추가 */
    int total_bits = 64 + IRIGFIX_LDPC_N;
    uint8_t *with_asm = malloc(total_bits * sizeof(uint8_t));
    
    for (int i = 0; i < 64; i++) {
        int byte_idx = i / 8;
        int bit_idx = i % 8;
        with_asm[i] = (LDPC_ASM_PATTERN[byte_idx] >> bit_idx) & 0x01;
    }
    
    memcpy(&with_asm, randomized, IRIGFIX_LDPC_N);
    
    /* Step 6: SOQPSK 변조 */
    int output_length = total_bits * IRIG_SAMPLES_PER_SYMBOL;
    float_complex *modulated = malloc(output_length * sizeof(float_complex));
    
    SOQPSK_Modulate(g_soqpsk_mod, with_asm, total_bits, modulated);
    
    /* Step 7: RF 전송 (실제 RF 드라이버 호출) */
    /* RF_Transmit(modulated, output_length, PT_TX_POWER_W); */
    
    PT_frames_transmitted++;
    
    free(info_bits);
    free(codeword);
    free(randomized);
    free(with_asm);
    free(modulated);
}

/* ============================================================
 * Task 함수들
 * ============================================================ */

void Task_SensorSampling(void *pvParameters)
{
    while (1) {
        MissileTM_ReadSensors(g_tm_system);
        
        PT_last_accel_magnitude = sqrtf(
            g_tm_system->current_frame.accel_x_g * g_tm_system->current_frame.accel_x_g +
            g_tm_system->current_frame.accel_y_g * g_tm_system->current_frame.accel_y_g +
            g_tm_system->current_frame.accel_z_g * g_tm_system->current_frame.accel_z_g
        );
        
        /* vTaskDelay(pdMS_TO_TICKS(PT_SENSOR_SAMPLE_PERIOD_MS)); */
    }
}

void Task_DataTransmit(void *pvParameters)
{
    while (1) {
        if (g_tm_system && g_tm_system->telemetry_active) {
            MissileTM_ProcessAndTransmit(g_tm_system);
        }
        
        /* vTaskDelay(pdMS_TO_TICKS(PT_DATA_TX_PERIOD_MS)); */
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
        
        /* vTaskDelay(pdMS_TO_TICKS(PT_LAUNCH_DETECTION_PERIOD_MS)); */
    }
}

/* ============================================================
 * Main 함수
 * ============================================================ */

int main(void)
{
    printf("=== Missile Telemetry System ===\n\n");
    
    printf("[PT_ - Project Tuning (자유롭게 변경 가능)]\n");
    printf("  센서 샘플: %d ms\n", PT_SENSOR_SAMPLE_PERIOD_MS);
    printf("  데이터 전송: %d ms\n", PT_DATA_TX_PERIOD_MS);
    printf("  발사 임계값: %.1f G\n", PT_LAUNCH_ACCEL_THRESHOLD_G);
    printf("  송신 전력: %.1f W\n\n", PT_TX_POWER_W);
    
    printf("[IRIG_ - Standard Parameters (신중하게 변경)]\n");
    printf("  반송파: %.2f GHz\n", IRIG_CARRIER_FREQ_HZ / 1e9);
    printf("  데이터율: %.1f Mbps\n", IRIG_DATA_RATE_BPS / 1e6);
    printf("  LDPC: Rate 2/3\n\n");
    
    printf("[IRIGFIX_ - Fixed Constants (변경 금지!)]\n");
    printf("  CPM ρ: %.2f\n", IRIGFIX_CPM_RHO_VAL);
    printf("  CPM B: %.2f\n", IRIGFIX_CPM_B_VAL);
    printf("  LDPC N: %d\n\n", IRIGFIX_LDPC_N_VAL);
    
    g_tm_system = MissileTM_Create();
    if (!g_tm_system) {
        printf("오류: 시스템 초기화 실패\n");
        return 1;
    }
    
    printf("시스템 초기화 성공\n\n");
    
    for (int i = 0; i < 5; i++) {
        printf("[Frame %d]\n", i);
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
