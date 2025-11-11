#include "missile_telemetry.h"
#include "ldpc_codec.h"
#include "soqpsk.h"
#include "data_storage.h"
#include "camera_interface.h"
#include "ground_control.h"
#include "emergency_system.h"
#include "telemetry_config.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define PT_SENSOR_SAMPLE_PERIOD_MS 1
#define PT_DATA_TX_PERIOD_MS 10
#define PT_LAUNCH_DETECTION_PERIOD_MS 100
#define PT_LAUNCH_ACCEL_THRESHOLD_G 5.0f
#define PT_LAUNCH_SUSTAINED_SAMPLES 10
#define PT_PLL_BANDWIDTH_SCALE 0.01f
#define PT_PLL_DAMPING_FACTOR 0.707f
#define PT_LDPC_DECODER_MAX_ITERATIONS 50
#define PT_LDPC_EARLY_TERMINATION_ENABLE 1
#define PT_MESSAGE_SCALING_FACTOR 1.0f
#define PT_TX_POWER_W 3.0f
#define PT_BATTERY_LOW_VOLTAGE_V 10.0f
#define PT_TEMPERATURE_HIGH_LIMIT_C 85.0f
#define PT_IMU_ACCEL_SCALE 100.0f
#define PT_IMU_GYRO_SCALE 2000.0f
#define PT_CONFIG_UPDATE_PERIOD_MS 5000

static MissileTelemetrySystem *g_tm_system = NULL;
static LDPC_Encoder *g_ldpc_encoder = NULL;
static LDPC_Decoder *g_ldpc_decoder = NULL;
static SOQPSK_Modulator *g_soqpsk_mod = NULL;
static SOQPSK_Demodulator *g_soqpsk_demod = NULL;

static LogBuffer *g_log_buffer = NULL;
static CameraDevice *g_camera = NULL;
static ControlState g_control_state = {0};
static EmergencyState *g_emergency_state = NULL;
static ConfigSet *g_config = NULL;

static uint32_t g_frames_transmitted = 0;
static uint32_t g_frames_received = 0;
static float g_last_accel_magnitude = 0.0f;

int MissileTM_InitializeSystem(void)
{
    printf("========================================\n");
    printf("미사일 텔레메트리 시스템 v3 초기화\n");
    printf("========================================\n\n");
    
    printf("[INIT] 시스템 구조 초기화...\n");
    g_tm_system = malloc(sizeof(MissileTelemetrySystem));
    if (!g_tm_system) {
        printf("오류: 시스템 구조 초기화 실패\n");
        return -1;
    }
    memset(g_tm_system, 0, sizeof(MissileTelemetrySystem));
    g_tm_system->system_armed = false;
    g_tm_system->launch_detected = false;
    g_tm_system->telemetry_active = false;
    
    printf("[INIT] SOQPSK 모듈 초기화...\n");
    g_soqpsk_mod = malloc(sizeof(SOQPSK_Modulator));
    if (!g_soqpsk_mod) {
        printf("오류: SOQPSK 변조기 초기화 실패\n");
        return -1;
    }
    memset(g_soqpsk_mod, 0, sizeof(SOQPSK_Modulator));
    
    g_soqpsk_demod = malloc(sizeof(SOQPSK_Demodulator));
    if (!g_soqpsk_demod) {
        printf("오류: SOQPSK 복조기 초기화 실패\n");
        return -1;
    }
    memset(g_soqpsk_demod, 0, sizeof(SOQPSK_Demodulator));
    
    printf("[INIT] LDPC 코덱 초기화...\n");
    g_ldpc_encoder = malloc(sizeof(LDPC_Encoder));
    if (!g_ldpc_encoder) {
        printf("오류: LDPC 인코더 초기화 실패\n");
        return -1;
    }
    memset(g_ldpc_encoder, 0, sizeof(LDPC_Encoder));
    
    g_ldpc_decoder = malloc(sizeof(LDPC_Decoder));
    if (!g_ldpc_decoder) {
        printf("오류: LDPC 디코더 초기화 실패\n");
        return -1;
    }
    memset(g_ldpc_decoder, 0, sizeof(LDPC_Decoder));
    
    printf("[INIT] 데이터 저장소 초기화...\n");
    g_log_buffer = DataStorage_Init(10000);
    if (!g_log_buffer) {
        printf("오류: 데이터 저장소 초기화 실패\n");
        return -1;
    }
    
    printf("[INIT] 카메라 초기화...\n");
    g_camera = Camera_Init();
    if (!g_camera) {
        printf("경고: 카메라 초기화 실패 (계속 진행)\n");
    } else {
        Camera_Start(g_camera);
    }
    
    printf("[INIT] 긴급 시스템 초기화...\n");
    g_emergency_state = EmergencySystem_Init();
    if (!g_emergency_state) {
        printf("오류: 긴급 시스템 초기화 실패\n");
        return -1;
    }
    
    printf("[INIT] 설정 시스템 초기화...\n");
    g_config = TelemetryConfig_Init();
    if (!g_config) {
        printf("오류: 설정 시스템 초기화 실패\n");
        return -1;
    }
    
    if (g_config) {
        TelemetryConfig_RegisterIntParam(g_config, 0,
            "PT_SENSOR_SAMPLE_PERIOD_MS", 1, 1, 100);
        TelemetryConfig_RegisterIntParam(g_config, 1,
            "PT_DATA_TX_PERIOD_MS", 10, 5, 100);
        TelemetryConfig_RegisterFloatParam(g_config, 2,
            "PT_LAUNCH_ACCEL_THRESHOLD_G", 5.0f, 1.0f, 10.0f);
        TelemetryConfig_RegisterFloatParam(g_config, 3,
            "PT_PLL_BANDWIDTH_SCALE", 0.01f, 0.001f, 0.1f);
        TelemetryConfig_RegisterIntParam(g_config, 4,
            "PT_LDPC_DECODER_MAX_ITERATIONS", 50, 10, 100);
    }
    
    memset(&g_control_state, 0, sizeof(ControlState));
    g_control_state.is_command_valid = true;
    g_control_state.current_thrust = 0.0f;
    
    printf("\n시스템 초기화 완료!\n\n");
    
    return 0;
}

void MissileTM_ShutdownSystem(void)
{
    printf("\n========================================\n");
    printf("시스템 종료 중...\n");
    printf("========================================\n");
    
    if (g_camera) {
        Camera_Stop(g_camera);
        Camera_Destroy(g_camera);
        g_camera = NULL;
    }
    
    if (g_tm_system) {
        free(g_tm_system);
        g_tm_system = NULL;
    }
    
    if (g_soqpsk_mod) {
        free(g_soqpsk_mod);
        g_soqpsk_mod = NULL;
    }
    
    if (g_soqpsk_demod) {
        free(g_soqpsk_demod);
        g_soqpsk_demod = NULL;
    }
    
    if (g_ldpc_encoder) {
        free(g_ldpc_encoder);
        g_ldpc_encoder = NULL;
    }
    
    if (g_ldpc_decoder) {
        free(g_ldpc_decoder);
        g_ldpc_decoder = NULL;
    }
    
    if (g_log_buffer) {
        DataStorage_Destroy(g_log_buffer);
        g_log_buffer = NULL;
    }
    
    if (g_emergency_state) {
        EmergencySystem_Destroy(g_emergency_state);
        g_emergency_state = NULL;
    }
    
    if (g_config) {
        TelemetryConfig_Destroy(g_config);
        g_config = NULL;
    }
    
    printf("시스템 종료 완료\n");
    printf("========================================\n\n");
}

void MissileTM_MainLoop(void)
{
    printf("========================================\n");
    printf("메인 루프 시작\n");
    printf("========================================\n\n");
    
    uint32_t loop_count = 0;
    uint32_t max_iterations = 100;
    
    while (loop_count < max_iterations) {
        loop_count++;
        
        if (g_tm_system) {
            g_tm_system->current_frame.frame_counter++;
            g_tm_system->current_frame.timestamp_us += 1000;
        }
        
        if (loop_count == 1) {
            if (g_tm_system) {
                g_tm_system->launch_detected = true;
                g_tm_system->telemetry_active = true;
                printf("[LAUNCH] 발사 감지!\n\n");
            }
        }
        
        if (g_tm_system && g_tm_system->telemetry_active && g_log_buffer) {
            LogEntry log_entry;
            memset(&log_entry, 0, sizeof(LogEntry));
            log_entry.entry_id = g_log_buffer->buffer_count;
            log_entry.timestamp_us = g_tm_system->current_frame.timestamp_us;
            memcpy(&log_entry.telemetry, &g_tm_system->current_frame,
                   sizeof(MissileTelemetryFrame));
            
            DataStorage_WriteEntry(g_log_buffer, &log_entry);
            g_frames_transmitted++;
        }
        
        if (loop_count % 10 == 0) {
            printf("[%d ms] TX: %d, Log: %d\n",
                   loop_count, g_frames_transmitted,
                   g_log_buffer ? DataStorage_GetEntryCount(g_log_buffer) : 0);
        }
        
        if (g_emergency_state && EmergencySystem_IsInEmergency(g_emergency_state)) {
            printf("[EMERGENCY] 긴급 모드 활성화\n");
            break;
        }
    }
    
    printf("\n메인 루프 종료\n");
}

int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("미사일 텔레메트리 시스템 v3\n");
    printf("========================================\n\n");
    
    if (MissileTM_InitializeSystem() != 0) {
        printf("시스템 초기화 실패\n");
        MissileTM_ShutdownSystem();
        return 1;
    }
    
    MissileTM_MainLoop();
    
    MissileTM_ShutdownSystem();
    
    printf("\n프로그램 종료\n\n");
    
    return 0;
}
