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

/* ============================================================
 * PT_ (Project Tuning) - 프로젝트별 튜닝 변수
 * ============================================================ */

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

/* ============================================================
 * 전역 변수
 * ============================================================ */

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

/* ============================================================
 * 시스템 초기화
 * ============================================================ */

int MissileTM_InitializeSystem(void)
{
    printf("========================================\n");
    printf("미사일 텔레메트리 시스템 v3 초기화\n");
    printf("========================================\n\n");
    
    /* 1. 시스템 구조 초기화 */
    printf("[INIT] 시스템 구조 초기화...\n");
    g_tm_system = malloc(sizeof(MissileTelemetrySystem));
    if (!g_tm_system) {
        printf(" 오류: 시스템 구조 초기화 실패\n");
        return -1;
    }
    memset(g_tm_system, 0, sizeof(MissileTelemetrySystem));
    
    /* 2. SOQPSK 변조/복조 초기화 */
    printf("[INIT] SOQPSK 모듈 초기화...\n");
    g_soqpsk_mod = malloc(sizeof(SOQPSK_Modulator));
    if (!g_soqpsk_mod) {
        printf(" 오류: SOQPSK 변조기 초기화 실패\n");
        return -1;
    }
    
    g_soqpsk_demod = malloc(sizeof(SOQPSK_Demodulator));
    if (!g_soqpsk_demod) {
        printf(" 오류: SOQPSK 복조기 초기화 실패\n");
        return -1;
    }
    
    /* 3. LDPC 코덱 초기화 */
    printf("[INIT] LDPC 코덱 초기화...\n");
    g_ldpc_encoder = malloc(sizeof(LDPC_Encoder));
    if (!g_ldpc_encoder) {
        printf(" 오류: LDPC 인코더 초기화 실패\n");
        return -1;
    }
    
    g_ldpc_decoder = malloc(sizeof(LDPC_Decoder));
    if (!g_ldpc_decoder) {
        printf(" 오류: LDPC 디코더 초기화 실패\n");
        return -1;
    }
    
    /* 4. 데이터 저장소 초기화 */
    printf("[INIT] 데이터 저장소 초기화 (10000 엔트리)...\n");
    g_log_buffer = DataStorage_Init(10000);
    if (!g_log_buffer) {
        printf(" 오류: 데이터 저장소 초기화 실패\n");
        return -1;
    }
    
    /* 5. 카메라 초기화 */
    printf("[INIT] 카메라 초기화...\n");
    g_camera = Camera_Init();
    if (!g_camera) {
        printf("  경고: 카메라 초기화 실패 (계속 진행)\n");
    } else {
        Camera_Start(g_camera);
    }
    
    /* 6. 긴급 시스템 초기화 */
    printf("[INIT] 긴급 시스템 초기화...\n");
    g_emergency_state = EmergencySystem_Init();
    if (!g_emergency_state) {
        printf(" 오류: 긴급 시스템 초기화 실패\n");
        return -1;
    }
    
    /* 7. 설정 시스템 초기화 */
    printf("[INIT] 설정 시스템 초기화...\n");
    g_config = TelemetryConfig_Init();
    if (!g_config) {
        printf(" 오류: 설정 시스템 초기화 실패\n");
        return -1;
    }
    
    /* 파라미터 등록 */
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
    TelemetryConfig_RegisterIntParam(g_config, 10,
        "PT_CAMERA_FPS", 10, 1, 60);
    TelemetryConfig_RegisterFloatParam(g_config, 20,
        "PT_THRUST_MAX", 100.0f, 1.0f, 1000.0f);
    TelemetryConfig_RegisterFloatParam(g_config, 21,
        "PT_RUDDER_MAX_ANGLE", 45.0f, 5.0f, 90.0f);
    
    /* 제어 상태 초기화 */
    memset(&g_control_state, 0, sizeof(ControlState));
    g_control_state.is_command_valid = true;
    g_control_state.current_thrust = 0.0f;
    
    printf("\n 시스템 초기화 완료!\n\n");
    
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
    }
    
    if (g_tm_system) {
        free(g_tm_system);
    }
    
    if (g_soqpsk_mod) {
        free(g_soqpsk_mod);
    }
    
    if (g_soqpsk_demod) {
        free(g_soqpsk_demod);
    }
    
    if (g_ldpc_encoder) {
        free(g_ldpc_encoder);
    }
    
    if (g_ldpc_decoder) {
        free(g_ldpc_decoder);
    }
    
    if (g_log_buffer) {
        DataStorage_Destroy(g_log_buffer);
    }
    
    if (g_emergency_state) {
        EmergencySystem_Destroy(g_emergency_state);
    }
    
    if (g_config) {
        TelemetryConfig_Destroy(g_config);
    }
    
    printf(" 시스템 종료 완료\n");
    printf("========================================\n\n");
}

/* ============================================================
 * 메인 루프
 * ============================================================ */

void MissileTM_MainLoop(void)
{
    printf("========================================\n");
    printf("메인 루프 시작\n");
    printf("========================================\n\n");
    
    uint32_t sensor_timer = 0;
    uint32_t tx_timer = 0;
    uint32_t launch_timer = 0;
    uint32_t config_sync_timer = 0;
    uint32_t loop_count = 0;
    
    while (1) {
        loop_count++;
        
        /* ========== 센서 수집 (1ms 주기) ========== */
        if (sensor_timer >= PT_SENSOR_SAMPLE_PERIOD_MS) {
            if (g_tm_system) {
                g_tm_system->current_frame.frame_counter++;
                g_tm_system->current_frame.timestamp_us += PT_SENSOR_SAMPLE_PERIOD_MS * 1000;
            }
            sensor_timer = 0;
        }
        
        /* ========== 발사 감지 (100ms 주기) ========== */
        if (launch_timer >= PT_LAUNCH_DETECTION_PERIOD_MS) {
            if (g_tm_system) {
                float accel_mag = sqrt(
                    g_tm_system->current_frame.accel_x_g * g_tm_system->current_frame.accel_x_g +
                    g_tm_system->current_frame.accel_y_g * g_tm_system->current_frame.accel_y_g +
                    g_tm_system->current_frame.accel_z_g * g_tm_system->current_frame.accel_z_g
                );
                g_last_accel_magnitude = accel_mag;
                
                if (accel_mag > PT_LAUNCH_ACCEL_THRESHOLD_G) {
                    g_tm_system->launch_detected = true;
                    g_tm_system->telemetry_active = true;
                    printf("\n🚀 [LAUNCH] 발사 감지! (가속도: %.2f G)\n\n", accel_mag);
                }
            }
            launch_timer = 0;
        }
        
        /* ========== 데이터 전송 (10ms 주기) ========== */
        if (tx_timer >= PT_DATA_TX_PERIOD_MS && g_tm_system && g_tm_system->telemetry_active) {
            if (g_log_buffer) {
                LogEntry log_entry;
                log_entry.entry_id = g_log_buffer->buffer_count;
                log_entry.timestamp_us = g_tm_system->current_frame.timestamp_us;
                memcpy(&log_entry.telemetry, &g_tm_system->current_frame,
                       sizeof(MissileTelemetryFrame));
                log_entry.last_command_type = g_control_state.is_command_valid ? 1 : 0;
                log_entry.last_thrust_cmd = g_control_state.current_thrust;
                
                DataStorage_WriteEntry(g_log_buffer, &log_entry);
            }
            
            if (g_camera) {
                CameraFrame *frame = Camera_CaptureFrame(g_camera);
                if (frame && g_log_buffer) {
                    DataStorage_WriteCameraFrame(g_log_buffer, frame->frame_id,
                                                frame->data, frame->data_size);
                    Camera_ReleaseFrame(frame);
                }
            }
            
            g_frames_transmitted++;
            tx_timer = 0;
        }
        
        /* ========== 지상국 명령 수신 ========== */
        uint8_t rx_buffer[1024];
        uint32_t rx_len = 0;
        
        if (rx_len > 0) {
            uint16_t msg_header = *(uint16_t*)rx_buffer;
            
            if (msg_header == 0x4346) {
                ConfigUpdateMessage *update_msg = (ConfigUpdateMessage*)rx_buffer;
                if (TelemetryConfig_ProcessUpdateMessage(g_config, update_msg)) {
                    printf("[CONFIG] 설정 변경 수신 및 적용\n");
                    TelemetryConfig_SyncToHardware(g_config);
                }
            }
            else if (msg_header == 0x4352) {
                ConfigResponseMessage *response = 
                    TelemetryConfig_GenerateResponseMessage(g_config);
                if (response) {
                    free(response);
                }
            }
            else if (msg_header == 0x4354) {
                GroundControlCommand *cmd = (GroundControlCommand*)rx_buffer;
                if (GroundControl_ProcessCommand(cmd, &g_control_state)) {
                    printf("[CONTROL] 제어 명령 수신\n");
                }
            }
            else if (msg_header == 0x454D) {
                EmergencyCommand *emerg_cmd = (EmergencyCommand*)rx_buffer;
                if (EmergencySystem_ProcessCommand(emerg_cmd, g_emergency_state)) {
                    printf("[EMERGENCY] 긴급 명령 실행\n");
                }
            }
            
            g_frames_received++;
        }
        
        /* ========== 설정 동기화 (5초 주기) ========== */
        if (config_sync_timer >= PT_CONFIG_UPDATE_PERIOD_MS) {
            ConfigResponseMessage *response = 
                TelemetryConfig_GenerateResponseMessage(g_config);
            if (response) {
                free(response);
            }
            config_sync_timer = 0;
        }
        
        /* ========== 타임아웃 및 긴급 체크 ========== */
        GroundControl_CheckTimeout(&g_control_state);
        EmergencySystem_CheckConditions();
        
        if (EmergencySystem_IsInEmergency(g_emergency_state)) {
            printf("\n  [EMERGENCY] 긴급 모드 활성화\n\n");
            break;
        }
        
        /* ========== 타이머 증가 ========== */
        sensor_timer += 1;
        tx_timer += 1;
        launch_timer += 1;
        config_sync_timer += 1;
        
        if (loop_count % 1000 == 0) {
            printf("[%d ms] TX: %d, RX: %d, Log: %d\n",
                   loop_count, g_frames_transmitted, g_frames_received,
                   DataStorage_GetEntryCount(g_log_buffer));
        }
    }
}

/* ============================================================
 * 메인 함수
 * ============================================================ */

int main(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║      미사일 텔레메트리 시스템 작동       ║\n");
    printf("╚════════════════════════════════════════╝\n\n");
    
    if (MissileTM_InitializeSystem() != 0) {
        printf(" 시스템 초기화 실패\n");
        return 1;
    }
    
    MissileTM_MainLoop();
    
    MissileTM_ShutdownSystem();
    
    printf("\n프로그램 종료\n\n");
    
    return 0;
}
