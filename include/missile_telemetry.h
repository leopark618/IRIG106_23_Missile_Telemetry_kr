#ifndef MISSILE_TELEMETRY_H
#define MISSILE_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * IRIG 106-23 표준 정의 (고정 상수)
 * ============================================================ */

/* IRIGFIX_: IRIG 106에서 고정으로 정한 값 (변경 금지!) */
#define IRIGFIX_DATA_RATE 10e6              /* 데이터율: 10 Mbps */
#define IRIGFIX_SAMPLE_RATE 80e6            /* 샘플링: 80 MHz */
#define IRIGFIX_CARRIER_FREQ 2.35e9         /* 반송파: 2.35 GHz */
#define IRIGFIX_SAMPLES_PER_SYMBOL 8        /* 심볼당 샘플: 8개 */

#define IRIGFIX_NUM_IMU_CHANNELS 6
#define IRIGFIX_NUM_PRESSURE_CHANNELS 4
#define IRIGFIX_NUM_TEMP_CHANNELS 8
#define IRIGFIX_NUM_GUIDANCE_CHANNELS 16

/* ============================================================
 * 시스템 데이터 구조
 * ============================================================ */

typedef struct {
    uint32_t frame_counter;                 /* 프레임 번호 */
    uint64_t timestamp_us;                  /* 타임스탬프 (마이크로초) */
    
    /* IMU 데이터 (가속도, 각속도) */
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    
    /* 센서 데이터 배열 */
    float pressure_psi[IRIGFIX_NUM_PRESSURE_CHANNELS];
    float temperature_c[IRIGFIX_NUM_TEMP_CHANNELS];
    
    /* 유도 명령 */
    float guidance_cmd[IRIGFIX_NUM_GUIDANCE_CHANNELS];
    float actuator_pos[IRIGFIX_NUM_GUIDANCE_CHANNELS];
    uint8_t flight_mode;
    
    /* GPS 위치 */
    double latitude;
    double longitude;
    float altitude_m;
    
    /* 시스템 상태 */
    float battery_voltage;
    uint16_t system_status;
    uint16_t crc16;
    
} __attribute__((packed)) MissileTelemetryFrame;

typedef struct {
    void *ldpc_encoder;
    void *soqpsk_modulator;
    
    void *imu_handle;
    void *adc_handle;
    void *uart_handle;
    
    MissileTelemetryFrame current_frame;
    uint8_t tx_buffer[16384];
    
    bool system_armed;
    bool launch_detected;
    bool telemetry_active;
    
    uint32_t frames_sent;
    uint32_t errors;
    
} MissileTelemetrySystem;

/* ============================================================
 * 함수 선언
 * ============================================================ */

MissileTelemetrySystem* MissileTM_Create(void);
void MissileTM_Destroy(MissileTelemetrySystem *sys);
void MissileTM_ReadSensors(MissileTelemetrySystem *sys);
bool MissileTM_DetectLaunch(MissileTelemetrySystem *sys);
void MissileTM_ProcessAndTransmit(MissileTelemetrySystem *sys);

#endif

