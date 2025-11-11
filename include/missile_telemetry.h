#ifndef MISSILE_TELEMETRY_H
#define MISSILE_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#define MISSILE_DATA_RATE 10e6
#define MISSILE_SAMPLE_RATE 80e6
#define MISSILE_CARRIER_FREQ 2.35e9
#define SAMPLES_PER_SYMBOL 8

#define NUM_IMU_CHANNELS 6
#define NUM_PRESSURE_CHANNELS 4
#define NUM_TEMP_CHANNELS 8
#define NUM_GUIDANCE_CHANNELS 16

typedef struct {
    uint32_t frame_counter;
    uint64_t timestamp_us;
    
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    
    float pressure_psi;
    float temperature_c;
    
    float guidance_cmd;
    float actuator_pos;
    uint8_t flight_mode;
    
    double latitude, longitude;
    float altitude_m;
    
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

MissileTelemetrySystem* MissileTM_Create(void);
void MissileTM_Destroy(MissileTelemetrySystem *sys);
void MissileTM_ReadSensors(MissileTelemetrySystem *sys);
bool MissileTM_DetectLaunch(MissileTelemetrySystem *sys);
void MissileTM_ProcessAndTransmit(MissileTelemetrySystem *sys);

#endif
