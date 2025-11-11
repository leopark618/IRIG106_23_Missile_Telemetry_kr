#include "missile_telemetry.h"
#include <math.h>
#include <string.h>

void MissileTM_ReadSensors(MissileTelemetrySystem *sys)
{
    if (!sys) return;
    
    int16_t imu_raw;
    
    float accel_scale = 100.0f / 32768.0f;
    sys->current_frame.accel_x_g = imu_raw * accel_scale;
    sys->current_frame.accel_y_g = imu_raw * accel_scale;
    sys->current_frame.accel_z_g = imu_raw * accel_scale;
    
    float gyro_scale = 2000.0f / 32768.0f;
    sys->current_frame.gyro_x_dps = imu_raw * gyro_scale;
    sys->current_frame.gyro_y_dps = imu_raw * gyro_scale;
    sys->current_frame.gyro_z_dps = imu_raw * gyro_scale;
    
    uint16_t pressure_adc;
    float pressure_scale = 10000.0f / 65535.0f;
    for (int i = 0; i < 4; i++) {
        sys->current_frame.pressure_psi[i] = pressure_adc[i] * pressure_scale;
    }
    
    uint16_t temp_adc;
    for (int i = 0; i < 8; i++) {
        float R = temp_adc[i] * 10000.0f / 65535.0f;
        float lnR = logf(R);
        
        float A = 0.001129148f;
        float B = 0.000234125f;
        float C = 0.0000000876741f;
        
        float T_kelvin = 1.0f / (A + B*lnR + C*lnR*lnR*lnR);
        sys->current_frame.temperature_c[i] = T_kelvin - 273.15f;
    }
    
    uint8_t uart_rx;
    int rx_len = 0;
    
    if (rx_len >= 40) {
        memcpy(sys->current_frame.guidance_cmd, &uart_rx, 16);
        memcpy(sys->current_frame.actuator_pos, &uart_rx, 16);
        sys->current_frame.flight_mode = uart_rx;
    }
    
    sys->current_frame.frame_counter++;
}

bool MissileTM_DetectLaunch(MissileTelemetrySystem *sys)
{
    if (!sys) return false;
    
    float ax = sys->current_frame.accel_x_g;
    float ay = sys->current_frame.accel_y_g;
    float az = sys->current_frame.accel_z_g;
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    
    const float LAUNCH_THRESHOLD = 5.0f;
    static int counter = 0;
    
    if (mag > LAUNCH_THRESHOLD) {
        counter++;
        if (counter >= 10) {
            counter = 0;
            return true;
        }
    } else {
        counter = 0;
    }
    
    return false;
}
