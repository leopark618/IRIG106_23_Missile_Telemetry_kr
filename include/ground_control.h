#ifndef GROUND_CONTROL_H
#define GROUND_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * PT_: 프로젝트 튜닝
 * ============================================================ */

#define PT_COMMAND_BUFFER_SIZE 256
#define PT_THRUST_MAX 100.0f
#define PT_THRUST_MIN 0.0f
#define PT_RUDDER_MAX_ANGLE 45.0f
#define PT_ELEVON_MAX_ANGLE 30.0f
#define PT_COMMAND_TIMEOUT_MS 5000

/* ============================================================
 * IRIGFIX_: 고정
 * ============================================================ */

#define IRIGFIX_COMMAND_HEADER 0xABCD

/* ============================================================
 * 명령 타입
 * ============================================================ */

typedef enum {
    CMD_NONE = 0x00,
    CMD_THRUST_UPDATE = 0x10,
    CMD_RUDDER_UPDATE = 0x11,
    CMD_ELEVON_UPDATE = 0x12,
    CMD_TRAJECTORY_CHANGE = 0x20,
    CMD_SPEED_INCREASE = 0x21,
    CMD_SPEED_DECREASE = 0x22,
    CMD_QUERY_STATUS = 0x30,
    CMD_REQUEST_CAMERA = 0x31,
    CMD_EMERGENCY_STOP = 0xF0,
    CMD_SELF_DESTRUCT = 0xF1,
} CommandType;

/* ============================================================
 * 명령 구조
 * ============================================================ */

typedef struct {
    uint16_t header;
    CommandType cmd_type;
    uint16_t cmd_id;
    uint32_t timestamp;
    
    union {
        struct {
            float thrust_percent;
        } thrust;
        
        struct {
            float rudder_angle;
        } rudder;
        
        struct {
            float elevon_angle;
        } elevon;
        
        struct {
            float target_heading;
            float target_pitch;
            float target_roll;
        } trajectory;
        
        struct {
            float speed_change;
        } speed;
        
    } payload;
    
    uint16_t checksum;
} GroundControlCommand;

typedef struct {
    float current_thrust;
    float current_rudder_angle;
    float current_elevon_angle;
    
    float target_heading;
    float target_pitch;
    float target_roll;
    
    uint64_t last_command_time;
    bool is_command_valid;
    
} ControlState;

/* ============================================================
 * 함수 선언
 * ============================================================ */

bool GroundControl_ReceiveCommand(uint8_t *data, uint32_t len,
                                   GroundControlCommand *cmd);
bool GroundControl_ProcessCommand(GroundControlCommand *cmd,
                                   ControlState *state);

bool GroundControl_ValidateCommand(GroundControlCommand *cmd);
bool GroundControl_ValidateCRC(GroundControlCommand *cmd);

void GroundControl_UpdateState(ControlState *state, GroundControlCommand *cmd);
void GroundControl_ApplyControls(ControlState *state);

void GroundControl_SetThrust(float percent);
void GroundControl_SetRudder(float angle);
void GroundControl_SetElevon(float angle);

void GroundControl_CheckTimeout(ControlState *state);

#endif
