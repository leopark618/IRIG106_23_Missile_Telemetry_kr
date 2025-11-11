#ifndef EMERGENCY_SYSTEM_H
#define EMERGENCY_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * PT_: 프로젝트 튜닝
 * ============================================================ */

#define PT_EMERGENCY_SHUTDOWN_TIME_MS 100
#define PT_SELF_DESTRUCT_CHARGE_DELAY_MS 500
#define PT_EMERGENCY_LOG_ENABLED 1
#define PT_PARACHUTE_DEPLOY_ALTITUDE_M 1000.0f

/* ============================================================
 * IRIGFIX_: 고정
 * ============================================================ */

#define IRIGFIX_EMERGENCY_MAGIC 0xDEADBEEF

/* ============================================================
 * 긴급 타입
 * ============================================================ */

typedef enum {
    EMERGENCY_NONE = 0x00,
    EMERGENCY_OVERHEAT = 0x01,
    EMERGENCY_LOW_FUEL = 0x02,
    EMERGENCY_BATTERY_LOW = 0x03,
    EMERGENCY_SYSTEM_FAILURE = 0x04,
    EMERGENCY_LOSS_OF_CONTROL = 0x05,
    EMERGENCY_ABORT = 0xFE,
    EMERGENCY_TERMINATE = 0xFF,
} EmergencyType;

/* ============================================================
 * 긴급 명령 구조
 * ============================================================ */

typedef struct {
    uint32_t magic;
    EmergencyType type;
    uint32_t timestamp;
    uint8_t verification_code;
    uint16_t crc;
} EmergencyCommand;

typedef struct {
    bool is_emergency_mode;
    EmergencyType current_emergency;
    uint64_t emergency_start_time;
    uint32_t emergency_count;
} EmergencyState;

/* ============================================================
 * 함수 선언
 * ============================================================ */

EmergencyState* EmergencySystem_Init(void);
void EmergencySystem_Destroy(EmergencyState *state);

void EmergencySystem_CheckConditions(void);
void EmergencySystem_DetectOverheat(void);
void EmergencySystem_DetectFuel(void);
void EmergencySystem_DetectBattery(void);

bool EmergencySystem_ProcessCommand(EmergencyCommand *cmd,
                                     EmergencyState *state);
bool EmergencySystem_ValidateCommand(EmergencyCommand *cmd);

void EmergencySystem_Terminate(void);
void EmergencySystem_SelfDestruct(void);
void EmergencySystem_AbortMission(void);

bool EmergencySystem_IsInEmergency(EmergencyState *state);
EmergencyType EmergencySystem_GetCurrentEmergency(EmergencyState *state);

#endif
