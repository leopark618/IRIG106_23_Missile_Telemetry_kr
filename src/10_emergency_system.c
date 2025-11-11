#include "emergency_system.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 긴급 정지 및 자폭 시스템 구현
 * ============================================================ */

EmergencyState* EmergencySystem_Init(void)
{
    EmergencyState *state = malloc(sizeof(EmergencyState));
    if (!state) return NULL;
    
    state->is_emergency_mode = false;
    state->current_emergency = EMERGENCY_NONE;
    state->emergency_start_time = 0;
    state->emergency_count = 0;
    
    return state;
}

void EmergencySystem_Destroy(EmergencyState *state)
{
    if (state) free(state);
}

bool EmergencySystem_ValidateCommand(EmergencyCommand *cmd)
{
    if (!cmd) return false;
    
    if (cmd->magic != IRIGFIX_EMERGENCY_MAGIC) {
        return false;
    }
    
    uint16_t calculated_crc = 0;
    uint8_t *data = (uint8_t *)cmd;
    for (size_t i = 0; i < sizeof(EmergencyCommand) - 2; i++) {
        calculated_crc ^= data[i];
    }
    
    if (calculated_crc != cmd->crc) {
        return false;
    }
    
    return true;
}

bool EmergencySystem_ProcessCommand(EmergencyCommand *cmd,
                                     EmergencyState *state)
{
    if (!cmd || !state) return false;
    
    if (!EmergencySystem_ValidateCommand(cmd)) {
        return false;
    }
    
    switch (cmd->type) {
        case EMERGENCY_TERMINATE:
            EmergencySystem_Terminate();
            break;
            
        case EMERGENCY_ABORT:
            EmergencySystem_AbortMission();
            break;
            
        default:
            return false;
    }
    
    state->is_emergency_mode = true;
    state->current_emergency = cmd->type;
    state->emergency_count++;
    
    return true;
}

void EmergencySystem_Terminate(void)
{
    printf("[EMERGENCY] 모든 추진력 차단...\n");
    printf("[EMERGENCY] 낙하산 배포...\n");
    printf("[EMERGENCY] Safe Mode 진입\n");
    
    while (1) {
        /* Safe Mode 유지 */
    }
}

void EmergencySystem_SelfDestruct(void)
{
    printf("[EMERGENCY] 자폭 시스템 활성화...\n");
    printf("[EMERGENCY] %d ms 대기 중...\n", PT_SELF_DESTRUCT_CHARGE_DELAY_MS);
    printf("[EMERGENCY] 폭발!!!\n");
    
    while (1);
}

void EmergencySystem_AbortMission(void)
{
    printf("[EMERGENCY] 미션 중단...\n");
    printf("[EMERGENCY] 복귀 시스템 활성화...\n");
    printf("[EMERGENCY] 최대 안전 모드 진입...\n");
}

void EmergencySystem_CheckConditions(void)
{
    EmergencySystem_DetectOverheat();
    EmergencySystem_DetectFuel();
    EmergencySystem_DetectBattery();
}

void EmergencySystem_DetectOverheat(void)
{
    /* 온도 체크 */
}

void EmergencySystem_DetectFuel(void)
{
    /* 연료 체크 */
}

void EmergencySystem_DetectBattery(void)
{
    /* 배터리 체크 */
}

bool EmergencySystem_IsInEmergency(EmergencyState *state)
{
    if (!state) return false;
    return state->is_emergency_mode;
}

EmergencyType EmergencySystem_GetCurrentEmergency(EmergencyState *state)
{
    if (!state) return EMERGENCY_NONE;
    return state->current_emergency;
}
