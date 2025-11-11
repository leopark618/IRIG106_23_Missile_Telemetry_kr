#ifndef TELEMETRY_CONFIG_H
#define TELEMETRY_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================
 * PT_: 프로젝트 튜닝 - 자유롭게 변경
 * ============================================================ */

#define PT_CONFIG_BUFFER_SIZE 4096
#define PT_CONFIG_UPDATE_PERIOD_MS 5000
#define PT_CONFIG_MAX_PARAMS 100

/* ============================================================
 * IRIGFIX_: IRIG 106 고정 - 절대 변경 금지
 * ============================================================ */

#define IRIGFIX_CONFIG_VERSION 2
#define IRIGFIX_CONFIG_MAGIC 0x43464700

/* ============================================================
 * 파라미터 타입
 * ============================================================ */

typedef enum {
    PARAM_TYPE_FLOAT = 0x01,
    PARAM_TYPE_INT32 = 0x02,
    PARAM_TYPE_BOOL = 0x03,
} ParamType;

/* ============================================================
 * 파라미터 구조 (문자열을 포인터로 변경)
 * ============================================================ */

typedef struct {
    uint8_t param_id;
    char *param_name;                    /*  포인터로 변경 */
    ParamType type;
    
    union {
        float float_val;
        int32_t int_val;
        bool bool_val;
    } current;
    
    union {
        float float_min;
        int32_t int_min;
    } min_val;
    
    union {
        float float_max;
        int32_t int_max;
    } max_val;
    
    union {
        float float_default;
        int32_t int_default;
        bool bool_default;
    } default_val;
    
    uint32_t last_updated;
    bool is_dirty;
    
} ConfigParameter;

typedef struct {
    uint32_t magic;
    uint8_t version;
    uint16_t param_count;
    ConfigParameter params[PT_CONFIG_MAX_PARAMS];
    uint32_t checksum;
} ConfigSet;

/* ============================================================
 * 송수신 메시지 (헤더값 수정)
 * ============================================================ */

typedef struct {
    uint16_t msg_header;                /* 0x4346 = "CF" */
    uint8_t msg_type;                   /* 0x01: 조회, 0x02: 변경 */
    uint16_t msg_len;
    
    uint8_t num_params;
    
    struct {
        uint8_t param_id;
        uint8_t type;
        union {
            float float_val;
            int32_t int_val;
            bool bool_val;
        } value;
    } param_updates[PT_CONFIG_MAX_PARAMS];
    
    uint16_t crc;
} ConfigUpdateMessage;

typedef struct {
    uint16_t msg_header;                /* 0x4352 = "CR" */
    uint8_t msg_type;                   /* 0x01: 조회 응답 */
    uint16_t msg_len;
    
    uint8_t num_params;
    
    struct {
        uint8_t param_id;
        char *param_name;               /*  포인터로 변경 */
        uint8_t type;
        union {
            float float_val;
            int32_t int_val;
            bool bool_val;
        } current_value;
        union {
            float float_val;
            int32_t int_val;
        } min_val;
        union {
            float float_val;
            int32_t int_val;
        } max_val;
    } param_info[PT_CONFIG_MAX_PARAMS];
    
    uint16_t crc;
} ConfigResponseMessage;

/* ============================================================
 * 함수 선언
 * ============================================================ */

ConfigSet* TelemetryConfig_Init(void);
void TelemetryConfig_Destroy(ConfigSet *config);

void TelemetryConfig_RegisterFloatParam(ConfigSet *config, uint8_t param_id,
                                         const char *name, float default_val,
                                         float min_val, float max_val);
void TelemetryConfig_RegisterIntParam(ConfigSet *config, uint8_t param_id,
                                       const char *name, int32_t default_val,
                                       int32_t min_val, int32_t max_val);
void TelemetryConfig_RegisterBoolParam(ConfigSet *config, uint8_t param_id,
                                        const char *name, bool default_val);

float TelemetryConfig_GetFloat(ConfigSet *config, uint8_t param_id);
int32_t TelemetryConfig_GetInt(ConfigSet *config, uint8_t param_id);
bool TelemetryConfig_GetBool(ConfigSet *config, uint8_t param_id);

bool TelemetryConfig_SetFloat(ConfigSet *config, uint8_t param_id, float value);
bool TelemetryConfig_SetInt(ConfigSet *config, uint8_t param_id, int32_t value);
bool TelemetryConfig_SetBool(ConfigSet *config, uint8_t param_id, bool value);

bool TelemetryConfig_ProcessUpdateMessage(ConfigSet *config,
                                           ConfigUpdateMessage *msg);
ConfigResponseMessage* TelemetryConfig_GenerateResponseMessage(ConfigSet *config);

void TelemetryConfig_SyncToHardware(ConfigSet *config);
void TelemetryConfig_ApplyAllChanges(ConfigSet *config);

bool TelemetryConfig_ValidateParam(ConfigSet *config, uint8_t param_id);
uint16_t TelemetryConfig_CalculateCRC(ConfigResponseMessage *msg);

#endif
