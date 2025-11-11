#include "telemetry_config.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================
 * 실시간 텔레메트리 설정 변경 구현
 * ============================================================ */

ConfigSet* TelemetryConfig_Init(void)
{
    ConfigSet *config = malloc(sizeof(ConfigSet));
    if (!config) return NULL;
    
    config->magic = IRIGFIX_CONFIG_MAGIC;
    config->version = IRIGFIX_CONFIG_VERSION;
    config->param_count = 0;
    
    memset(config->params, 0, sizeof(config->params));
    
    return config;
}

void TelemetryConfig_Destroy(ConfigSet *config)
{
    if (config) {
        for (uint8_t i = 0; i < config->param_count; i++) {
            if (config->params[i].param_name) {
                free(config->params[i].param_name);
            }
        }
        free(config);
    }
}

void TelemetryConfig_RegisterFloatParam(ConfigSet *config, uint8_t param_id,
                                         const char *name, float default_val,
                                         float min_val, float max_val)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS || !name) return;
    
    ConfigParameter *param = &config->params[param_id];
    
    param->param_id = param_id;
    param->param_name = malloc(strlen(name) + 1);
    if (param->param_name) {
        strcpy(param->param_name, name);
    }
    param->type = PARAM_TYPE_FLOAT;
    
    param->current.float_val = default_val;
    param->min_val.float_min = min_val;
    param->max_val.float_max = max_val;
    param->default_val.float_default = default_val;
    
    param->last_updated = 0;
    param->is_dirty = false;
    
    config->param_count++;
}

void TelemetryConfig_RegisterIntParam(ConfigSet *config, uint8_t param_id,
                                       const char *name, int32_t default_val,
                                       int32_t min_val, int32_t max_val)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS || !name) return;
    
    ConfigParameter *param = &config->params[param_id];
    
    param->param_id = param_id;
    param->param_name = malloc(strlen(name) + 1);
    if (param->param_name) {
        strcpy(param->param_name, name);
    }
    param->type = PARAM_TYPE_INT32;
    
    param->current.int_val = default_val;
    param->min_val.int_min = min_val;
    param->max_val.int_max = max_val;
    param->default_val.int_default = default_val;
    
    param->last_updated = 0;
    param->is_dirty = false;
    
    config->param_count++;
}

void TelemetryConfig_RegisterBoolParam(ConfigSet *config, uint8_t param_id,
                                        const char *name, bool default_val)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS || !name) return;
    
    ConfigParameter *param = &config->params[param_id];
    
    param->param_id = param_id;
    param->param_name = malloc(strlen(name) + 1);
    if (param->param_name) {
        strcpy(param->param_name, name);
    }
    param->type = PARAM_TYPE_BOOL;
    
    param->current.bool_val = default_val;
    param->default_val.bool_default = default_val;
    
    param->last_updated = 0;
    param->is_dirty = false;
    
    config->param_count++;
}

float TelemetryConfig_GetFloat(ConfigSet *config, uint8_t param_id)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return 0.0f;
    
    ConfigParameter *param = &config->params[param_id];
    if (param->type != PARAM_TYPE_FLOAT) return 0.0f;
    
    return param->current.float_val;
}

int32_t TelemetryConfig_GetInt(ConfigSet *config, uint8_t param_id)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return 0;
    
    ConfigParameter *param = &config->params[param_id];
    if (param->type != PARAM_TYPE_INT32) return 0;
    
    return param->current.int_val;
}

bool TelemetryConfig_GetBool(ConfigSet *config, uint8_t param_id)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return false;
    
    ConfigParameter *param = &config->params[param_id];
    if (param->type != PARAM_TYPE_BOOL) return false;
    
    return param->current.bool_val;
}

bool TelemetryConfig_SetFloat(ConfigSet *config, uint8_t param_id, float value)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return false;
    
    ConfigParameter *param = &config->params[param_id];
    if (param->type != PARAM_TYPE_FLOAT) return false;
    
    if (value < param->min_val.float_min || 
        value > param->max_val.float_max) {
        return false;
    }
    
    param->current.float_val = value;
    param->is_dirty = true;
    
    return true;
}

bool TelemetryConfig_SetInt(ConfigSet *config, uint8_t param_id, int32_t value)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return false;
    
    ConfigParameter *param = &config->params[param_id];
    if (param->type != PARAM_TYPE_INT32) return false;
    
    if (value < param->min_val.int_min || 
        value > param->max_val.int_max) {
        return false;
    }
    
    param->current.int_val = value;
    param->is_dirty = true;
    
    return true;
}

bool TelemetryConfig_SetBool(ConfigSet *config, uint8_t param_id, bool value)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return false;
    
    ConfigParameter *param = &config->params[param_id];
    if (param->type != PARAM_TYPE_BOOL) return false;
    
    param->current.bool_val = value;
    param->is_dirty = true;
    
    return true;
}

bool TelemetryConfig_ProcessUpdateMessage(ConfigSet *config,
                                           ConfigUpdateMessage *msg)
{
    if (!config || !msg) return false;
    
    /*  올바른 헤더값: 0x4346 = "CF" */
    if (msg->msg_header != 0x4346) return false;
    
    bool all_success = true;
    
    for (uint8_t i = 0; i < msg->num_params; i++) {
        uint8_t param_id = msg->param_updates[i].param_id;
        uint8_t type = msg->param_updates[i].type;
        
        bool success = false;
        
        switch (type) {
            case PARAM_TYPE_FLOAT:
                success = TelemetryConfig_SetFloat(config, param_id,
                                                   msg->param_updates[i].value.float_val);
                break;
                
            case PARAM_TYPE_INT32:
                success = TelemetryConfig_SetInt(config, param_id,
                                                 msg->param_updates[i].value.int_val);
                break;
                
            case PARAM_TYPE_BOOL:
                success = TelemetryConfig_SetBool(config, param_id,
                                                  msg->param_updates[i].value.bool_val);
                break;
        }
        
        if (!success) {
            all_success = false;
        }
    }
    
    return all_success;
}

ConfigResponseMessage* TelemetryConfig_GenerateResponseMessage(ConfigSet *config)
{
    if (!config) return NULL;
    
    ConfigResponseMessage *response = malloc(sizeof(ConfigResponseMessage));
    if (!response) return NULL;
    
    /*  올바른 헤더값: 0x4352 = "CR" */
    response->msg_header = 0x4352;
    response->msg_type = 0x01;
    response->num_params = config->param_count;
    
    for (uint8_t i = 0; i < config->param_count && i < PT_CONFIG_MAX_PARAMS; i++) {
        ConfigParameter *param = &config->params[i];
        
        response->param_info[i].param_id = param->param_id;
        
        /*  포인터 직접 할당 */
        response->param_info[i].param_name = param->param_name;
        response->param_info[i].type = param->type;
        
        switch (param->type) {
            case PARAM_TYPE_FLOAT:
                response->param_info[i].current_value.float_val = param->current.float_val;
                response->param_info[i].min_val.float_val = param->min_val.float_min;
                response->param_info[i].max_val.float_val = param->max_val.float_max;
                break;
                
            case PARAM_TYPE_INT32:
                response->param_info[i].current_value.int_val = param->current.int_val;
                response->param_info[i].min_val.int_val = param->min_val.int_min;
                response->param_info[i].max_val.int_val = param->max_val.int_max;
                break;
                
            case PARAM_TYPE_BOOL:
                response->param_info[i].current_value.bool_val = param->current.bool_val;
                break;
        }
    }
    
    response->crc = TelemetryConfig_CalculateCRC(response);
    response->msg_len = sizeof(ConfigResponseMessage);
    
    return response;
}

void TelemetryConfig_SyncToHardware(ConfigSet *config)
{
    if (!config) return;
    
    for (uint8_t i = 0; i < config->param_count; i++) {
        ConfigParameter *param = &config->params[i];
        
        if (!param->is_dirty) continue;
        
        if (param->param_name) {
            printf("[CONFIG] %s 변경됨\n", param->param_name);
        }
        
        param->is_dirty = false;
    }
}

void TelemetryConfig_ApplyAllChanges(ConfigSet *config)
{
    if (!config) return;
    TelemetryConfig_SyncToHardware(config);
}

bool TelemetryConfig_ValidateParam(ConfigSet *config, uint8_t param_id)
{
    if (!config || param_id >= PT_CONFIG_MAX_PARAMS) return false;
    
    ConfigParameter *param = &config->params[param_id];
    
    switch (param->type) {
        case PARAM_TYPE_FLOAT:
            return (param->current.float_val >= param->min_val.float_min &&
                    param->current.float_val <= param->max_val.float_max);
                    
        case PARAM_TYPE_INT32:
            return (param->current.int_val >= param->min_val.int_min &&
                    param->current.int_val <= param->max_val.int_max);
                    
        case PARAM_TYPE_BOOL:
            return true;
    }
    
    return false;
}

uint16_t TelemetryConfig_CalculateCRC(ConfigResponseMessage *msg)
{
    if (!msg) return 0;
    
    uint16_t crc = 0;
    uint8_t *data = (uint8_t *)msg;
    
    for (size_t i = 0; i < msg->msg_len - 2; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}
