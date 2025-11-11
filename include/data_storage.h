#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "missile_telemetry.h"

/* ============================================================
 * PT_: 프로젝트 튜닝 - 자유롭게 변경
 * ============================================================ */

#define PT_MAX_LOG_ENTRIES 10000
#define PT_ENABLE_DATA_LOGGING 1
#define PT_ENABLE_CAMERA_LOGGING 1
#define PT_CAMERA_FRAME_SIZE (1024 * 100)
#define PT_LOG_BUFFER_SIZE (1024 * 512)

/* ============================================================
 * IRIGFIX_: IRIG 106 고정 - 절대 변경 금지
 * ============================================================ */

#define IRIGFIX_LOG_FORMAT_VERSION 1
#define IRIGFIX_LOG_MAGIC 0x4D49534C

/* ============================================================
 * 로그 엔트리 구조
 * ============================================================ */

typedef struct {
    uint32_t entry_id;
    uint64_t timestamp_us;
    MissileTelemetryFrame telemetry;
    
    uint32_t camera_frame_id;
    uint32_t camera_data_size;
    uint8_t *camera_data;
    
    uint8_t last_command_type;
    float last_thrust_cmd;
    
} LogEntry;

typedef struct {
    LogEntry *buffer;
    uint32_t buffer_count;
    uint32_t buffer_capacity;
    uint32_t write_position;
    bool is_full;
} LogBuffer;

/* ============================================================
 * 함수 선언
 * ============================================================ */

LogBuffer* DataStorage_Init(uint32_t capacity);
void DataStorage_Destroy(LogBuffer *log);

void DataStorage_WriteEntry(LogBuffer *log, LogEntry *entry);
void DataStorage_WriteCameraFrame(LogBuffer *log, uint32_t frame_id,
                                   uint8_t *frame_data, uint32_t size);

LogEntry* DataStorage_ReadEntry(LogBuffer *log, uint32_t index);
uint8_t* DataStorage_ReadCameraFrame(LogBuffer *log, uint32_t frame_id);

uint32_t DataStorage_GetEntryCount(LogBuffer *log);
uint32_t DataStorage_GetCameraFrameCount(LogBuffer *log);

bool DataStorage_SaveToSD(LogBuffer *log, const char *filename);
bool DataStorage_LoadFromSD(LogBuffer *log, const char *filename);

#endif
