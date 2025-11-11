#include "data_storage.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 데이터 저장소 구현
 * ============================================================ */

LogBuffer* DataStorage_Init(uint32_t capacity)
{
    LogBuffer *log = malloc(sizeof(LogBuffer));
    if (!log) return NULL;
    
    log->buffer = malloc(capacity * sizeof(LogEntry));
    if (!log->buffer) {
        free(log);
        return NULL;
    }
    
    log->buffer_count = 0;
    log->buffer_capacity = capacity;
    log->write_position = 0;
    log->is_full = false;
    
    return log;
}

void DataStorage_Destroy(LogBuffer *log)
{
    if (log) {
        if (log->buffer) free(log->buffer);
        free(log);
    }
}

void DataStorage_WriteEntry(LogBuffer *log, LogEntry *entry)
{
    if (!log || !entry) return;
    
    if (log->buffer_count >= log->buffer_capacity) {
        log->is_full = true;
        log->write_position = 0;
    }
    
    memcpy(&log->buffer[log->write_position], entry, sizeof(LogEntry));
    
    log->write_position++;
    if (log->write_position >= log->buffer_capacity) {
        log->write_position = 0;
    }
    
    if (!log->is_full) {
        log->buffer_count++;
    }
}

void DataStorage_WriteCameraFrame(LogBuffer *log, uint32_t frame_id,
                                   uint8_t *frame_data, uint32_t size)
{
    if (!log || !frame_data) return;
    
    if (log->write_position > 0) {
        LogEntry *current = &log->buffer[log->write_position - 1];
        current->camera_frame_id = frame_id;
        current->camera_data_size = size;
        current->camera_data = frame_data;
    }
}

LogEntry* DataStorage_ReadEntry(LogBuffer *log, uint32_t index)
{
    if (!log || index >= log->buffer_count) return NULL;
    return &log->buffer[index];
}

uint8_t* DataStorage_ReadCameraFrame(LogBuffer *log, uint32_t frame_id)
{
    if (!log) return NULL;
    
    for (uint32_t i = 0; i < log->buffer_count; i++) {
        if (log->buffer[i].camera_frame_id == frame_id) {
            return log->buffer[i].camera_data;
        }
    }
    
    return NULL;
}

uint32_t DataStorage_GetEntryCount(LogBuffer *log)
{
    if (!log) return 0;
    return log->buffer_count;
}

uint32_t DataStorage_GetCameraFrameCount(LogBuffer *log)
{
    if (!log) return 0;
    
    uint32_t count = 0;
    for (uint32_t i = 0; i < log->buffer_count; i++) {
        if (log->buffer[i].camera_data_size > 0) {
            count++;
        }
    }
    
    return count;
}

bool DataStorage_SaveToSD(LogBuffer *log, const char *filename)
{
    if (!log || !filename) return false;
    
    FILE *file = fopen(filename, "wb");
    if (!file) return false;
    
    uint32_t magic = IRIGFIX_LOG_MAGIC;
    fwrite(&magic, sizeof(uint32_t), 1, file);
    
    uint32_t count = log->buffer_count;
    fwrite(&count, sizeof(uint32_t), 1, file);
    
    for (uint32_t i = 0; i < count; i++) {
        fwrite(&log->buffer[i], sizeof(LogEntry), 1, file);
        
        if (log->buffer[i].camera_data && log->buffer[i].camera_data_size > 0) {
            fwrite(log->buffer[i].camera_data, 
                   log->buffer[i].camera_data_size, 1, file);
        }
    }
    
    fclose(file);
    return true;
}

bool DataStorage_LoadFromSD(LogBuffer *log, const char *filename)
{
    if (!log || !filename) return false;
    
    FILE *file = fopen(filename, "rb");
    if (!file) return false;
    
    uint32_t magic;
    fread(&magic, sizeof(uint32_t), 1, file);
    if (magic != IRIGFIX_LOG_MAGIC) {
        fclose(file);
        return false;
    }
    
    uint32_t count;
    fread(&count, sizeof(uint32_t), 1, file);
    
    for (uint32_t i = 0; i < count && i < log->buffer_capacity; i++) {
        fread(&log->buffer[i], sizeof(LogEntry), 1, file);
        
        if (log->buffer[i].camera_data_size > 0) {
            log->buffer[i].camera_data = malloc(log->buffer[i].camera_data_size);
            if (log->buffer[i].camera_data) {
                fread(log->buffer[i].camera_data, 
                      log->buffer[i].camera_data_size, 1, file);
            }
        }
    }
    
    log->buffer_count = count;
    fclose(file);
    return true;
}
