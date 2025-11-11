#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * PT_: 프로젝트 튜닝
 * ============================================================ */

#define PT_CAMERA_RESOLUTION_WIDTH 320
#define PT_CAMERA_RESOLUTION_HEIGHT 240
#define PT_CAMERA_FPS 10
#define PT_CAMERA_COMPRESSION_RATIO 10
#define PT_CAMERA_ENABLE 1

/* ============================================================
 * IRIGFIX_: 고정
 * ============================================================ */

#define IRIGFIX_CAMERA_MAX_FRAME_SIZE (1024 * 200)
#define IRIGFIX_CAMERA_FORMAT_JPEG 1

/* ============================================================
 * 카메라 구조
 * ============================================================ */

typedef struct {
    uint32_t frame_id;
    uint64_t timestamp_us;
    uint16_t width;
    uint16_t height;
    uint32_t data_size;
    uint8_t *data;
} CameraFrame;

typedef struct {
    void *camera_handle;
    uint32_t current_frame_id;
    bool is_streaming;
} CameraDevice;

/* ============================================================
 * 함수 선언
 * ============================================================ */

CameraDevice* Camera_Init(void);
void Camera_Destroy(CameraDevice *cam);

bool Camera_Start(CameraDevice *cam);
bool Camera_Stop(CameraDevice *cam);

CameraFrame* Camera_CaptureFrame(CameraDevice *cam);
void Camera_ReleaseFrame(CameraFrame *frame);

#endif
