#include "camera_interface.h"
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * 카메라 인터페이스 구현
 * ============================================================ */

CameraDevice* Camera_Init(void)
{
    CameraDevice *cam = malloc(sizeof(CameraDevice));
    if (!cam) return NULL;
    
    cam->camera_handle = NULL;
    cam->current_frame_id = 0;
    cam->is_streaming = false;
    
    return cam;
}

void Camera_Destroy(CameraDevice *cam)
{
    if (cam) {
        free(cam);
    }
}

bool Camera_Start(CameraDevice *cam)
{
    if (!cam) return false;
    
    cam->is_streaming = true;
    cam->current_frame_id = 0;
    
    return true;
}

bool Camera_Stop(CameraDevice *cam)
{
    if (!cam) return false;
    
    cam->is_streaming = false;
    
    return true;
}

CameraFrame* Camera_CaptureFrame(CameraDevice *cam)
{
    if (!cam || !cam->is_streaming) return NULL;
    
    CameraFrame *frame = malloc(sizeof(CameraFrame));
    if (!frame) return NULL;
    
    frame->frame_id = cam->current_frame_id++;
    frame->width = PT_CAMERA_RESOLUTION_WIDTH;
    frame->height = PT_CAMERA_RESOLUTION_HEIGHT;
    frame->data_size = 0;
    frame->data = NULL;
    
    return frame;
}

void Camera_ReleaseFrame(CameraFrame *frame)
{
    if (frame) {
        if (frame->data) free(frame->data);
        free(frame);
    }
}
