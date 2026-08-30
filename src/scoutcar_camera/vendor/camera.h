// camera.h
#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <stddef.h>
#include "image_utils.h"

#ifdef __cplusplus
namespace cv { class VideoCapture; }

struct camera_context_t {
    cv::VideoCapture *cap;
    int camera_id;              // 按索引打开时的设备号；按路径打开时为 -1
    int width;
    int height;
    char device_path[128];      // 实际打开的设备路径

    // RKISP mainpath 使用 V4L2 Multi-Planar；OpenCV 的 V4L2 后端无法直接读取它。
    int v4l2_fd;
    int v4l2_streaming;
    void *v4l2_buffers[4];
    size_t v4l2_lengths[4];
    int v4l2_buffer_count;
};

extern "C" {
#else
typedef struct camera_context_t camera_context_t;
#endif


/**
 * 打开摄像头（按索引，旧接口；等效于 open_camera_path("/dev/video<id>")）
 */
int open_camera(int camera_id, camera_context_t **ctx);

/**
 * 按设备路径打开（如 /dev/video1 或 /dev/v4l/by-id/usb-...-video-index0）
 * 打开后会尝试读一帧验证，确认是能出帧的真实摄像头
 */
int open_camera_path(const char* path, camera_context_t **ctx,
                     int width, int height, int fps);

/**
 * 自动搜索可用的真实摄像头并打开：
 *   1. 优先 RKISP mainpath（MIPI 摄像头，经 ISP/3A 处理后的输出）
 *   2. 回退 /dev/v4l/by-id 下 *-video-index0 节点（USB 序列号稳定，设备号变也不变）
 *   3. 最后扫描 /dev/video[0-9]*，跳过 dec/enc 与元数据节点，探测能出帧者
 * 返回 0 成功；失败返回 -1（并打印原因）
 */
int open_camera_auto(camera_context_t **ctx, int width, int height, int fps,
                     bool prefer_mipi);

/**
 * 读取一帧（返回 RGB888 格式的 image_buffer_t）
 * 使用完后需 free(img->virt_addr)
 */
int read_camera_frame(camera_context_t *ctx, image_buffer_t *img);

/**
 * 关闭摄像头
 */
void close_camera(camera_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_H__ */
