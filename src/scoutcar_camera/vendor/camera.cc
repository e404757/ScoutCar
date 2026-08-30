// camera.cc
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <linux/videodev2.h>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "image_utils.h"
#include "camera.h"

// ═══════════════ 内部工具 ═══════════════

// 列出目录下以 prefix 开头的完整路径（排序后）
static std::vector<std::string> listDirPrefix(const char* dir, const std::string& prefix) {
    std::vector<std::string> out;
    DIR* d = opendir(dir);
    if (d == NULL) return out;
    struct dirent* e;
    while ((e = readdir(d)) != NULL) {
        std::string name = e->d_name;
        if (prefix.empty() || name.rfind(prefix, 0) == 0)
            out.push_back(std::string(dir) + "/" + name);
    }
    closedir(d);
    std::sort(out.begin(), out.end());
    return out;
}

// 从 "/dev/video12" 提取数字 12
static int videoNum(const std::string& path) {
    std::string name = path.substr(path.rfind('/') + 1);
    name = name.substr(5);  // 去掉 "video"
    int v = 0;
    for (char c : name)
        if (isdigit(static_cast<unsigned char>(c))) v = v * 10 + (c - '0');
    return v;
}

// 尝试按路径打开并验证能出帧；成功返回 0
static bool isRkispMainpath(const std::string& video_path) {
    // 用 realpath 解析符号链接（如 /dev/video-camera0 -> /dev/video11），
    // 否则软的别名在 /sys/class/video4linux/ 下没有对应条目，会被误判为非 rkisp。
    char real[512];
    const char * rp = realpath(video_path.c_str(), real);
    std::string name = rp ? std::string(rp).substr(std::string(rp).rfind('/') + 1)
                          : video_path.substr(video_path.rfind('/') + 1);
    std::ifstream name_file("/sys/class/video4linux/" + name + "/name");
    std::string device_name;
    std::getline(name_file, device_name);
    return device_name == "rkisp_mainpath";
}

static int xioctl(int fd, unsigned long request, void *arg) {
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

static void close_v4l2(camera_context_t *c) {
    if (c == NULL || c->v4l2_fd < 0) return;
    if (c->v4l2_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        xioctl(c->v4l2_fd, VIDIOC_STREAMOFF, &type);
    }
    for (int i = 0; i < c->v4l2_buffer_count; ++i) {
        if (c->v4l2_buffers[i] != NULL) {
            munmap(c->v4l2_buffers[i], c->v4l2_lengths[i]);
        }
    }
    close(c->v4l2_fd);
    c->v4l2_fd = -1;
    c->v4l2_streaming = 0;
}

static int open_rkisp_v4l2(const char *path, camera_context_t **ctx,
                           int width, int height) {
    camera_context_t *c = static_cast<camera_context_t *>(calloc(1, sizeof(camera_context_t)));
    if (c == NULL) return -1;
    c->v4l2_fd = open(path, O_RDWR | O_NONBLOCK);
    if (c->v4l2_fd < 0) {
        free(c);
        return -1;
    }

    struct v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = width > 0 ? static_cast<unsigned int>(width) : 640U;
    fmt.fmt.pix_mp.height = height > 0 ? static_cast<unsigned int>(height) : 480U;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.num_planes = 1;
    if (xioctl(c->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) {
        close_v4l2(c);
        free(c);
        return -1;
    }

    struct v4l2_requestbuffers req {};
    req.count = 3;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(c->v4l2_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        close_v4l2(c);
        free(c);
        return -1;
    }

    c->v4l2_buffer_count = std::min(static_cast<int>(req.count), 4);
    for (int i = 0; i < c->v4l2_buffer_count; ++i) {
        struct v4l2_plane planes[VIDEO_MAX_PLANES] {};
        struct v4l2_buffer buf {};
        buf.type = req.type;
        buf.memory = req.memory;
        buf.index = static_cast<unsigned int>(i);
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;
        if (xioctl(c->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0 || buf.length < 1) {
            close_v4l2(c);
            free(c);
            return -1;
        }
        c->v4l2_lengths[i] = planes[0].length;
        c->v4l2_buffers[i] = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, c->v4l2_fd, planes[0].m.mem_offset);
        if (c->v4l2_buffers[i] == MAP_FAILED) {
            c->v4l2_buffers[i] = NULL;
            close_v4l2(c);
            free(c);
            return -1;
        }
        if (xioctl(c->v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
            close_v4l2(c);
            free(c);
            return -1;
        }
    }

    enum v4l2_buf_type type = static_cast<enum v4l2_buf_type>(req.type);
    if (xioctl(c->v4l2_fd, VIDIOC_STREAMON, &type) < 0) {
        close_v4l2(c);
        free(c);
        return -1;
    }
    c->v4l2_streaming = 1;
    c->camera_id = -1;
    c->width = static_cast<int>(fmt.fmt.pix_mp.width);
    c->height = static_cast<int>(fmt.fmt.pix_mp.height);
    snprintf(c->device_path, sizeof(c->device_path), "%s", path);
    printf("open_camera: 已打开 RKISP %s, 分辨率 %dx%d\n", path, c->width, c->height);
    *ctx = c;
    return 0;
}

static int try_open_path(const char* path, camera_context_t **ctx,
                         int width, int height, int fps) {
    if (ctx == NULL || path == NULL) return -1;
    // rkisp_mainpath（MIPI）用手动 V4L2 路径：C++ OpenCV 4.5.4 默认请求 YUYV 会被 rkisp 拒绝
    // （isOpened=0），而 NV12 多平面能正常 S_FMT/STREAMON。open_rkisp_v4l2 走 V4L2 直取。
    if (isRkispMainpath(path)) {
        return open_rkisp_v4l2(path, ctx, width, height);
    }
    camera_context_t *c = (camera_context_t *)malloc(sizeof(camera_context_t));
    if (c == NULL) return -1;
    c->cap = NULL;
    c->v4l2_fd = -1;

    // 显式指定 V4L2 后端，避免 OpenCV 把路径当 URI 交给 GStreamer 报错
    try {
        c->cap = new cv::VideoCapture(path, cv::CAP_V4L2);
    } catch (...) {
        c->cap = NULL;
    }

    if (c->cap == NULL || !c->cap->isOpened()) {
        printf("open_camera: %s 无法打开，跳过\n", path);
        if (c->cap != NULL) { c->cap->release(); delete c->cap; }
        free(c);
        return -1;
    }

    // RKISP 默认可能给出 13MP 原始分辨率；先在 V4L2 侧请求 ROS 所需尺寸，
    // 避免每帧在用户态搬运数十 MB 数据。
    if (width > 0) c->cap->set(cv::CAP_PROP_FRAME_WIDTH, width);
    if (height > 0) c->cap->set(cv::CAP_PROP_FRAME_HEIGHT, height);
    if (fps > 0) c->cap->set(cv::CAP_PROP_FPS, fps);

    // 验证能读出一帧（跳过元数据节点 / 无法出帧的设备）
    cv::Mat frame;
    bool ok = false;
    try {
        ok = c->cap->read(frame) && !frame.empty();
    } catch (...) {
        ok = false;
    }
    if (!ok) {
        printf("open_camera: %s 打开但读不到帧，跳过\n", path);
        c->cap->release();
        delete c->cap;
        free(c);
        return -1;
    }

    c->camera_id = -1;
    snprintf(c->device_path, sizeof(c->device_path), "%s", path);
    c->width  = frame.cols;
    c->height = frame.rows;
    printf("open_camera: 已打开 %s, 分辨率 %dx%d\n", path, c->width, c->height);
    *ctx = c;
    return 0;
}

// ═══════════════ 对外接口 ═══════════════

int open_camera(int camera_id, camera_context_t **ctx) {
    char path[32];
    snprintf(path, sizeof(path), "/dev/video%d", camera_id);
    if (try_open_path(path, ctx, 0, 0, 0) == 0) {
        (*ctx)->camera_id = camera_id;
        return 0;
    }
    return -1;
}

int open_camera_path(const char* path, camera_context_t **ctx,
                     int width, int height, int fps) {
    return try_open_path(path, ctx, width, height, fps);
}

int open_camera_auto(camera_context_t **ctx, int width, int height, int fps,
                     bool prefer_mipi) {
    if (ctx == NULL) return -1;

    // 1) 优先 MIPI 的 RKISP 主输出；这里按设备名识别，不依赖 /dev/videoN 编号。
    if (prefer_mipi) {
        std::vector<std::string> mipi_candidates;
        for (const auto& p : listDirPrefix("/dev", "video")) {
            std::string name = p.substr(p.rfind('/') + 1);
            std::string num = name.substr(5);
            if (!num.empty() && std::all_of(num.begin(), num.end(), [](char c) {
                    return isdigit(static_cast<unsigned char>(c));
                }) && isRkispMainpath(p)) {
                mipi_candidates.push_back(p);
            }
        }
        std::sort(mipi_candidates.begin(), mipi_candidates.end(),
                  [](const std::string& a, const std::string& b) { return videoNum(a) < videoNum(b); });
        for (const auto& p : mipi_candidates) {
            if (try_open_path(p.c_str(), ctx, width, height, fps) == 0) return 0;
        }
    }

    // 2) 回退稳定 by-id 视频节点（USB 序列号，设备号变也不变）
    std::vector<std::string> candidates;
    for (const auto& p : listDirPrefix("/dev/v4l/by-id", "")) {
        if (p.find("-video-index0") != std::string::npos)
            candidates.push_back(p);
    }
    // 2) 回退稳定 by-path（USB 端口）
    if (candidates.empty()) {
        for (const auto& p : listDirPrefix("/dev/v4l/by-path", "")) {
            if (p.find("-video-index0") != std::string::npos &&
                (prefer_mipi || p.find("rkisp") == std::string::npos))
                candidates.push_back(p);
        }
    }
    for (const auto& p : candidates) {
        if (try_open_path(p.c_str(), ctx, width, height, fps) == 0) return 0;
    }

    // 3) 兜底：扫描 /dev/video[0-9]*，按编号探测（跳过 dec/enc 与元数据节点）
    std::vector<std::string> vids;
    for (const auto& p : listDirPrefix("/dev", "video")) {
        std::string name = p.substr(p.rfind('/') + 1);
        if (name == "video-dec0" || name == "video-enc0") continue;  // 硬解编/解码器，非摄像头
        // 只要 video<纯数字>
        std::string num = name.substr(5);
        bool digits = !num.empty();
        for (char c : num)
            if (!isdigit(static_cast<unsigned char>(c))) { digits = false; break; }
        if (digits) vids.push_back(p);
    }
    std::sort(vids.begin(), vids.end(),
              [](const std::string& a, const std::string& b) { return videoNum(a) < videoNum(b); });
    for (const auto& p : vids) {
        if (!prefer_mipi && isRkispMainpath(p)) continue;
        if (try_open_path(p.c_str(), ctx, width, height, fps) == 0) return 0;
    }

    printf("open_camera_auto: 未找到可用摄像头\n");
    return -1;
}

int read_camera_frame(camera_context_t *ctx, image_buffer_t *img)
{
    if (ctx == NULL || img == NULL || (ctx->cap == NULL && ctx->v4l2_fd < 0))
        return -1;

    // 1. RKISP mainpath 以 NV12 Multi-Planar MMAP 输出；转换为 RGB888。
    if (ctx->v4l2_fd >= 0) {
        static int v4l2_errors = 0;
        struct pollfd pfd {};
        pfd.fd = ctx->v4l2_fd;
        pfd.events = POLLIN;
        const int poll_ret = poll(&pfd, 1, 1000);
        if (poll_ret <= 0) {
            if (v4l2_errors++ < 3) perror("read_camera_frame: V4L2 poll");
            return -1;
        }

        struct v4l2_plane planes[VIDEO_MAX_PLANES] {};
        struct v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;
        if (xioctl(ctx->v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (v4l2_errors++ < 3) perror("read_camera_frame: VIDIOC_DQBUF");
            return -1;
        }
        if (buf.index >= static_cast<unsigned int>(ctx->v4l2_buffer_count)) {
            return -1;
        }
        cv::Mat nv12(ctx->height * 3 / 2, ctx->width, CV_8UC1,
                     ctx->v4l2_buffers[buf.index]);
        cv::Mat frame_rgb;
        cv::cvtColor(nv12, frame_rgb, cv::COLOR_YUV2RGB_NV12);
        const int queue_ret = xioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf);
        if (queue_ret < 0) {
            if (v4l2_errors++ < 3) perror("read_camera_frame: VIDIOC_QBUF");
            return -1;
        }
        if (frame_rgb.empty()) return -1;

        img->width = frame_rgb.cols;
        img->height = frame_rgb.rows;
        img->format = IMAGE_FORMAT_RGB888;
        img->virt_addr = static_cast<unsigned char *>(malloc(frame_rgb.total() * frame_rgb.elemSize()));
        if (img->virt_addr == NULL) return -1;
        memcpy(img->virt_addr, frame_rgb.data, frame_rgb.total() * frame_rgb.elemSize());
        return 0;
    }

    // 2. USB 等普通设备由 OpenCV 读取（默认 BGR）。
    cv::Mat frame_bgr;
    if (!ctx->cap->read(frame_bgr))
    {
        printf("read_camera_frame: 读取帧失败\n");
        return -1;
    }

    // 2. BGR -> RGB 转换
    cv::Mat frame_rgb;
    cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);

    // 3. 填充 image_buffer_t
    img->width  = frame_rgb.cols;
    img->height = frame_rgb.rows;
    img->format = IMAGE_FORMAT_RGB888;
    img->virt_addr = (unsigned char *)malloc(frame_rgb.total() * frame_rgb.elemSize());
    if (img->virt_addr == NULL)
        return -1;

    memcpy(img->virt_addr, frame_rgb.data, frame_rgb.total() * frame_rgb.elemSize());

    return 0;
}

void close_camera(camera_context_t *ctx)
{
    if (ctx == NULL)
        return;

    if (ctx->v4l2_fd >= 0)
    {
        close_v4l2(ctx);
    }
    if (ctx->cap != NULL)
    {
        ctx->cap->release();
        delete ctx->cap;
        ctx->cap = NULL;
    }

    printf("close_camera: 已关闭 %s\n", ctx->device_path);
    free(ctx);
}
