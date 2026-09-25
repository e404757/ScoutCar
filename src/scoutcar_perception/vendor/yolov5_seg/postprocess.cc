
#include "yolov5_seg.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <arm_neon.h>
#include "rknn_matmul_api.h"//调用 NPU 做矩阵乘法，加速 Mask 生成。
#include "im2d.hpp"
#include "dma_alloc.hpp"
#include "drm_alloc.hpp"
#include "Float16.h"
#include "easy_timer.h"

#include <chrono>
#include <cmath>
#include <set>
#include <vector>
// #define USE_FP_RESIZE 0

static char *labels[OBJ_CLASS_NUM];

const int anchor[3][6] = {{10, 13, 16, 30, 33, 23},
                          {30, 61, 62, 45, 59, 119},
                          {116, 90, 156, 198, 373, 326}};

int clamp(float val, int min, int max)//将val限制在min到max之间
{
    return val > min ? (val < max ? val : max) : min;
}

static char *readLine(FILE *fp, char *buffer, int *len)
{
    int ch;
    int i = 0;
    size_t buff_len = 0;

    buffer = (char *)malloc(buff_len + 1);
    if (!buffer)
        return NULL; // Out of memory

    while ((ch = fgetc(fp)) != '\n' && ch != EOF)
    {
        buff_len++;
        void *tmp = realloc(buffer, buff_len + 1);
        if (tmp == NULL)
        {
            free(buffer);
            return NULL; // Out of memory
        }
        buffer = (char *)tmp;

        buffer[i] = (char)ch;
        i++;
    }
    buffer[i] = '\0';

    *len = buff_len;

    // Detect end
    if (ch == EOF && (i == 0 || ferror(fp)))
    {
        free(buffer);
        return NULL;
    }
    return buffer;
}

static int readLines(const char *fileName, char *lines[], int max_line)
{
    FILE *file = fopen(fileName, "r");
    char *s;
    int i = 0;
    int n = 0;

    if (file == NULL)
    {
        printf("Open %s fail!\n", fileName);
        return -1;
    }

    while ((s = readLine(file, s, &n)) != NULL)
    {
        lines[i++] = s;
        if (i >= max_line)
            break;
    }
    fclose(file);
    return i;
}

static int loadLabelName(const char *locationFilename, char *label[])
{
    printf("load lable %s\n", locationFilename);
    readLines(locationFilename, label, OBJ_CLASS_NUM);
    return 0;
}

//计算交并比iou
static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                              float ymax1)
{
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
    return u <= 0.f ? 0.f : (i / u);
}
//对同一类别的检测框，保留置信度最高的，删除与其重叠过多的框
static int nms(int validCount, std::vector<float> &outputLocations, std::vector<int> classIds, std::vector<int> &order,
               int filterId, float threshold)
{
    for (int i = 0; i < validCount; ++i)
    {
        int n = order[i];
        if (n == -1 || classIds[n] != filterId)
        {
            continue;
        }
        for (int j = i + 1; j < validCount; ++j)
        {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId)
            {
                continue;
            }
            float xmin0 = outputLocations[n * 4 + 0];
            float ymin0 = outputLocations[n * 4 + 1];
            float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
            float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

            float xmin1 = outputLocations[m * 4 + 0];
            float ymin1 = outputLocations[m * 4 + 1];
            float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
            float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

            float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);

            if (iou > threshold)
            {
                order[j] = -1;
            }
        }
    }
    return 0;
}

static int quick_sort_indice_inverse(std::vector<float> &input, int left, int right, std::vector<int> &indices)
{
    float key;
    int key_index;
    int low = left;
    int high = right;
    if (left < right)
    {
        key_index = indices[left];
        key = input[left];
        while (low < high)
        {
            while (low < high && input[high] <= key)
            {
                high--;
            }
            input[low] = input[high];
            indices[low] = indices[high];
            while (low < high && input[low] >= key)
            {
                low++;
            }
            input[high] = input[low];
            indices[high] = indices[low];
        }
        input[low] = key;
        indices[low] = key_index;
        quick_sort_indice_inverse(input, left, low - 1, indices);
        quick_sort_indice_inverse(input, low + 1, right, indices);
    }
    return low;
}

void resize_by_opencv_fp(float *input_image, int input_width, int input_height, int boxes_num, float *output_image, int target_width, int target_height)
{
    for (int b = 0; b < boxes_num; b++)
    {
        cv::Mat src_image(input_height, input_width, CV_32F, &input_image[b * input_width * input_height]);
        cv::Mat dst_image;
        cv::resize(src_image, dst_image, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        memcpy(&output_image[b * target_width * target_height], dst_image.data, target_width * target_height * sizeof(float));
    }
}

void resize_by_opencv_uint8(uint8_t *input_image, int input_width, int input_height, int boxes_num, uint8_t *output_image, int target_width, int target_height)
{
    for (int b = 0; b < boxes_num; b++)
    {
        cv::Mat src_image(input_height, input_width, CV_8U, &input_image[b * input_width * input_height]);
        cv::Mat dst_image;
        cv::resize(src_image, dst_image, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        memcpy(&output_image[b * target_width * target_height], dst_image.data, target_width * target_height * sizeof(uint8_t));
    }
}

void resize_by_rga_rk3588(uint8_t *input_image, int input_width, int input_height, uint8_t *output_image, int target_width, int target_height)
{
    char *src_buf, *dst_buf;
    int src_buf_size, dst_buf_size;
    rga_buffer_handle_t src_handle, dst_handle;
    int src_width = input_width;
    int src_height = input_height;
    int src_format = RK_FORMAT_YCbCr_400;
    int dst_width = target_width;
    int dst_height = target_height;
    int dst_format = RK_FORMAT_YCbCr_400;
    int dst_dma_fd, src_dma_fd;
    rga_buffer_t dst = {};
    rga_buffer_t src = {};

    dst_buf_size = dst_width * dst_height * get_bpp_from_format(dst_format);
    src_buf_size = src_width * src_height * get_bpp_from_format(src_format);

    /*
     * Allocate dma_buf within 4G from dma32_heap,
     * return dma_fd and virtual address.
     */
    dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, dst_buf_size, &dst_dma_fd, (void **)&dst_buf);
    dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, src_buf_size, &src_dma_fd, (void **)&src_buf);
    memcpy(src_buf, input_image, src_buf_size);

    dst_handle = importbuffer_fd(dst_dma_fd, dst_buf_size);
    src_handle = importbuffer_fd(src_dma_fd, src_buf_size);

    dst = wrapbuffer_handle(dst_handle, dst_width, dst_height, dst_format);
    src = wrapbuffer_handle(src_handle, src_width, src_height, src_format);

    int ret = imresize(src, dst);
    if (ret == IM_STATUS_SUCCESS)
    {
        printf("%s running success!\n", "rga_resize");
    }
    else
    {
        printf("%s running failed, %s\n", "rga_resize", imStrError((IM_STATUS)ret));
    }

    memcpy(output_image, dst_buf, target_width * target_height);

    releasebuffer_handle(src_handle);
    releasebuffer_handle(dst_handle);
    dma_buf_free(src_buf_size, &src_dma_fd, src_buf);
    dma_buf_free(dst_buf_size, &dst_dma_fd, dst_buf);
}

class DrmObject
{
public:
    int drm_buffer_fd;
    int drm_buffer_handle;
    size_t actual_size;
    uint8_t *drm_buf;
};

//把每个目标的掩膜裁剪到对应检测框范围内，并合并到一张图。
void crop_mask_fp(float *seg_mask, uint8_t *all_mask_in_one, float *boxes, int boxes_num, int *cls_id, int height, int width)
{
    for (int b = 0; b < boxes_num; b++)
    {
        float x1 = boxes[b * 4 + 0];
        float y1 = boxes[b * 4 + 1];
        float x2 = boxes[b * 4 + 2];
        float y2 = boxes[b * 4 + 3];

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (j >= x1 && j < x2 && i >= y1 && i < y2)
                {
                    if (all_mask_in_one[i * width + j] == 0)
                    {
                        if (seg_mask[b * width * height + i * width + j] > 0)
                        {
                            all_mask_in_one[i * width + j] = (cls_id[b] + 1);
                        }
                        else
                        {
                            all_mask_in_one[i * width + j] = 0;
                        }
                    }
                }
            }
        }
    }
}

void crop_mask_uint8(uint8_t *seg_mask, uint8_t *all_mask_in_one, float *boxes, int boxes_num, int *cls_id, int height, int width)
{
    for (int b = 0; b < boxes_num; b++)
    {
        float x1 = boxes[b * 4 + 0];
        float y1 = boxes[b * 4 + 1];
        float x2 = boxes[b * 4 + 2];
        float y2 = boxes[b * 4 + 3];

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (j >= x1 && j < x2 && i >= y1 && i < y2)
                {
                    if (all_mask_in_one[i * width + j] == 0)
                    {
                        if (seg_mask[b * width * height + i * width + j] > 0)
                        {
                            all_mask_in_one[i * width + j] = (cls_id[b] + 1);
                        }
                        else
                        {
                            all_mask_in_one[i * width + j] = 0;
                        }
                    }
                }
            }
        }
    }
}

//矩阵乘法（掩膜生成核心）
void matmul_by_cpu_fp(std::vector<float> &A, float *B, float *C, int ROWS_A, int COLS_A, int COLS_B)
{

    float temp = 0;
    for (int i = 0; i < ROWS_A; i++)
    {
        for (int j = 0; j < COLS_B; j++)
        {
            temp = 0;
            for (int k = 0; k < COLS_A; k++)
            {
                temp += A[i * COLS_A + k] * B[k * COLS_B + j];
            }
            C[i * COLS_B + j] = temp;
        }
    }
}

void matmul_by_cpu_uint8(std::vector<float> &A, float *B, uint8_t *C, int ROWS_A, int COLS_A, int COLS_B)
{

    float temp = 0;
    for (int i = 0; i < ROWS_A; i++)
    {
        for (int j = 0; j < COLS_B; j++)
        {
            temp = 0;
            for (int k = 0; k < COLS_A; k++)
            {
                temp += A[i * COLS_A + k] * B[k * COLS_B + j];
            }
            if (temp > 0)
            {
                C[i * COLS_B + j] = 4;
            }
            else
            {
                C[i * COLS_B + j] = 0;
            }
        }
    }
}

void matmul_by_npu_fp(std::vector<float> &A_input, float *B_input, float *C_input, int ROWS_A, int COLS_A, int COLS_B, seg_rknn_app_context_t *app_ctx)
{
    int B_layout = 0;
    int AC_layout = 0;
    int32_t M = ROWS_A;
    int32_t K = COLS_A;
    int32_t N = COLS_B;

    rknn_matmul_ctx ctx;
    rknn_matmul_info info;
    memset(&info, 0, sizeof(rknn_matmul_info));
    info.M = M;
    info.K = K;
    info.N = N;
    info.type = RKNN_FLOAT16_MM_FLOAT16_TO_FLOAT32;
    info.B_layout = B_layout;
    info.AC_layout = AC_layout;

    rknn_matmul_io_attr io_attr;
    memset(&io_attr, 0, sizeof(rknn_matmul_io_attr));

    rknpu2::float16 int8Vector_A[ROWS_A * COLS_A];
    for (int i = 0; i < ROWS_A * COLS_A; ++i)
    {
        int8Vector_A[i] = (rknpu2::float16)A_input[i];
    }

    rknpu2::float16 int8Vector_B[COLS_A * COLS_B];
    for (int i = 0; i < COLS_A * COLS_B; ++i)
    {
        int8Vector_B[i] = (rknpu2::float16)B_input[i];
    }

    int ret = rknn_matmul_create(&ctx, &info, &io_attr);
    // Create A
    rknn_tensor_mem *A = rknn_create_mem(ctx, io_attr.A.size);
    // Create B
    rknn_tensor_mem *B = rknn_create_mem(ctx, io_attr.B.size);
    // Create C
    rknn_tensor_mem *C = rknn_create_mem(ctx, io_attr.C.size);

    memcpy(A->virt_addr, int8Vector_A, A->size);
    memcpy(B->virt_addr, int8Vector_B, B->size);

    // Set A
    ret = rknn_matmul_set_io_mem(ctx, A, &io_attr.A);
    // Set B
    ret = rknn_matmul_set_io_mem(ctx, B, &io_attr.B);
    // Set C
    ret = rknn_matmul_set_io_mem(ctx, C, &io_attr.C);

    // Run
    ret = rknn_matmul_run(ctx);
    for (int i = 0; i < ROWS_A * COLS_B; ++i)
    {
        C_input[i] = ((float *)C->virt_addr)[i];
    }

    // destroy
    rknn_destroy_mem(ctx, A);
    rknn_destroy_mem(ctx, B);
    rknn_destroy_mem(ctx, C);
    rknn_matmul_destroy(ctx);
}

// 复用 matmul 上下文；框数增大时才扩容，避免每帧重复创建 NPU 内存。
struct SegMaskMatmul {
    rknn_matmul_ctx ctx = 0;
    rknn_matmul_io_attr attr{};
    rknn_tensor_mem *a = nullptr;
    rknn_tensor_mem *b = nullptr;
    rknn_tensor_mem *c = nullptr;
    int capacity = 0;
};

static SegMaskMatmul mask_matmul;

// RK3588 的 FP16 指令只用于输入转换，其余后处理仍用原有编译配置。
__attribute__((target("arch=armv8.2-a+fp16"), optimize("O3")))
static void convert_mask_input_fp16(const float *src, void *dst_data, int count)
{
    auto *dst = static_cast<__fp16 *>(dst_data);
    int i = 0;
    for (; i + 4 <= count; i += 4) {
        vst1_f16(dst + i, vcvt_f16_f32(vld1q_f32(src + i)));
    }
    for (; i < count; ++i) dst[i] = src[i];
}

// 量化 proto 直接反量化到 MatMul 的 FP16 B 缓冲区，省去 float32 中间数组。
__attribute__((target("arch=armv8.2-a+fp16"), optimize("O3")))
static void convert_quant_proto_fp16(const int8_t *src, void *dst_data,
                                     int count, int32_t zero_point, float scale)
{
    auto *dst = static_cast<__fp16 *>(dst_data);
    for (int i = 0; i < count; ++i) {
        dst[i] = (static_cast<float>(src[i]) - static_cast<float>(zero_point)) * scale;
    }
}

struct SegMaskMatmulTiming {
    double setup_ms = 0;
    double convert_ms = 0;
    double bind_ms = 0;
    double sync_in_ms = 0;
    double run_ms = 0;
    double sync_out_ms = 0;
    double binary_ms = 0;
};

static void release_mask_matmul()
{
    if (mask_matmul.a) rknn_destroy_mem(mask_matmul.ctx, mask_matmul.a);
    if (mask_matmul.b) rknn_destroy_mem(mask_matmul.ctx, mask_matmul.b);
    if (mask_matmul.c) rknn_destroy_mem(mask_matmul.ctx, mask_matmul.c);
    if (mask_matmul.ctx) rknn_matmul_destroy(mask_matmul.ctx);
    mask_matmul = SegMaskMatmul{};
}

static int matmul_by_npu_uint8(const std::vector<float> &coefficients,
                              const float *proto, const int8_t *quant_proto,
                              int32_t proto_zero_point, float proto_scale,
                              uint8_t *mask, int boxes_num,
                              SegMaskMatmulTiming *timing)
{
    using Clock = std::chrono::steady_clock;
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const auto t_start = Clock::now();
    constexpr int k = PROTO_CHANNEL;
    constexpr int n = PROTO_HEIGHT * PROTO_WEIGHT;
    bool created = false;
    if (boxes_num > mask_matmul.capacity) {
        release_mask_matmul();
        rknn_matmul_info info{};
        info.M = boxes_num;
        info.K = k;
        info.N = n;
        info.type = RKNN_FLOAT16_MM_FLOAT16_TO_FLOAT32;
        int ret = rknn_matmul_create(&mask_matmul.ctx, &info, &mask_matmul.attr);
        if (ret != RKNN_SUCC) {
            release_mask_matmul();
            return ret;
        }
        ret = rknn_matmul_set_core_mask(mask_matmul.ctx, RKNN_NPU_CORE_2);
        if (ret != RKNN_SUCC) {
            release_mask_matmul();
            return ret;
        }
        mask_matmul.a = rknn_create_mem(mask_matmul.ctx, mask_matmul.attr.A.size);
        mask_matmul.b = rknn_create_mem(mask_matmul.ctx, mask_matmul.attr.B.size);
        mask_matmul.c = rknn_create_mem(mask_matmul.ctx, mask_matmul.attr.C.size);
        if (!mask_matmul.a || !mask_matmul.b || !mask_matmul.c) {
            release_mask_matmul();
            return -1;
        }
        mask_matmul.capacity = boxes_num;
        created = true;
    }

    const auto t_setup = Clock::now();
    convert_mask_input_fp16(coefficients.data(), mask_matmul.a->virt_addr,
                            boxes_num * k);
    auto *a = static_cast<__fp16 *>(mask_matmul.a->virt_addr);
    for (int i = boxes_num * k; i < mask_matmul.capacity * k; ++i) a[i] = 0.0f;
    if (quant_proto) {
        convert_quant_proto_fp16(quant_proto, mask_matmul.b->virt_addr,
                                 k * n, proto_zero_point, proto_scale);
    } else {
        convert_mask_input_fp16(proto, mask_matmul.b->virt_addr, k * n);
    }
    const auto t_convert = Clock::now();

    // 每帧更新 proto 后重新绑定 B，保留现有正确路径。
    int bind_ret = RKNN_SUCC;
    if (created) bind_ret = rknn_matmul_set_io_mem(mask_matmul.ctx, mask_matmul.a, &mask_matmul.attr.A);
    if (bind_ret == RKNN_SUCC) bind_ret = rknn_matmul_set_io_mem(mask_matmul.ctx, mask_matmul.b, &mask_matmul.attr.B);
    if (created && bind_ret == RKNN_SUCC) bind_ret = rknn_matmul_set_io_mem(mask_matmul.ctx, mask_matmul.c, &mask_matmul.attr.C);
    if (bind_ret != RKNN_SUCC) {
        release_mask_matmul();
        return bind_ret;
    }
    const auto t_bind = Clock::now();
    int ret = rknn_mem_sync(mask_matmul.ctx, mask_matmul.a, RKNN_MEMORY_SYNC_TO_DEVICE);
    if (ret != RKNN_SUCC) return ret;
    ret = rknn_mem_sync(mask_matmul.ctx, mask_matmul.b, RKNN_MEMORY_SYNC_TO_DEVICE);
    if (ret != RKNN_SUCC) return ret;
    const auto t_sync_in = Clock::now();
    ret = rknn_matmul_run(mask_matmul.ctx);
    if (ret != RKNN_SUCC) return ret;
    const auto t_run = Clock::now();
    ret = rknn_mem_sync(mask_matmul.ctx, mask_matmul.c, RKNN_MEMORY_SYNC_FROM_DEVICE);
    if (ret != RKNN_SUCC) return ret;
    const auto t_sync_out = Clock::now();

    const auto *c = static_cast<const float *>(mask_matmul.c->virt_addr);
    for (int i = 0; i < boxes_num * n; ++i) mask[i] = c[i] > 0.0f ? 4 : 0;
    const auto t_end = Clock::now();
    timing->setup_ms = ms(t_start, t_setup);
    timing->convert_ms = ms(t_setup, t_convert);
    timing->bind_ms = ms(t_convert, t_bind);
    timing->sync_in_ms = ms(t_bind, t_sync_in);
    timing->run_ms = ms(t_sync_in, t_run);
    timing->sync_out_ms = ms(t_run, t_sync_out);
    timing->binary_ms = ms(t_sync_out, t_end);
    return 0;
}

// 去掉 letterbox 填充，还原到原图下方的裁剪区域。
void seg_reverse(uint8_t *seg_mask, uint8_t *seg_mask_real,
                 int model_in_height, int model_in_width, int ori_in_height,
                 int ori_in_width, const letterbox_t *letter_box)
{
    const int crop_h = ori_in_height - CROP_TOP_PIXELS;
    const int resized_w = std::min(model_in_width - letter_box->x_pad,
                                   (int)std::round(ori_in_width * letter_box->scale));
    const int resized_h = std::min(model_in_height - letter_box->y_pad,
                                   (int)std::round(crop_h * letter_box->scale_y));
    cv::Mat model_mask(model_in_height, model_in_width, CV_8U, seg_mask);
    cv::Mat original_mask(ori_in_height, ori_in_width, CV_8U, seg_mask_real);
    original_mask.setTo(0);
    cv::resize(model_mask(cv::Rect(letter_box->x_pad, letter_box->y_pad,
                                  resized_w, resized_h)),
               original_mask(cv::Rect(0, CROP_TOP_PIXELS, ori_in_width, crop_h)),
               cv::Size(ori_in_width, crop_h), 0, 0, cv::INTER_LINEAR);
}
//检测框坐标还原
int box_reverse(int position, int boundary, int pad, float scale)
{
    return (int)((clamp(position, 0, boundary) - pad) / scale);
}

static float sigmoid(float x) { return 1.0 / (1.0 + expf(-x)); }

static float unsigmoid(float y) { return -1.0 * logf((1.0 / y) - 1.0); }

inline static int32_t __clip(float val, float min, float max)
{
    float f = val <= min ? min : (val >= max ? max : val);
    return f;
}

//将 float32 量化为 int8
static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale)
{
    float dst_val = (f32 / scale) + zp;
    int8_t res = (int8_t)__clip(dst_val, -128, 127);
    return res;
}

//将 int8 反量化为 float32
static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }

//INT8 量化版解析
static int process_i8(rknn_output *all_input, int input_id, int *anchor, int grid_h, int grid_w, int height, int width, int stride,
                      std::vector<float> &boxes, std::vector<float> &segments, float *proto, std::vector<float> &objProbs, std::vector<int> &classId, float threshold,
                      float *raw_class_score, seg_rknn_app_context_t *app_ctx)
{

    int validCount = 0;
    int grid_len = grid_h * grid_w;

    if (input_id % 2 == 1)
    {
        return validCount;
    }

    if (input_id == 6)
    {
        // 有候选框时，proto 在 MatMul 输入准备阶段直接写入 FP16 B。
        return validCount;
    }

    int8_t *input = (int8_t *)all_input[input_id].buf;
    int8_t *input_seg = (int8_t *)all_input[input_id + 1].buf;
    int32_t zp = app_ctx->output_attrs[input_id].zp;
    float scale = app_ctx->output_attrs[input_id].scale;
    int32_t zp_seg = app_ctx->output_attrs[input_id + 1].zp;
    float scale_seg = app_ctx->output_attrs[input_id + 1].scale;

    int8_t thres_i8 = qnt_f32_to_affine(threshold, zp, scale);

    for (int a = 0; a < 3; a++)
    {
        for (int i = 0; i < grid_h; i++)
        {
            for (int j = 0; j < grid_w; j++)
            {
                int8_t box_confidence = input[(PROP_BOX_SIZE * a + 4) * grid_len + i * grid_w + j];
                if (box_confidence >= thres_i8)
                {
                    int offset = (PROP_BOX_SIZE * a) * grid_len + i * grid_w + j;
                    int offset_seg = (PROTO_CHANNEL * a) * grid_len + i * grid_w + j;
                    int8_t *in_ptr = input + offset;
                    int8_t *in_ptr_seg = input_seg + offset_seg;

                    float box_x = (deqnt_affine_to_f32(*in_ptr, zp, scale)) * 2.0 - 0.5;
                    float box_y = (deqnt_affine_to_f32(in_ptr[grid_len], zp, scale)) * 2.0 - 0.5;
                    float box_w = (deqnt_affine_to_f32(in_ptr[2 * grid_len], zp, scale)) * 2.0;
                    float box_h = (deqnt_affine_to_f32(in_ptr[3 * grid_len], zp, scale)) * 2.0;
                    box_x = (box_x + j) * (float)stride;
                    box_y = (box_y + i) * (float)stride;
                    box_w = box_w * box_w * (float)anchor[a * 2];
                    box_h = box_h * box_h * (float)anchor[a * 2 + 1];
                    box_x -= (box_w / 2.0);
                    box_y -= (box_h / 2.0);

                    int8_t maxClassProbs = in_ptr[5 * grid_len];
                    int maxClassId = 0;
                    for (int k = 1; k < OBJ_CLASS_NUM; ++k)
                    {
                        int8_t prob = in_ptr[(5 + k) * grid_len];
                        if (prob > maxClassProbs)
                        {
                            maxClassId = k;
                            maxClassProbs = prob;
                        }
                    }

                    float box_conf_f32 = deqnt_affine_to_f32(box_confidence, zp, scale);
                    float class_prob_f32 = deqnt_affine_to_f32(maxClassProbs, zp, scale);
                    float limit_score = box_conf_f32 * class_prob_f32;
                    // 记录每类"未过滤最高分"(不过 BOX_THRESH 阈值也记录)，用于标定阈值。
                    for (int k = 0; k < OBJ_CLASS_NUM; ++k)
                    {
                        float cls_prob_k = deqnt_affine_to_f32(in_ptr[(5 + k) * grid_len], zp, scale);
                        float score_k = box_conf_f32 * cls_prob_k;
                        if (score_k > raw_class_score[k]) raw_class_score[k] = score_k;
                    }
                    // if (maxClassProbs > thres_i8)
                    if (limit_score > threshold)
                    {
                        for (int k = 0; k < PROTO_CHANNEL; k++)
                        {
                            float seg_element_fp = deqnt_affine_to_f32(in_ptr_seg[(k)*grid_len], zp_seg, scale_seg);
                            segments.push_back(seg_element_fp);
                        }

                        objProbs.push_back((deqnt_affine_to_f32(maxClassProbs, zp, scale)) * (deqnt_affine_to_f32(box_confidence, zp, scale)));
                        classId.push_back(maxClassId);
                        validCount++;
                        boxes.push_back(box_x);
                        boxes.push_back(box_y);
                        boxes.push_back(box_w);
                        boxes.push_back(box_h);
                    }
                }
            }
        }
    }
    return validCount;
}

static int process_fp32(rknn_output *all_input, int input_id, int *anchor, int grid_h, int grid_w, int height, int width, int stride,
                        std::vector<float> &boxes, std::vector<float> &segments, float *proto, std::vector<float> &objProbs, std::vector<int> &classId, float threshold,
                        float *raw_class_score)
{
    int validCount = 0;
    int grid_len = grid_h * grid_w;

    if (input_id % 2 == 1)
    {
        return validCount;
    }

    if (input_id == 6)
    {
        float *input_proto = (float *)all_input[input_id].buf;
        for (int i = 0; i < PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT; i++)
        {
            proto[i] = input_proto[i];
        }
        return validCount;
    }

    float *input = (float *)all_input[input_id].buf;
    float *input_seg = (float *)all_input[input_id + 1].buf;

    for (int a = 0; a < 3; a++)
    {
        for (int i = 0; i < grid_h; i++)
        {
            for (int j = 0; j < grid_w; j++)
            {
                float box_confidence = input[(PROP_BOX_SIZE * a + 4) * grid_len + i * grid_w + j];
                if (box_confidence >= threshold)
                {
                    int offset = (PROP_BOX_SIZE * a) * grid_len + i * grid_w + j;
                    int offset_seg = (PROTO_CHANNEL * a) * grid_len + i * grid_w + j;
                    float *in_ptr = input + offset;
                    float *in_ptr_seg = input_seg + offset_seg;

                    float box_x = *in_ptr * 2.0 - 0.5;
                    float box_y = in_ptr[grid_len] * 2.0 - 0.5;
                    float box_w = in_ptr[2 * grid_len] * 2.0;
                    float box_h = in_ptr[3 * grid_len] * 2.0;
                    box_x = (box_x + j) * (float)stride;
                    box_y = (box_y + i) * (float)stride;
                    box_w = box_w * box_w * (float)anchor[a * 2];
                    box_h = box_h * box_h * (float)anchor[a * 2 + 1];
                    box_x -= (box_w / 2.0);
                    box_y -= (box_h / 2.0);

                    float maxClassProbs = in_ptr[5 * grid_len];
                    int maxClassId = 0;
                    for (int k = 1; k < OBJ_CLASS_NUM; ++k)
                    {
                        float prob = in_ptr[(5 + k) * grid_len];
                        if (prob > maxClassProbs)
                        {
                            maxClassId = k;
                            maxClassProbs = prob;
                        }
                    }
                    float limit_score = maxClassProbs * box_confidence;
                    // 记录每类"未过滤最高分"(不过 BOX_THRESH 阈值也记录)，用于标定阈值。
                    for (int k = 0; k < OBJ_CLASS_NUM; ++k)
                    {
                        float score_k = box_confidence * in_ptr[(5 + k) * grid_len];
                        if (score_k > raw_class_score[k]) raw_class_score[k] = score_k;
                    }
                    // if (maxClassProbs > threshold)
                    if (limit_score > threshold)
                    {
                        for (int k = 0; k < PROTO_CHANNEL; k++)
                        {
                            float seg_element_f32 = in_ptr_seg[(k)*grid_len];
                            segments.push_back(seg_element_f32);
                        }

                        objProbs.push_back(maxClassProbs * box_confidence);
                        classId.push_back(maxClassId);
                        validCount++;
                        boxes.push_back(box_x);
                        boxes.push_back(box_y);
                        boxes.push_back(box_w);
                        boxes.push_back(box_h);
                    }
                }
            }
        }
    }
    return validCount;
}

int seg_post_process(seg_rknn_app_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, seg_object_detect_result_list *od_results)
{
    using Clock = std::chrono::steady_clock;
    const auto ms = [](auto start, auto end) {
        return std::chrono::duration<double, std::milli>(end - start).count();
    };
    const auto t0 = Clock::now();

    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;

    std::vector<float> filterSegments;
    float proto[PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT];
    std::vector<float> filterSegments_by_nms;

    int model_in_width = app_ctx->model_width;
    int model_in_height = app_ctx->model_height;

    int validCount = 0;
    int stride = 0;
    int grid_h = 0;
    int grid_w = 0;

    // 未过滤(阈值前)的每类最高分，用于标定 BOX_THRESH
    float raw_class_score[OBJ_CLASS_NUM] = {0.f, 0.f};

    memset(od_results, 0, sizeof(seg_object_detect_result_list));

    // 1.解析模型输出
    double parse_stage_ms[4] = {};  // 依次存输出 0、2、4、6 的耗时
    int parse_stage_valid[3] = {};  // 输出 0、2、4 各产生多少候选框
    for (int i = 0; i < 7; i++)
    {
        grid_h = app_ctx->output_attrs[i].dims[2];
        grid_w = app_ctx->output_attrs[i].dims[3];
        stride = model_in_height / grid_h;

        const auto stage_begin = Clock::now();
        int added = 0;
        if (i == 6 && validCount == 0) {
            break;
        }
        if (app_ctx->is_quant)
        {
           added = process_i8(outputs, i, (int *)anchor[i / 2], grid_h, grid_w, model_in_height, model_in_width, stride, filterBoxes, filterSegments, proto, objProbs,
                                     classId, conf_threshold, raw_class_score, app_ctx);
        }
        else
        {
            added = process_fp32(outputs, i, (int *)anchor[i / 2], grid_h, grid_w, model_in_height, model_in_width, stride, filterBoxes, filterSegments, proto, objProbs,
                                       classId, conf_threshold, raw_class_score);
        }
        const auto stage_end = Clock::now();
        validCount += added;
        if (i % 2 == 0) {
        parse_stage_ms[i / 2] = ms(stage_begin, stage_end);
        if (i < 6) {
                parse_stage_valid[i / 2] = added;
        }
    }
    }

    // 无论是否为 0，都写回原始每类最高分(供遥测画真实置信度分布)
    for (int k = 0; k < OBJ_CLASS_NUM; ++k)
        od_results->raw_class_score[k] = raw_class_score[k];

    const auto t_parse = Clock::now();
    printf(
        "Seg parse: det0=%.2f(%d) det2=%.2f(%d) "
        "det4=%.2f(%d) proto=%.2f total=%.2f ms\n",
        parse_stage_ms[0], parse_stage_valid[0],
        parse_stage_ms[1], parse_stage_valid[1],
        parse_stage_ms[2], parse_stage_valid[2],
        parse_stage_ms[3], ms(t0, t_parse));

    // 2.nms
    if (validCount <= 0)
    {
        return 0;
    }
    std::vector<int> indexArray;
    for (int i = 0; i < validCount; ++i)
    {
        indexArray.push_back(i);
    }

    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> class_set(std::begin(classId), std::end(classId));

    for (auto c : class_set)
    {
        nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
    }

    int last_count = 0;
    od_results->count = 0;

    //3.收集nms结果
    for (int i = 0; i < validCount; ++i)
    {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE)
        {
            continue;
        }
        int n = indexArray[i];

        float x1 = filterBoxes[n * 4 + 0];
        float y1 = filterBoxes[n * 4 + 1];
        float x2 = x1 + filterBoxes[n * 4 + 2];
        float y2 = y1 + filterBoxes[n * 4 + 3];
        int id = classId[n];
        float obj_conf = objProbs[i];

        for (int k = 0; k < PROTO_CHANNEL; k++)
        {
            filterSegments_by_nms.push_back(filterSegments[n * PROTO_CHANNEL + k]);
        }

        od_results->results[last_count].box.left = x1;
        od_results->results[last_count].box.top = y1;
        od_results->results[last_count].box.right = x2;
        od_results->results[last_count].box.bottom = y2;

        od_results->results[last_count].prop = obj_conf;
        od_results->results[last_count].cls_id = id;
        last_count++;
    }
    od_results->count = last_count;
    const int boxes_num = od_results->count;

    const auto t_nms = Clock::now();

    //坐标还原
    float filterBoxes_by_nms[boxes_num * 4];
    int cls_id[boxes_num];
    for (int i = 0; i < boxes_num; i++)
    {
        // for crop_mask
        filterBoxes_by_nms[i * 4 + 0] = od_results->results[i].box.left;   // x1;
        filterBoxes_by_nms[i * 4 + 1] = od_results->results[i].box.top;    // y1;
        filterBoxes_by_nms[i * 4 + 2] = od_results->results[i].box.right;  // x2;
        filterBoxes_by_nms[i * 4 + 3] = od_results->results[i].box.bottom; // y2;
        cls_id[i] = od_results->results[i].cls_id;

        // 去掉填充，映回裁剪区域，再加上原图顶部裁掉的 120 行。
        od_results->results[i].box.left = clamp(
            (od_results->results[i].box.left - letter_box->x_pad) / letter_box->scale,
            0, app_ctx->input_image_width - 1);
        od_results->results[i].box.top = clamp(
            CROP_TOP_PIXELS +
                (od_results->results[i].box.top - letter_box->y_pad) / letter_box->scale_y,
            CROP_TOP_PIXELS, app_ctx->input_image_height - 1);
        od_results->results[i].box.right = clamp(
            (od_results->results[i].box.right - letter_box->x_pad) / letter_box->scale,
            0, app_ctx->input_image_width - 1);
        od_results->results[i].box.bottom = clamp(
            CROP_TOP_PIXELS +
                (od_results->results[i].box.bottom - letter_box->y_pad) / letter_box->scale_y,
            CROP_TOP_PIXELS, app_ctx->input_image_height - 1);
    }

    TIMER timer;
#ifdef USE_FP_RESIZE
    timer.tik();
    // compute the mask through Matmul
    int ROWS_A = boxes_num;
    int COLS_A = PROTO_CHANNEL;
    int COLS_B = PROTO_HEIGHT * PROTO_WEIGHT;
    float *matmul_out = (float *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT * sizeof(float));
    // matmul_by_cpu_fp(filterSegments_by_nms, proto, matmul_out, ROWS_A, COLS_A, COLS_B);
    matmul_by_npu_fp(filterSegments_by_nms, proto, matmul_out, ROWS_A, COLS_A, COLS_B, app_ctx);
    timer.tok();
    // timer.print_time("matmul_by_npu_fp");

    timer.tik();
    // resize to (boxes_num, model_in_width, model_in_height)
    float *seg_mask = (float *)malloc(boxes_num * model_in_height * model_in_width * sizeof(float));
    resize_by_opencv_fp(matmul_out, PROTO_WEIGHT, PROTO_HEIGHT, boxes_num, seg_mask, model_in_width, model_in_height);
    timer.tok();
    timer.print_time("resize_by_opencv_fp");

    timer.tik();
    // crop mask
    uint8_t *all_mask_in_one = (uint8_t *)malloc(model_in_height * model_in_width * sizeof(uint8_t));
    memset(all_mask_in_one, 0, model_in_height * model_in_width * sizeof(uint8_t));
    crop_mask_fp(seg_mask, all_mask_in_one, filterBoxes_by_nms, boxes_num, cls_id, model_in_height, model_in_width);
    timer.tok();
    timer.print_time("crop_mask_fp");
#else
    timer.tik();
    // 只在 NPU 上计算矩阵，保持后续 uint8 resize/crop 路径不变。
    uint8_t *matmul_out = (uint8_t *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT * sizeof(uint8_t));
    SegMaskMatmulTiming matmul_timing;
    const int8_t *quant_proto = app_ctx->is_quant
        ? static_cast<const int8_t *>(outputs[6].buf) : nullptr;
    const int matmul_ret = matmul_by_npu_uint8(
        filterSegments_by_nms, proto, quant_proto,
        app_ctx->output_attrs[6].zp, app_ctx->output_attrs[6].scale,
        matmul_out, boxes_num, &matmul_timing);
    if (matmul_ret != 0) {
        free(matmul_out);
        return matmul_ret;
    }

    timer.tok();
    timer.print_time("matmul_by_npu_uint8");
    const float matmul_ms = timer.get_time();

    timer.tik();
    uint8_t *seg_mask = (uint8_t *)malloc(boxes_num * model_in_height * model_in_width * sizeof(uint8_t));
    resize_by_opencv_uint8(matmul_out, PROTO_WEIGHT, PROTO_HEIGHT, boxes_num, seg_mask, model_in_width, model_in_height);
    timer.tok();
    const float resize_ms = timer.get_time();
    timer.print_time("resize_by_opencv_uint8");

    timer.tik();
    // crop mask
    uint8_t *all_mask_in_one = (uint8_t *)malloc(model_in_height * model_in_width * sizeof(uint8_t));
    memset(all_mask_in_one, 0, model_in_height * model_in_width * sizeof(uint8_t));
    crop_mask_uint8(seg_mask, all_mask_in_one, filterBoxes_by_nms, boxes_num, cls_id, model_in_height, model_in_width);
    timer.tok();
    const float crop_ms = timer.get_time();
    timer.print_time("crop_mask_uint8");
    printf("Seg mask: boxes=%d matmul=%.2f (setup=%.2f convert=%.2f bind=%.2f sync_in=%.2f run=%.2f sync_out=%.2f binary=%.2f) resize=%.2f crop=%.2f ms\n",
           boxes_num, matmul_ms, matmul_timing.setup_ms,
           matmul_timing.convert_ms, matmul_timing.bind_ms,
           matmul_timing.sync_in_ms, matmul_timing.run_ms,
           matmul_timing.sync_out_ms, matmul_timing.binary_ms,
           resize_ms, crop_ms);
#endif

    timer.tik();
    
    int ori_in_height = app_ctx->input_image_height;
    int ori_in_width = app_ctx->input_image_width;
    uint8_t *real_seg_mask = (uint8_t *)malloc(ori_in_height * ori_in_width * sizeof(uint8_t));
    const auto t_mask = Clock::now();
    seg_reverse(all_mask_in_one, real_seg_mask,
                model_in_height, model_in_width, ori_in_height, ori_in_width, letter_box);
    od_results->results_seg[0].seg_mask = real_seg_mask;
    free(all_mask_in_one);
    free(seg_mask);
    free(matmul_out);
    timer.tok();
    timer.print_time("seg_reverse");
    const auto t_end = Clock::now();

    printf("Seg post: valid=%d boxes=%d parse=%.2f nms=%.2f mask=%.2f reverse=%.2f ms\n",
        validCount, boxes_num,
        ms(t0, t_parse), ms(t_parse, t_nms),
        ms(t_nms, t_mask), ms(t_mask, t_end));

    return 0;
}

int seg_init_post_process(const char *label_path)
{
    int ret = 0;
    ret = loadLabelName(label_path, labels);
    if (ret < 0)
    {
        printf("Load %s failed!\n", label_path);
        return -1;
    }
    return 0;
}

char *seg_cls_to_name(int cls_id)
{

    if (cls_id >= OBJ_CLASS_NUM)
    {
        return "null";
    }

    if (labels[cls_id])
    {
        return labels[cls_id];
    }

    return "null";
}

void seg_deinit_post_process()
{
    release_mask_matmul();
    for (int i = 0; i < OBJ_CLASS_NUM; i++)
    {
        {
            free(labels[i]);
            labels[i] = nullptr;
        }
    }
}