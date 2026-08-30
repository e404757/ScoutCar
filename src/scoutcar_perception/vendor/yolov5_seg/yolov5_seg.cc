#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "yolov5_seg.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"
#include <chrono>
static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

int init_yolov5_seg_model(const char *model_path, rknn_app_context_t *app_ctx)
{

    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    //加载rknn模型
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, RKNN_FLAG_PRIOR_MEDIUM, NULL);
    rknn_set_core_mask(ctx, RKNN_NPU_CORE_0_1);


    free(model);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    //获取模型输入输出数量
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // Get Model Input Info
    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // Get Model Output Info
    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(output_attrs[i]));
    }

    // Set to context
    app_ctx->rknn_ctx = ctx;

    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type != RKNN_TENSOR_FLOAT16)
    {
        app_ctx->is_quant = true;
    }
    else
    {
        app_ctx->is_quant = false;


    }

    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        app_ctx->model_height = input_attrs[0].dims[1];
        app_ctx->model_width = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    return 0;
}

int release_yolov5_seg_model(rknn_app_context_t *app_ctx)
{
    if (app_ctx->input_attrs != NULL)
    {
        free(app_ctx->input_attrs);
        app_ctx->input_attrs = NULL;
    }
    if (app_ctx->output_attrs != NULL)
    {
        free(app_ctx->output_attrs);
        app_ctx->output_attrs = NULL;
    }
    if (app_ctx->rknn_ctx != 0)
    {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

// CPU 预处理: 裁剪顶部 crop_top 像素后, 把 src(RGB888) 最近邻拉伸到 dst 尺寸。
// 仅用于"非 RGA 兼容内存"的输入(如内存视频帧), 避免 RGA importbuffer_virtualaddr 崩溃。
// src/dst 均为 RGB888, dst 大小 = dst_w*dst_h*3。
static void convert_image_cpu_crop_stretch(const uint8_t* src, int src_w, int src_h,
                                           uint8_t* dst, int dst_w, int dst_h,
                                           int crop_top)
{
    const int crop_h = src_h - crop_top;
    if (crop_h <= 0 || dst_w <= 0 || dst_h <= 0) return;
    for (int dy = 0; dy < dst_h; dy++) {
        // 源行(带裁剪): 0..crop_h-1
        int sy = (int)(((long long)dy * crop_h) / dst_h);
        if (sy >= crop_h) sy = crop_h - 1;
        const uint8_t* srow = src + (size_t)(sy + crop_top) * src_w * 3;
        uint8_t* drow = dst + (size_t)dy * dst_w * 3;
        for (int dx = 0; dx < dst_w; dx++) {
            int sx = (int)(((long long)dx * src_w) / dst_w);
            if (sx >= src_w) sx = src_w - 1;
            drow[dx * 3 + 0] = srow[sx * 3 + 0];
            drow[dx * 3 + 1] = srow[sx * 3 + 1];
            drow[dx * 3 + 2] = srow[sx * 3 + 2];
        }
    }
}

int inference_yolov5_seg_model(rknn_app_context_t *app_ctx, image_buffer_t *img, object_detect_result_list *od_results)
{
    int ret;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[app_ctx->io_num.n_input];
    rknn_output outputs[app_ctx->io_num.n_output];
    const float nms_threshold = NMS_THRESH;
    const float box_conf_threshold = BOX_THRESH;
    int bg_color = 114; // pad color for letterbox

    if ((!app_ctx) || !(img) || (!od_results))
    {
        return -1;
    }

    memset(od_results, 0x00, sizeof(*od_results));
    memset(&letter_box, 0, sizeof(letterbox_t));
    memset(&dst_img, 0, sizeof(image_buffer_t));
    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    
    // Pre Process
    app_ctx->input_image_width = img->width;
    app_ctx->input_image_height = img->height;
    dst_img.width = app_ctx->model_width;
    dst_img.height = app_ctx->model_height;
    dst_img.format = IMAGE_FORMAT_RGB888;
    dst_img.size = get_image_size(&dst_img);
    dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);
    if (dst_img.virt_addr == NULL)
    {
        printf("malloc buffer size:%d fail!\n", dst_img.size);
        return -1;
    }

    // 训练一致的预处理: 裁剪顶部 CROP_TOP_PIXELS 像素 -> 拉伸到 640x640 (非 letterbox)
    {
        image_rect_t src_box;
        src_box.left = 0;
        src_box.top = CROP_TOP_PIXELS;
        src_box.right = img->width - 1;
        src_box.bottom = img->height - 1;

        image_rect_t dst_box;
        dst_box.left = 0;
        dst_box.top = 0;
        dst_box.right = dst_img.width - 1;
        dst_box.bottom = dst_img.height - 1;

        ret = convert_image(img, &dst_img, &src_box, &dst_box, bg_color);
        if (ret < 0)
        {
            printf("convert_image (crop+stretch) fail! ret=%d\n", ret);
            goto out;
        }
    }

    // 坐标映射 (模型 640x640 -> 原图): x_orig = x/scale, y_orig = y_pad + y/scale_y
    letter_box.scale = (float)dst_img.width / (float)img->width;
    letter_box.scale_y = (float)dst_img.height / (float)(img->height - CROP_TOP_PIXELS);
    letter_box.x_pad = 0;
    letter_box.y_pad = CROP_TOP_PIXELS;
    // Set Input Data
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = dst_img.virt_addr;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        goto out;
    }

    // Run
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        goto out;
    }
    {
        rknn_perf_run perf_run;
        ret = rknn_query(app_ctx->rknn_ctx, RKNN_QUERY_PERF_RUN,
                         &perf_run, sizeof(perf_run));
        if (ret == RKNN_SUCC)
        {
            // printf("-- NPU infer: %.3f ms\n", perf_run.run_duration / 1000.0);
        }
    }

    // Get Output (通用API — 输出格式转换复杂，保留)
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < app_ctx->io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = (!app_ctx->is_quant);  // int8 模型取原始 int8（process_i8 反量化），fp16 取 float32
    }
    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // Post Process
    post_process(app_ctx, outputs, &letter_box, box_conf_threshold, nms_threshold, od_results);

    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

out:
    if (dst_img.virt_addr != NULL)
    {
        free(dst_img.virt_addr);
    }

    return ret;
}

// CPU 预处理的推理入口: 与 inference_yolov5_seg_model 唯一区别是预处理用 CPU
// (crop+stretch), 不经过 RGA。供输入为 malloc 普通内存(如内存视频帧)的场景使用,
// 避免 RGA importbuffer_virtualaddr 对非 RGA 兼容内存崩溃。模型与后处理完全复用。
int inference_yolov5_seg_model_cpu(rknn_app_context_t *app_ctx, image_buffer_t *img, object_detect_result_list *od_results)
{
    int ret;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[app_ctx->io_num.n_input];
    rknn_output outputs[app_ctx->io_num.n_output];
    const float nms_threshold = NMS_THRESH;
    const float box_conf_threshold = BOX_THRESH;
    int bg_color = 114; // pad color for letterbox

    if ((!app_ctx) || !(img) || (!od_results))
    {
        return -1;
    }

    memset(od_results, 0x00, sizeof(*od_results));
    memset(&letter_box, 0, sizeof(letterbox_t));
    memset(&dst_img, 0, sizeof(image_buffer_t));
    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    // 期望输入为 RGB888
    if (img->format != IMAGE_FORMAT_RGB888)
    {
        printf("inference_cpu: 仅支持 RGB888 输入\n");
        return -1;
    }

    // Pre Process (CPU)
    app_ctx->input_image_width = img->width;
    app_ctx->input_image_height = img->height;
    dst_img.width = app_ctx->model_width;
    dst_img.height = app_ctx->model_height;
    dst_img.format = IMAGE_FORMAT_RGB888;
    dst_img.size = get_image_size(&dst_img);
    dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);
    if (dst_img.virt_addr == NULL)
    {
        printf("malloc buffer size:%d fail!\n", dst_img.size);
        return -1;
    }

    // 训练一致的预处理: 裁剪顶部 CROP_TOP_PIXELS 像素 -> 拉伸到 640x640 (非 letterbox)，用 CPU 实现
    if (img->height > CROP_TOP_PIXELS) {
        convert_image_cpu_crop_stretch(img->virt_addr, img->width, img->height,
                                       dst_img.virt_addr, dst_img.width, dst_img.height,
                                       CROP_TOP_PIXELS);
    } else {
        // 高度不足裁剪量(异常输入): 全图拉伸
        convert_image_cpu_crop_stretch(img->virt_addr, img->width, img->height,
                                       dst_img.virt_addr, dst_img.width, dst_img.height, 0);
    }

    // 坐标映射 (模型 640x640 -> 原图): x_orig = x/scale, y_orig = y_pad + y/scale_y
    letter_box.scale = (float)dst_img.width / (float)img->width;
    letter_box.scale_y = (float)dst_img.height / (float)(img->height - CROP_TOP_PIXELS);
    letter_box.x_pad = 0;
    letter_box.y_pad = CROP_TOP_PIXELS;
    // Set Input Data
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = dst_img.virt_addr;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        goto out;
    }

    // Run
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        goto out;
    }
    {
        rknn_perf_run perf_run;
        ret = rknn_query(app_ctx->rknn_ctx, RKNN_QUERY_PERF_RUN,
                         &perf_run, sizeof(perf_run));
        if (ret == RKNN_SUCC)
        {
            // printf("-- NPU infer: %.3f ms\n", perf_run.run_duration / 1000.0);
        }
    }

    // Get Output (通用API — 输出格式转换复杂，保留)
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < app_ctx->io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = (!app_ctx->is_quant);  // int8 模型取原始 int8（process_i8 反量化），fp16 取 float32
    }
    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // Post Process
    post_process(app_ctx, outputs, &letter_box, box_conf_threshold, nms_threshold, od_results);

    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

out:
    if (dst_img.virt_addr != NULL)
    {
        free(dst_img.virt_addr);
    }

    return ret;
}