#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "yolov5_seg.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

int init_yolov5_seg_model(const char *model_path, seg_rknn_app_context_t *app_ctx)
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

int release_yolov5_seg_model(seg_rknn_app_context_t *app_ctx)
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

// 与训练图片一致：先裁顶部 120 行，再等比缩放并居中填充 114。
static void convert_image_letterbox(const uint8_t* src, int src_w, int src_h,
                                        uint8_t* dst, int dst_w, int dst_h,
                                        letterbox_t* letter_box)
{
    const int crop_h = src_h - CROP_TOP_PIXELS;
    const float scale = std::min((float)dst_w / src_w, (float)dst_h / crop_h);
    const int resize_w = (int)std::round(src_w * scale);
    const int resize_h = (int)std::round(crop_h * scale);
    const int left = (dst_w - resize_w) / 2;
    const int top = (dst_h - resize_h) / 2;
    cv::Mat source(src_h, src_w, CV_8UC3, const_cast<uint8_t*>(src));
    cv::Mat target(dst_h, dst_w, CV_8UC3, dst);
    target.setTo(cv::Scalar(114, 114, 114));
    cv::resize(source(cv::Rect(0, CROP_TOP_PIXELS, src_w, crop_h)),
               target(cv::Rect(left, top, resize_w, resize_h)),
               cv::Size(resize_w, resize_h), 0, 0, cv::INTER_LINEAR);
    letter_box->scale = (float)resize_w / src_w;
    letter_box->scale_y = (float)resize_h / crop_h;
    letter_box->x_pad = left;
    letter_box->y_pad = top;
}

// 分割推理：裁剪顶部后进行 letterbox，再运行模型和后处理。
int inference_yolov5_seg_model(seg_rknn_app_context_t *app_ctx, image_buffer_t *img, seg_object_detect_result_list *od_results)
{
    using Clock = std::chrono::steady_clock;
    Clock::time_point t_pre_begin, t_pre_end;
    Clock::time_point t_input_end, t_run_end;
    Clock::time_point t_output_end, t_post_end;
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
    };

    int ret;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[app_ctx->io_num.n_input];
    rknn_output outputs[app_ctx->io_num.n_output];
    const float nms_threshold = NMS_THRESH;
    const float box_conf_threshold = BOX_THRESH;

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
        printf("inference: 仅支持 RGB888 输入\n");
        return -1;
    }

    // Pre Process
    t_pre_begin = Clock::now();

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

    convert_image_letterbox(img->virt_addr, img->width, img->height,
                                dst_img.virt_addr, dst_img.width, dst_img.height,
                                &letter_box);
    t_pre_end = Clock::now();

    // Set Input Data
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = dst_img.virt_addr;
    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    t_input_end = Clock::now();
    
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        goto out;
    }
    // Run
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    t_run_end = Clock::now();
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
    t_output_end = Clock::now();
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // Post Process
    ret = seg_post_process(app_ctx, outputs, &letter_box, box_conf_threshold, nms_threshold, od_results);
    t_post_end = Clock::now();
    printf("Seg detail: pre=%.2f input=%.2f run=%.2f output=%.2f post=%.2f ms\n",
       ms(t_pre_begin, t_pre_end),
       ms(t_pre_end, t_input_end),
       ms(t_input_end, t_run_end),
       ms(t_run_end, t_output_end),
       ms(t_output_end, t_post_end));
    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

out:
    if (dst_img.virt_addr != NULL)
    {
        free(dst_img.virt_addr);
    }

    return ret;
}