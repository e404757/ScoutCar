#ifndef _RKNN_YOLOV5_SEG_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV5_SEG_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#define OBJ_NAME_MAX_SIZE 64    // 类别名称最大长度
#define OBJ_NUMB_MAX_SIZE 128   // 最多检测 128 个目标
#define OBJ_CLASS_NUM 2        // 2 类: road(0), barrier(1)
#define NMS_THRESH 0.45         // NMS 阈值（重叠度 > 0.45 的框会被抑制）
#define BOX_THRESH 0.6          // 置信度阈值（新模型置信度高；阈值越高 → 框越少 → 掩膜计算越快）
#define PROP_BOX_SIZE (5 + OBJ_CLASS_NUM)  // 每个候选框原始数据长度 = 5 + 类别数

#define PROTO_CHANNEL 32
#define PROTO_HEIGHT 160
#define PROTO_WEIGHT 160

// 训练预处理: 裁剪顶部 CROP_TOP_PIXELS 像素后拉伸到 640x640 (推理必须一致)
#define CROP_TOP_PIXELS 120



typedef struct
{
    image_rect_t box;//检测框
    float prop;      //置信度
    int cls_id;      //类别id
} object_detect_result;

typedef struct
{
    uint8_t *seg_mask;
} object_segment_result;

typedef struct
{
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
    object_segment_result results_seg[OBJ_NUMB_MAX_SIZE];
    // 未过滤(阈值前)的每类最高分(score = box_conf * class_prob)，用于标定 BOX_THRESH。
    // 无论是否通过 BOX_THRESH 都更新，因此能画出真实的置信度分布。
    float raw_class_score[OBJ_CLASS_NUM];
} object_detect_result_list;

int init_post_process(const char *label_path);
void deinit_post_process();
char *coco_cls_to_name(int cls_id);
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);
void deinitPostProcess();
int clamp(float val, int min, int max);

#endif //_RKNN_YOLOV5_SEG_DEMO_POSTPROCESS_H_
