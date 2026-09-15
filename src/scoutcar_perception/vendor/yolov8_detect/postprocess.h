#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 20
#define NMS_THRESH 0.45
#define BOX_THRESH 0.25

// class detect_rknn_app_context_t;

typedef struct {
    image_rect_t box;
    float prop;
    int cls_id;
} detect_object_detect_result;

typedef struct {
    int id;
    int count;
    detect_object_detect_result results[OBJ_NUMB_MAX_SIZE];
} detect_object_detect_result_list;

int detect_init_post_process(const char *label_path);
void detect_deinit_post_process();
char *detect_cls_to_name(int cls_id);
int detect_post_process(detect_rknn_app_context_t *app_ctx, void *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, detect_object_detect_result_list *od_results);

void detect_deinitPostProcess();
#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
