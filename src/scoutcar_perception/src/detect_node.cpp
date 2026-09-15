#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"

class DetectNode : public rclcpp::Node
{
    public:
        DetectNode() : Node("yolov5_detect_node")
        {
            const auto package_share =
                ament_index_cpp::get_package_share_directory(
                    "scoutcar_perception");
            model_path_ = declare_parameter<std::string>("model_path", package_share + "/home/orangepi/CityScout/src/scoutcar_perception/models/yolov5_detect");
            front_image_topic_ = declare_parameter<std::string>("front_image_topic", "/camera/front/image_raw");
            turn_image_topic_ = declare_parameter<std::string>("turn_image_topic", "/camera/turn/image_raw");
            

            pub_detection_ = create_publisher<sensor_msgs::msg::Image>("yolov5/detection", 10);
            sub_front_image_ = create_subscription<sensor_msgs::msg::Image>(
                front_image_topic_, rclcpp::QoS(1).best_effort(),
                [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                    process_frame(msg);
                });
            sub_turn_image_ = create_subscription<sensor_msgs::msg::Image>(
                turn_image_topic_, rclcpp::QoS(1).best_effort(),
                [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                    process_frame(msg);
                });
            
            if(init_yolov5_model(model_path_.c_str(), &rknn_app_ctx_) != 0) {
                RCLCPP_ERROR(get_logger(), "Failed to initialize yolov5 model: %s", model_path_.c_str());
                model_ok_ = false;
            } else {
                model_ok_ = true;
                RCLCPP_INFO(get_logger(), "Yolov5 model initialized successfully");
            }

        }
        ~DetectNode()
        {
            deinit_post_process();
            release_yolov5_model(&rknn_app_ctx_);
        }
    private:
        void process_frame(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            if (!model_ok_) {
                return;
            }
            // Convert sensor_msgs/Image to image_buffer_t
            image_buffer_t img{};
            img.width = static_cast<int>(msg->width);
            img.height = static_cast<int>(msg->height);
            img.format = IMAGE_FORMAT_RGB888;
            img.virt_addr = const_cast<unsigned char*>(msg->data.data());

            object_detect_result_list od_results{};
            if (inference_yolov5_model(&rknn_app_ctx_, &img, &od_results) != 0) {
                RCLCPP_ERROR(get_logger(), "Inference failed");
                return;
            }
             
            char text[256];
            for (int i = 0; i < od_results.count; i++)
            {
                object_detect_result *det_result = &(od_results.results[i]);
                printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                    det_result->box.left, det_result->box.top,
                    det_result->box.right, det_result->box.bottom,
                    det_result->prop);
                int x1 = det_result->box.left;
                int y1 = det_result->box.top;
                int x2 = det_result->box.right;
                int y2 = det_result->box.bottom;

                draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);

                sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
                draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
            }
        }

        std::string model_path_;
        std::string front_image_topic_;
        std::string turn_image_topic_;
        bool model_ok_ = false;
        rknn_app_context_t rknn_app_ctx_{};
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_detection_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_front_image_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_turn_image_;





}
int main(int argc, char **argv)
{
   
    int ret;
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    init_post_process();

    

    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    ret = read_image(image_path, &src_image);



    if (ret != 0)
    {
        printf("read image fail! ret=%d image_path=%s\n", ret, image_path);
        goto out;
    }

    object_detect_result_list od_results;

    ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
    if (ret != 0)
    {
        printf("init_yolov5_model fail! ret=%d\n", ret);
        goto out;
    }

    // 画框和概率
    char text[256];
    for (int i = 0; i < od_results.count; i++)
    {
        object_detect_result *det_result = &(od_results.results[i]);
        printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
               det_result->box.left, det_result->box.top,
               det_result->box.right, det_result->box.bottom,
               det_result->prop);
        int x1 = det_result->box.left;
        int y1 = det_result->box.top;
        int x2 = det_result->box.right;
        int y2 = det_result->box.bottom;

        draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);

        sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
        draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
    }

    write_image("out.png", &src_image);

out:
    deinit_post_process();

    ret = release_yolov5_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov5_model fail! ret=%d\n", ret);
    }

    if (src_image.virt_addr != NULL)
    {
#if defined(RV1106_1103) 
        dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
#else
        free(src_image.virt_addr);
#endif
    }

    return 0;
}
