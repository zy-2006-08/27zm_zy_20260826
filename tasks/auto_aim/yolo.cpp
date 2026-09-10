#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#ifdef TENSOR_RT_MAKE
#include "trt_yolos/trt_yolo_0708.hpp"
#endif

#ifdef OPENVINO_MAKE
#include "yolos/yolo11.hpp"
#include "yolos/yolo7.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"
#endif

namespace auto_aim
{
  YOLO::YOLO(const std::string & config_path, bool debug)
  {
    auto yaml = YAML::LoadFile(config_path);
    auto yolo_name = yaml["yolo_name"].as<std::string>();

#ifdef OPENVINO_MAKE
    if (yolo_name == "yolov8")
    {
      yolo_ = std::make_unique<YOLOV8>(config_path, debug);
    }

    else if (yolo_name == "yolo11")
    {
      yolo_ = std::make_unique<YOLO11>(config_path, debug);
    }

    else if (yolo_name == "yolov5")
    {
      yolo_ = std::make_unique<YOLOV5>(config_path, debug);
    }

    // yolo7 不是 YOLOv7 网络，加载的仍是 assets/yolov8.xml。
    // 它是 YOLOV8 的副本，只把 sort_keypoints 换成极角排序，
    // 用来验证/修正「装甲板 roll 超过 22° 就识别不到」。详见 yolos/yolo7.hpp。
    //
    // 同时接受 "yolov7" 这个写法：旁边三个后端都叫 yolov5 / yolov8 / yolo11，
    // 顺手写成 yolov7 很自然（而且 YOLOv7 确实是个真实存在的网络，更容易混）。
    // 两种写法都指向同一个 YOLO7，避免因为一个字母排查半天。
    else if (yolo_name == "yolo7" || yolo_name == "yolov7")
    {
      yolo_ = std::make_unique<YOLO7>(config_path, debug);
    }

    else
    {
      throw std::runtime_error(
        "Unknown yolo name: " + yolo_name + "! 可用值: yolov5 / yolov8 / yolo11 / yolo7(=yolov7, 修正关键点排序的 v8)");
    }

#endif

#ifdef TENSOR_RT_MAKE
    if (yolo_name == "trt_0708")
    {
      yolo_ = std::make_unique<TensorRTYolo>(config_path, debug);
    }

    else
    {
      throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
    }
#endif
  }

  std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count) { return yolo_->detect(img, frame_count); }

  std::list<Armor> YOLO::postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
  {
    return yolo_->postprocess(scale, output, bgr_img, frame_count);
  }

}  // namespace auto_aim