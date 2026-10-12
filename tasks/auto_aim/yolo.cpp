#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/logger.hpp"

#ifdef OPENVINO_MAKE
#include "yolos/yolo11.hpp"
#include "yolos/yolo7.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"
#endif

// TENSOR_RT_MAKE 由 tasks/auto_aim/CMakeLists.txt 在检测到 TensorRT 时自动定义
// (Jetson 有, x86 小电脑没有)。同一份代码两边都能编。
#ifdef TENSOR_RT_MAKE
#include "yolos/yolov8_trt.hpp"
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
      // 推理后端选择 (yaml 的 yolov8_backend, 可省略):
      //   auto     (默认) 编了 TensorRT 且 engine 文件存在 -> TensorRT, 否则 OpenVINO
      //   tensorrt 强制 TensorRT, engine 不在就报错
      //   openvino 强制 OpenVINO (Jetson 上对比/排查用)
      // 所以同一份 sb_long.yaml: x86 小电脑走 OpenVINO, Jetson 走 TensorRT, 不用改配置。
      const std::string backend = yaml["yolov8_backend"] ? yaml["yolov8_backend"].as<std::string>() : "auto";
#ifdef TENSOR_RT_MAKE
      const std::string engine =
        yaml["yolov8_engine_path"] ? yaml["yolov8_engine_path"].as<std::string>() : "../assets/yolov8_fp16.engine";
      const bool use_trt = backend == "tensorrt" || (backend == "auto" && std::filesystem::exists(engine));
      if (backend == "auto" && !use_trt)
        tools::logger()->warn("[YOLO] 本机支持 TensorRT 但没有 {}, 回退 OpenVINO(慢)。生成: bash scripts/build_trt_engine.sh", engine);
      if (use_trt)
        yolo_ = std::make_unique<YOLOV8TRT>(config_path, debug);
      else
        yolo_ = std::make_unique<YOLOV8>(config_path, debug);
#else
      const bool use_trt = false;
      if (backend == "tensorrt") throw std::runtime_error("yolov8_backend: tensorrt, 但本机没装 TensorRT (只有 Jetson 支持)");
      yolo_ = std::make_unique<YOLOV8>(config_path, debug);
#endif
      tools::logger()->info("[YOLO] yolov8 推理后端: {}", use_trt ? "TensorRT (GPU)" : "OpenVINO (CPU)");
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
  }

  std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count) { return yolo_->detect(img, frame_count); }

  std::list<Armor> YOLO::postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
  {
    return yolo_->postprocess(scale, output, bgr_img, frame_count);
  }

}  // namespace auto_aim