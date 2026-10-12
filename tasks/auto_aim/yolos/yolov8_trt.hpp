#ifndef AUTO_AIM__YOLOV8_TRT_HPP
#define AUTO_AIM__YOLOV8_TRT_HPP

// TensorRT 版 YOLOv8 —— 只在 Jetson 上编译 (CMake 检测到 TensorRT 才定义 TENSOR_RT_MAKE)。
//
// 为什么继承 YOLOV8 而不是另写一套:
//   letterbox 前处理、parse 解码、数字分类、传统CV角点精修、画图这些全部复用父类,
//   这里只替换"网络前向"那一步 (覆盖 infer())。这样 x86(OpenVINO) 和 Jetson(TensorRT)
//   识别逻辑逐行一致, 两边调好的参数可以直接互用, 差别只剩推理后端本身的数值误差。
//
// 模型: yaml 的 yolov8_engine_path (默认 ../assets/yolov8_fp16.engine)。
//   engine 和 GPU / TensorRT 版本绑定, 不能跨机器拷贝, 每台 Jetson 首次要自己生成:
//     bash scripts/build_trt_engine.sh
//   engine 输入 images [1,3,416,416] f32 (RGB, /255), 输出 [1,14,3549] f32,
//   与 OpenVINO 那份 yolov8.xml 的张量定义完全相同。

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <vector>

#include "tasks/auto_aim/yolos/yolov8.hpp"

namespace auto_aim
{

  class YOLOV8TRT : public YOLOV8
  {
    public:
    YOLOV8TRT(const std::string & config_path, bool debug);
    ~YOLOV8TRT() override;

    YOLOV8TRT(const YOLOV8TRT &) = delete;
    YOLOV8TRT & operator=(const YOLOV8TRT &) = delete;

    protected:
    cv::Mat infer(const cv::Mat & input_416_bgr) override;

    private:
    static constexpr int kInput = 416;

    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_ = nullptr;

    std::string input_name_, output_name_;
    int out_rows_ = 0, out_cols_ = 0;  // 14 x 3549

    // 显存与页锁定内存, 构造时一次分配, 每帧复用 (每帧 malloc 会严重掉帧)
    void * d_input_ = nullptr;   // float [1,3,416,416]
    void * d_output_ = nullptr;  // float [1,14,3549]
    float * h_input_ = nullptr;  // pinned, 与 d_input_ 同尺寸
    float * h_output_ = nullptr;  // pinned, 与 d_output_ 同尺寸
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV8_TRT_HPP
