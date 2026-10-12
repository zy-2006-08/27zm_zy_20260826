#include "yolov8_trt.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>

#include "tools/logger.hpp"

namespace auto_aim
{
  namespace
  {
    class TrtLogger : public nvinfer1::ILogger
    {
      void log(Severity severity, const char * msg) noexcept override
      {
        if (severity <= Severity::kWARNING) tools::logger()->warn("[TensorRT] {}", msg);
      }
    };
    TrtLogger g_trt_logger;

    void cuda_check(cudaError_t e, const char * what)
    {
      if (e != cudaSuccess) throw std::runtime_error(std::string("[YOLOV8TRT] ") + what + ": " + cudaGetErrorString(e));
    }

    size_t volume(const nvinfer1::Dims & d)
    {
      size_t v = 1;
      for (int i = 0; i < d.nbDims; i++) v *= static_cast<size_t>(d.d[i]);
      return v;
    }
  }  // namespace

  YOLOV8TRT::YOLOV8TRT(const std::string & config_path, bool debug) : YOLOV8(config_path, debug, false)
  {
    auto yaml = YAML::LoadFile(config_path);
    const std::string engine_path =
      yaml["yolov8_engine_path"] ? yaml["yolov8_engine_path"].as<std::string>() : "../assets/yolov8_fp16.engine";

    std::ifstream f(engine_path, std::ios::binary);
    if (!f)
      throw std::runtime_error(
        "[YOLOV8TRT] 找不到 TensorRT engine: " + engine_path +
        "\n  engine 与本机 GPU/TensorRT 版本绑定, 每台 Jetson 第一次要生成: bash scripts/build_trt_engine.sh");
    std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    runtime_.reset(nvinfer1::createInferRuntime(g_trt_logger));
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_)
      throw std::runtime_error("[YOLOV8TRT] engine 反序列化失败 (多半是在别的机器/别的 TensorRT 版本生成的), 重新生成: bash scripts/build_trt_engine.sh");
    context_.reset(engine_->createExecutionContext());

    for (int i = 0; i < engine_->getNbIOTensors(); i++)
    {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT)
        input_name_ = name;
      else
        output_name_ = name;
    }
    const auto in_dims = engine_->getTensorShape(input_name_.c_str());
    const auto out_dims = engine_->getTensorShape(output_name_.c_str());
    if (volume(in_dims) != 3ull * kInput * kInput || out_dims.nbDims != 3)
      throw std::runtime_error("[YOLOV8TRT] engine 张量形状不对, 期望输入 [1,3,416,416] 输出 [1,14,N]");
    out_rows_ = static_cast<int>(out_dims.d[1]);
    out_cols_ = static_cast<int>(out_dims.d[2]);

    const size_t in_bytes = volume(in_dims) * sizeof(float);
    const size_t out_bytes = volume(out_dims) * sizeof(float);
    cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate");
    cuda_check(cudaMalloc(&d_input_, in_bytes), "cudaMalloc input");
    cuda_check(cudaMalloc(&d_output_, out_bytes), "cudaMalloc output");
    cuda_check(cudaMallocHost(reinterpret_cast<void **>(&h_input_), in_bytes), "cudaMallocHost input");
    cuda_check(cudaMallocHost(reinterpret_cast<void **>(&h_output_), out_bytes), "cudaMallocHost output");
    context_->setTensorAddress(input_name_.c_str(), d_input_);
    context_->setTensorAddress(output_name_.c_str(), d_output_);

    tools::logger()->info("[YOLOV8TRT] 已加载 {} (输出 {}x{})", engine_path, out_rows_, out_cols_);
  }

  YOLOV8TRT::~YOLOV8TRT()
  {
    if (stream_) cudaStreamSynchronize(stream_);
    context_.reset();
    engine_.reset();
    runtime_.reset();
    if (d_input_) cudaFree(d_input_);
    if (d_output_) cudaFree(d_output_);
    if (h_input_) cudaFreeHost(h_input_);
    if (h_output_) cudaFreeHost(h_output_);
    if (stream_) cudaStreamDestroy(stream_);
  }

  cv::Mat YOLOV8TRT::infer(const cv::Mat & input_416_bgr)
  {
    // 与 OpenVINO 侧 PrePostProcessor 等价: BGR u8 HWC -> RGB f32 /255 CHW
    // 直接写进页锁定内存, 省一次拷贝。
    const int area = kInput * kInput;
    float * r = h_input_;
    float * g = h_input_ + area;
    float * b = h_input_ + 2 * area;
    for (int y = 0; y < kInput; y++)
    {
      const uint8_t * p = input_416_bgr.ptr<uint8_t>(y);
      const int row = y * kInput;
      for (int x = 0; x < kInput; x++, p += 3)
      {
        b[row + x] = p[0] * (1.0f / 255.0f);
        g[row + x] = p[1] * (1.0f / 255.0f);
        r[row + x] = p[2] * (1.0f / 255.0f);
      }
    }

    const size_t in_bytes = 3ull * area * sizeof(float);
    const size_t out_bytes = static_cast<size_t>(out_rows_) * out_cols_ * sizeof(float);
    cuda_check(cudaMemcpyAsync(d_input_, h_input_, in_bytes, cudaMemcpyHostToDevice, stream_), "H2D");
    if (!context_->enqueueV3(stream_)) throw std::runtime_error("[YOLOV8TRT] enqueueV3 失败");
    cuda_check(cudaMemcpyAsync(h_output_, d_output_, out_bytes, cudaMemcpyDeviceToHost, stream_), "D2H");
    cuda_check(cudaStreamSynchronize(stream_), "sync");

    // clone: h_output_ 下一帧会被覆盖, 而 parse() 会就地 transpose
    return cv::Mat(out_rows_, out_cols_, CV_32F, h_output_).clone();
  }

}  // namespace auto_aim
