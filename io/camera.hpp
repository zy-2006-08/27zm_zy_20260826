#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
  class CameraBase
  {
    public:
    int64_t sensorWidth = -1, sensorHeight = -1;  //相机分辨率
    std::atomic<bool> is_paused_{false};
    std::atomic<bool> capturing_{false};  // 相机正常运行
    std::chrono::steady_clock::time_point last_read_t;
    std::string camera_sn_;

    virtual ~CameraBase() = default;
    CameraBase(const std::string & sn) : camera_sn_(sn) {};
    virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
    virtual bool try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;

    virtual void pause() {}   //停止
    virtual void resume() {}  //开启

    virtual void clear_camera_frame_buffer() = 0;
  };

  class Camera
  {
    public:
    std::string main_and_secondary = "main";  //是否是主相机
    cv::Mat img_gamma_lut;
    double img_gamma = 1.0;

    Camera(const std::string & config_path);

    static void initSDK();
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
    bool try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

    // 新增对外调用的接口
    void pause()
    {
      if (camera_) camera_->pause();
    }
    void resume()
    {
      if (camera_) camera_->resume();
    }
    bool is_paused()
    {
      if (camera_)
        return camera_->is_paused_;
      else
        return false;
    }
    std::chrono::steady_clock::time_point get_last_read_t() { return camera_->last_read_t; }
    bool get_capturing() { return this->camera_->capturing_.load(); }
    std::string get_camera_sn() { return camera_->camera_sn_; }
    void clear_camera_frame_buffer() { camera_->clear_camera_frame_buffer(); };

    private:
    std::unique_ptr<CameraBase> camera_;
  };

}  // namespace io

#endif  // IO__CAMERA_HPP