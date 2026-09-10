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

    // 相机品牌自动探测。
    //
    // 动机: 队里同时有大恒和海康两种相机, 换相机时要手工改 yaml 的 camera_name,
    // 忘了改的表现是"程序起不来"或"找不到相机", 排查起来很费时间。
    // 让程序自己枚举一遍就能消掉这类问题。
    //
    // 返回 "hikrobot" / "daheng" / "" (都没找到)。
    // 两家都插着时优先返回 hikrobot —— 仅出于给一个确定结果, 无偏好含义;
    // 这种情况下应当在 yaml 里显式写明品牌。
    static std::string detect_brand();

    // 探测到的相机 SN。detect_brand() 成功后才有效。
    // 用途: yaml 里 camera_sn 写错或为空时可以回落到实际枚举到的 SN。
    static std::string detected_sn();

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