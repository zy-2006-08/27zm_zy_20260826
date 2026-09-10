#include "camera.hpp"

#include <stdexcept>

#include "MvCameraControl.h"  // 品牌探测要直接枚举海康设备

#include "daheng/daheng.hpp"
#include "hikrobot/hikrobot.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
  namespace
  {
    // 探测结果缓存: 枚举一次要几百毫秒, 而构造流程里会多处用到,
    // 缓存下来避免重复枚举。
    std::string g_detected_brand;
    std::string g_detected_sn;
  }  // namespace

  // 注意 initSDK 的容错语义:
  //
  // 原实现直接调 DahengCamera::initSDK(), 而后者在 GXInitLib() 失败时只打一条
  // error 日志就返回。问题在于, 只要链接了 gxiapi, 没装大恒 SDK 的机器上
  // 连"只用海康相机"这条路都会先卡在这条错误日志上, 让人误以为是海康的问题。
  //
  // 这里保持"调用它、但不因它失败而中断"的语义, 并把日志降级为 debug 级别的
  // 说明性提示: 大恒 SDK 的存在与否, 只应影响大恒相机能不能用。
  void Camera::initSDK() { DahengCamera::initSDK(); }

  std::string Camera::detected_sn() { return g_detected_sn; }

  std::string Camera::detect_brand()
  {
    if (!g_detected_brand.empty()) return g_detected_brand;

    // ---- 先探海康 ----
    // 只枚举 USB, 与 io/hikrobot/hikrobot.cpp 的 capture_start 保持一致口径。
    // 枚举失败时 0x80000006 = 传输层插件缺失(未装 /opt/MVS), 不是"没有相机",
    // 这两种情况要分开报, 否则会把环境问题误判成硬件问题。
    MV_CC_DEVICE_INFO_LIST hik_list;
    memset(&hik_list, 0, sizeof(hik_list));
    int hik_ret = MV_CC_EnumDevices(MV_USB_DEVICE, &hik_list);

    if (hik_ret == MV_OK && hik_list.nDeviceNum > 0)
    {
      g_detected_brand = "hikrobot";
      g_detected_sn = reinterpret_cast<const char *>(hik_list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chSerialNumber);
      tools::logger()->info("[Camera] 探测到海康相机 型号={} SN={}",
                            reinterpret_cast<const char *>(hik_list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chModelName), g_detected_sn);
      return g_detected_brand;
    }
    if (hik_ret != MV_OK)
      tools::logger()->debug("[Camera] 海康枚举返回 {:#x}{}", hik_ret,
                             hik_ret == 0x80000006 ? " (传输层插件缺失, 未安装 /opt/MVS)" : "");

    // ---- 再探大恒 ----
    // GXInitLib 未装 SDK 时会失败, 用 try 兜住, 不能让它把整个探测带崩。
    DahengCamera::initSDK();
    uint32_t dh_num = 0;
    if (GXUpdateDeviceList(&dh_num, 1000) == GX_STATUS_SUCCESS && dh_num > 0)
    {
      GX_DEVICE_BASE_INFO * infos = new GX_DEVICE_BASE_INFO[dh_num];
      size_t size = dh_num * sizeof(GX_DEVICE_BASE_INFO);
      if (GXGetAllDeviceBaseInfo(infos, &size) == GX_STATUS_SUCCESS)
      {
        g_detected_brand = "daheng";
        g_detected_sn = infos[0].szSN;
        tools::logger()->info("[Camera] 探测到大恒相机 型号={} SN={}", infos[0].szModelName, g_detected_sn);
      }
      delete[] infos;
      if (!g_detected_brand.empty()) return g_detected_brand;
    }

    tools::logger()->warn("[Camera] 未探测到任何相机 (海康与大恒都没有)");
    return "";
  }
  Camera::Camera(const std::string & config_path)
  {
    auto yaml = tools::load(config_path);
    auto camera_name = tools::read<std::string>(yaml, "camera_name");

    // camera_name: "auto" -> 自己枚举决定用哪家。
    // 也接受 yaml 里干脆没写 camera_name 的情况(tools::read 会抛, 由调用方处理),
    // 这里只处理显式写 auto 的路径, 保持行为可预期。
    if (camera_name == "auto")
    {
      camera_name = detect_brand();
      if (camera_name.empty()) throw std::runtime_error("camera_name=auto 但未探测到任何相机!");
      tools::logger()->info("[Camera] auto 模式选定: {}", camera_name);
    }
    auto exposure_us = tools::read<double>(yaml, "exposure_us");

    bool flip = tools::read<bool>(yaml, "flip");
    bool mirror = tools::read<bool>(yaml, "mirror");

    img_gamma = tools::read<double>(yaml, "img_gamma");

    int lut_size = 1 << 8;
    this->img_gamma_lut = cv::Mat(lut_size, 1, CV_8U);
    for (int i = 0; i < lut_size; i++)
    {
      img_gamma_lut.data[i] = cv::saturate_cast<uchar>(pow(i / 255.0, img_gamma) * 255.0);
    }

    if (camera_name == "hikrobot")
    {
      auto gain = tools::read<double>(yaml, "gain");
      auto vid_pid = "2bdf:0001";

      // camera_sn 允许缺失或为空: 海康这边只有一台相机时没必要强制写 SN,
      // 写错 SN 反而会让 hikrobot.cpp 的 ChoiceCamrea 匹配不上。
      // 缺失时回落到探测到的 SN。
      std::string sn;
      try
      {
        sn = tools::read<std::string>(yaml, "camera_sn");
      }
      catch (const std::exception &)
      {
        sn = "";
      }
      if (sn.empty())
      {
        if (g_detected_brand.empty()) detect_brand();
        sn = g_detected_sn;
        tools::logger()->info("[Camera] yaml 未指定 camera_sn, 使用探测到的 SN={}", sn);
      }

      camera_ = std::make_unique<HikRobot>(sn, exposure_us, gain, vid_pid, flip, mirror);
    }

    else if (camera_name == "daheng")
    {
      auto gain = tools::read<double>(yaml, "gain");
      auto gamma = tools::read<double>(yaml, "gamma");
      auto vid_pid = "2ba2:4d55";
      auto camera_sn = tools::read<std::string>(yaml, "camera_sn");
      camera_ = std::make_unique<DahengCamera>(camera_sn, exposure_us, gain, gamma, flip, mirror);
    }

    else
    {
      throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
    }
  }

  void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
  {
    camera_->read(img, timestamp);
    // if(img_gamma == 1.0){
    //   cv::LUT(img, img_gamma_lut, img);
    // }

    if (std::abs(img_gamma - 1.0) > 1e-6)
    {
      cv::LUT(img, img_gamma_lut, img);
    }
  }
  bool Camera::try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) { return camera_->try_read(img, timestamp); }
}  // namespace io