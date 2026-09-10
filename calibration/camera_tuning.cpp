// ============================================================================
//  相机调参助手 / Camera tuning helper
// ----------------------------------------------------------------------------
//  用途：接上相机后实时看画面，同时把「对焦清晰度」和「亮度」量化成数字显示在
//        画面上，边拧镜头边看数字，不用靠肉眼猜。曝光/增益可用键盘实时改，
//        改完按 p 存到 configs/tuning_result.yaml，再手动抄进 calibration.yaml。
//
//  ★ 本程序支持大恒与海康两种相机，启动时自动探测，不需要改配置。
//    原来只支持大恒（直调 GX* API），插海康相机时报「没有找到相机」——
//    因为大恒 SDK 枚举不到海康设备，这跟相机好坏无关。
//    现在把相机操作抽成 TuningCamera 接口，两家各实现一份，启动时先枚举
//    海康、再枚举大恒，找到哪个用哪个。
//
//  为什么不直接读写 configs/calibration.yaml：
//    那个文件里有 5 段相机配置（4 段注释掉的备用参数）、标定矩阵、手眼变换，
//    还夹着大量中文注释。让程序自动改这种文件容易改错位置且不易发现，
//    所以只写一个独立的结果文件，最后一步由人工确认后填入。
//
//  为什么不用工程的 io::Camera：
//    io::Camera 只在构造时设一次曝光（见 io/daheng/daheng.cpp 的
//    initialize_camera 与 io/hikrobot/hikrobot.cpp 的 capture_start），
//    运行期改不了。调参需要「改完立刻看到效果」，所以这里直接调厂商 SDK。
//
//  ★ 画面方向：本程序会读 calibration.yaml 的 flip/mirror，和上车程序保持一致。
//    按 f 键可以现场切换（转 180 度），用来确定相机到底是正装还是倒装。
//    定下来之后按 p 存盘，flip/mirror 会一起写进 tuning_result.yaml。
//
//  ⚠️ 相机是独占访问：本程序运行时 detect_test / test_simple /
//     rb_auto_aim_debug / hik_grab 都打不开相机，反之亦然。调完记得按 q 退出。
//     大恒占用报 -0x3ec(-1004)，海康报 0x80000203。
//
//  用法：
//    cd build && ./camera_tuning
//    窗口弹在小电脑接的显示器上（SSH 里跑需要先 export DISPLAY=:0）
// ============================================================================

#include <fmt/core.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "GxIAPI.h"
#include "MvCameraControl.h"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace
{
  // ---- 对焦判定 ----
  // 清晰度是相对量纲，跟镜头、分辨率、拍摄内容都有关。
  // 数值来自 MER2-160-227U3C + 1440x1080 实测：严重虚焦约 6，
  // 对好焦（拍有纹理的物体）能到 180 以上。
  constexpr double SHARP_GOOD = 100.0;
  constexpr double SHARP_FAIR = 40.0;

  // ---- 曝光判定 ----
  // ⚠️ 自瞄的曝光标准和普通摄影相反：要的不是「画面好看」，而是
  //    「灯条亮、环境全暗」。原因见 tasks/auto_aim/detector.cpp 第 57 行 ——
  //    检测第一步就是 cv::threshold(gray, binary, threshold_, ...)，
  //    把灰度 > threshold 的像素切成白色，再 findContours 按长宽比筛灯条。
  //
  //    装甲板灯条是自发光的，压低曝光后它依然很亮，而墙面、桌子、人、灯管
  //    会被压到阈值以下，二值化后自动消失。画面整体偏暗（亮度 40-70）是
  //    目标状态，不是缺陷。反之调到「好看」的亮度 100+，环境里所有亮物
  //    都会和灯条一起变白，检测器被假灯条淹没。
  //
  //    另外压曝光还有两个收益：
  //      · 减少运动模糊 —— 4000us 云台转动时灯条边缘仍清晰，
  //        33000us 会拖成糊线，长宽比失真被 check_geometry 筛掉
  //      · 提高帧率上限 —— 4000us 可跑 227fps(相机上限)，33000us 只有 30fps
  //
  //    参考队里实际上车用的值：configs/demo.yaml 与 sb_long.yaml 都是
  //    exposure_us: 4000, gain: 0.4~0.5。
  constexpr double BRIGHT_LO = 30.0;  // 太暗则连灯条也提不出来
  constexpr double BRIGHT_HI = 80.0;  // 太亮则环境干扰变多

  // detector 的二值化阈值，与 configs/demo.yaml 的 threshold 保持一致。
  // 「超阈值像素占比」是本工具最有用的指标：它直接等于二值化后的白色面积。
  // 没有装甲板时这个值应该接近 0（环境不该有东西过阈值）；
  // 有装甲板时应该只有灯条那一小片过阈值，通常 < 2%。
  constexpr int DETECT_THRESHOLD = 150;
  constexpr double ABOVE_TH_LIMIT = 2.0;  // 超阈值占比上限(%)

  const std::string CONFIG_PATH = "../configs/calibration.yaml";
  const std::string RESULT_PATH = "../configs/tuning_result.yaml";

  /// 对焦清晰度：拉普拉斯响应的方差。图像越清晰，边缘越锐利，二阶导响应越强。
  /// 这是对焦评价的经典指标，比人眼判断稳定得多。
  double sharpness(const cv::Mat & gray)
  {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, sd;
    cv::meanStdDev(lap, mean, sd);
    return sd[0] * sd[0];
  }

  // ==========================================================================
  //  相机抽象接口
  //
  //  只暴露调参真正需要的四件事：拿一帧 BGR 图、改曝光、改增益、报量程。
  //  故意不做成通用相机层（工程里已经有 io::CameraBase 了），
  //  这里的关键差异是「运行期可改参数」，而 io::CameraBase 没有这个能力。
  // ==========================================================================
  class TuningCamera
  {
    public:
    virtual ~TuningCamera() = default;

    /// 取一帧并转成 BGR。取不到返回 false（超时属正常，调用方 continue 即可）。
    virtual bool read(cv::Mat & bgr) = 0;

    /// 设曝光，单位微秒。传入值会被夹到相机量程内。
    virtual void set_exposure(double us) = 0;

    /// 设增益，传入 0~1 归一化值（与工程 yaml 的 gain 口径一致）。
    virtual void set_gain(double norm) = 0;

    virtual double expo_min() const = 0;
    virtual double expo_max() const = 0;

    /// 增益的原始物理值（大恒是 dB，海康是 SDK 的 Gain 值），仅用于存盘时记录。
    virtual double gain_raw(double norm) const = 0;
    virtual double gain_raw_min() const = 0;
    virtual double gain_raw_max() const = 0;

    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual std::string brand() const = 0;
    virtual std::string model() const = 0;
  };

  // ==========================================================================
  //  大恒实现（保留原有逻辑，仅从 main 里搬进来）
  // ==========================================================================
  class DahengTuning : public TuningCamera
  {
    public:
    /// 打开成功返回实例，失败返回 nullptr（不抛异常，方便逐个品牌试）。
    static std::unique_ptr<DahengTuning> open()
    {
      if (GXInitLib() != GX_STATUS_SUCCESS)
      {
        tools::logger()->debug("[Tuning] 大恒 SDK 初始化失败（未装 Galaxy SDK 时属正常）");
        return nullptr;
      }
      uint32_t dev_num = 0;
      GXUpdateDeviceList(&dev_num, 1000);
      if (dev_num == 0)
      {
        tools::logger()->debug("[Tuning] 大恒未枚举到相机");
        return nullptr;
      }

      GX_DEV_HANDLE h = nullptr;
      GX_STATUS st = GXOpenDeviceByIndex(1, &h);
      if (st != GX_STATUS_SUCCESS)
      {
        // -0x3ec(-1004) = 设备被占用。最常见原因是 detect_test 或自瞄程序还开着。
        tools::logger()->warn("[Tuning] 大恒打开失败 status={:#x}（可能被其他程序占用）", st);
        return nullptr;
      }
      return std::unique_ptr<DahengTuning>(new DahengTuning(h));
    }

    ~DahengTuning() override
    {
      GXStreamOff(h_);
      GXCloseDevice(h_);
      GXCloseLib();
    }

    bool read(cv::Mat & bgr) override
    {
      if (GXGetImage(h_, &frame_, 200) != GX_STATUS_SUCCESS) return false;
      if (frame_.nStatus != 0) return false;
      cv::Mat bayer(h_px_, w_px_, CV_8UC1, frame_.pImgBuf);
      cv::cvtColor(bayer, bgr, bayer_code_);
      return true;
    }

    void set_exposure(double us) override { GXSetFloat(h_, GX_FLOAT_EXPOSURE_TIME, us); }
    void set_gain(double norm) override { GXSetFloat(h_, GX_FLOAT_GAIN, gain_raw(norm)); }

    double expo_min() const override { return expo_range_.dMin; }
    double expo_max() const override { return expo_range_.dMax; }

    /// 大恒的增益是 dB 值（本机量程 0-24），而工程 yaml 里的 gain 是 0-1 归一化值，
    /// 由 io/daheng/daheng.cpp 换算成 dB。这里保持同样的换算口径，
    /// 保证 p 键存出来的数字可以直接抄进 yaml。
    double gain_raw(double norm) const override { return gain_range_.dMin + norm * (gain_range_.dMax - gain_range_.dMin); }
    double gain_raw_min() const override { return gain_range_.dMin; }
    double gain_raw_max() const override { return gain_range_.dMax; }

    int width() const override { return w_px_; }
    int height() const override { return h_px_; }
    std::string brand() const override { return "daheng"; }
    std::string model() const override { return model_; }

    private:
    explicit DahengTuning(GX_DEV_HANDLE h) : h_(h)
    {
      int64_t w = 0, hh = 0, cf = 0;
      GXGetInt(h_, GX_INT_WIDTH, &w);
      GXGetInt(h_, GX_INT_HEIGHT, &hh);
      GXGetEnum(h_, GX_ENUM_PIXEL_COLOR_FILTER, &cf);
      w_px_ = static_cast<int>(w);
      h_px_ = static_cast<int>(hh);

      GXGetFloatRange(h_, GX_FLOAT_EXPOSURE_TIME, &expo_range_);
      GXSetEnum(h_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
      GXGetFloatRange(h_, GX_FLOAT_GAIN, &gain_range_);

      GXSetEnum(h_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
      GXSetEnum(h_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

      // Bayer 转彩色。工程里用大恒的 DxRaw8toRGB24，但那个库(libdximageproc)
      // 小电脑上没装，调参只是看画面，用 OpenCV 的 demosaic 完全够。
      switch (cf)
      {
        case GX_COLOR_FILTER_BAYER_RG: bayer_code_ = cv::COLOR_BayerRG2BGR; break;
        case GX_COLOR_FILTER_BAYER_GB: bayer_code_ = cv::COLOR_BayerGB2BGR; break;
        case GX_COLOR_FILTER_BAYER_BG: bayer_code_ = cv::COLOR_BayerBG2BGR; break;
        default: bayer_code_ = cv::COLOR_BayerGR2BGR; break;
      }

      char name[128] = {0};
      size_t len = sizeof(name);
      if (GXGetString(h_, GX_STRING_DEVICE_MODEL_NAME, name, &len) == GX_STATUS_SUCCESS) model_ = name;

      raw_.resize(static_cast<size_t>(w_px_) * h_px_);
      frame_.pImgBuf = raw_.data();
      GXStreamOn(h_);
    }

    GX_DEV_HANDLE h_ = nullptr;
    int w_px_ = 0, h_px_ = 0;
    GX_FLOAT_RANGE expo_range_{}, gain_range_{};
    cv::ColorConversionCodes bayer_code_ = cv::COLOR_BayerGR2BGR;
    std::vector<uint8_t> raw_;
    GX_FRAME_DATA frame_{};
    std::string model_ = "unknown";
  };

  // ==========================================================================
  //  海康实现
  // ==========================================================================
  class HikTuning : public TuningCamera
  {
    public:
    static std::unique_ptr<HikTuning> open()
    {
      MV_CC_DEVICE_INFO_LIST list;
      memset(&list, 0, sizeof(list));
      int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
      if (ret != MV_OK)
      {
        // 0x80000006 = 资源申请失败，实际含义是传输层插件缺失（未装 /opt/MVS）。
        // 这跟「没插相机」是两回事，必须分开报，否则会把环境问题当硬件问题查。
        tools::logger()->debug("[Tuning] 海康枚举返回 {:#x}{}", ret, ret == 0x80000006 ? "（未安装 /opt/MVS）" : "");
        return nullptr;
      }
      if (list.nDeviceNum == 0)
      {
        tools::logger()->debug("[Tuning] 海康未枚举到相机");
        return nullptr;
      }

      void * h = nullptr;
      if (MV_CC_CreateHandle(&h, list.pDeviceInfo[0]) != MV_OK) return nullptr;

      ret = MV_CC_OpenDevice(h, MV_ACCESS_Exclusive, 0);
      if (ret != MV_OK)
      {
        // 0x80000203 = 被占用，对应大恒的 -0x3ec
        tools::logger()->warn("[Tuning] 海康打开失败 {:#x}{}", ret, ret == 0x80000203 ? "（被其他程序占用）" : "");
        MV_CC_DestroyHandle(h);
        return nullptr;
      }

      const char * model = reinterpret_cast<const char *>(list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chModelName);
      return std::unique_ptr<HikTuning>(new HikTuning(h, model));
    }

    ~HikTuning() override
    {
      MV_CC_StopGrabbing(h_);
      MV_CC_CloseDevice(h_);
      MV_CC_DestroyHandle(h_);
    }

    bool read(cv::Mat & bgr) override
    {
      MV_FRAME_OUT frame;
      memset(&frame, 0, sizeof(frame));
      if (MV_CC_GetImageBuffer(h_, &frame, 200) != MV_OK) return false;

      const unsigned int w = frame.stFrameInfo.nWidth;
      const unsigned int hh = frame.stFrameInfo.nHeight;
      bgr.create(hh, w, CV_8UC3);

      // 用 SDK 的 ConvertPixelType 而不是自己写 cv::cvtColor 的 Bayer 分支：
      // 排列选错了颜色会整体翻转（红蓝互换），而 SDK 按相机上报的
      // enPixelType 处理，不会错。本机实测是 BayerRG8。
      MV_CC_PIXEL_CONVERT_PARAM cvt;
      memset(&cvt, 0, sizeof(cvt));
      cvt.nWidth = w;
      cvt.nHeight = hh;
      cvt.pSrcData = frame.pBufAddr;
      cvt.nSrcDataLen = frame.stFrameInfo.nFrameLen;
      cvt.enSrcPixelType = frame.stFrameInfo.enPixelType;
      cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      cvt.pDstBuffer = bgr.data;
      cvt.nDstBufferSize = w * hh * 3;
      const int cret = MV_CC_ConvertPixelType(h_, &cvt);

      MV_CC_FreeImageBuffer(h_, &frame);
      return cret == MV_OK;
    }

    void set_exposure(double us) override { MV_CC_SetFloatValue(h_, "ExposureTime", static_cast<float>(us)); }

    /// 海康的增益换算口径与 io/hikrobot/hikrobot.cpp 保持一致：
    /// 那里是 set_float_value("Gain", gain_ * AutoGainUpperLimit)，
    /// 也就是把 yaml 的 0~1 乘以增益上限。这里沿用，保证 p 键存出的
    /// 数字抄进 yaml 后亮度一致。
    void set_gain(double norm) override { MV_CC_SetFloatValue(h_, "Gain", static_cast<float>(gain_raw(norm))); }

    double expo_min() const override { return expo_min_; }
    double expo_max() const override { return expo_max_; }

    double gain_raw(double norm) const override { return norm * gain_upper_; }
    double gain_raw_min() const override { return 0.0; }
    double gain_raw_max() const override { return gain_upper_; }

    int width() const override { return w_px_; }
    int height() const override { return h_px_; }
    std::string brand() const override { return "hikrobot"; }
    std::string model() const override { return model_; }

    private:
    HikTuning(void * h, const std::string & model) : h_(h), model_(model)
    {
      MV_CC_SetEnumValue(h_, "TriggerMode", 0);            // 关触发，自由运行
      MV_CC_SetEnumValue(h_, "AcquisitionMode", 2);        // Continuous
      MV_CC_SetEnumValue(h_, "ExposureAuto", 0);           // 手动曝光（否则改了会被自动覆盖）
      MV_CC_SetEnumValue(h_, "GainAuto", 0);               // 手动增益
      MV_CC_SetEnumValue(h_, "BalanceWhiteAuto", 1);       // 自动白平衡

      MVCC_INTVALUE iv;
      memset(&iv, 0, sizeof(iv));
      if (MV_CC_GetIntValue(h_, "Width", &iv) == MV_OK) w_px_ = static_cast<int>(iv.nCurValue);
      memset(&iv, 0, sizeof(iv));
      if (MV_CC_GetIntValue(h_, "Height", &iv) == MV_OK) h_px_ = static_cast<int>(iv.nCurValue);

      MVCC_FLOATVALUE fv;
      memset(&fv, 0, sizeof(fv));
      if (MV_CC_GetFloatValue(h_, "ExposureTime", &fv) == MV_OK)
      {
        expo_min_ = fv.fMin;
        expo_max_ = fv.fMax;
      }
      memset(&fv, 0, sizeof(fv));
      if (MV_CC_GetFloatValue(h_, "AutoGainUpperLimit", &fv) == MV_OK && fv.fMax > 0) gain_upper_ = fv.fMax;

      MV_CC_StartGrabbing(h_);
    }

    void * h_ = nullptr;
    int w_px_ = 0, h_px_ = 0;
    double expo_min_ = 30.0, expo_max_ = 1000000.0;
    // 拿不到 AutoGainUpperLimit 时的兜底值。海康常见量程上限约 16~24dB，
    // 取 16 偏保守：宁可增益偏小（画面偏暗、可继续加曝光），
    // 也不要偏大导致噪点淹掉灯条边缘。
    double gain_upper_ = 16.0;
    std::string model_ = "unknown";
  };

  /// 按「先海康、后大恒」的顺序尝试打开。
  /// 这个顺序没有偏好含义，只是要有个确定行为；两家都插着时应显式指定。
  std::unique_ptr<TuningCamera> open_any_camera()
  {
    if (auto hik = HikTuning::open()) return hik;
    if (auto dh = DahengTuning::open()) return dh;
    return nullptr;
  }
}  // namespace

int main()
{
  auto cam = open_any_camera();
  if (!cam)
  {
    tools::logger()->error("没有找到可用相机");
    tools::logger()->error("排查顺序：");
    tools::logger()->error("  1. lsusb 看设备在不在（海康 2bdf:xxxx / 大恒 2ba2:xxxx）");
    tools::logger()->error("  2. 是否被其他程序占用（detect_test / hik_grab / rb_auto_aim_debug）");
    tools::logger()->error("  3. 海康需装 /opt/MVS；大恒需装 Galaxy SDK");
    return 1;
  }

  tools::logger()->info("相机：{} {}  分辨率 {}x{}", cam->brand(), cam->model(), cam->width(), cam->height());
  tools::logger()->info("曝光量程 {:.0f}-{:.0f}us  增益量程 {:.1f}-{:.1f}（原始值）", cam->expo_min(), cam->expo_max(), cam->gain_raw_min(),
                        cam->gain_raw_max());

  // ---- 初始曝光/增益：从 calibration.yaml 读，这样一启动就是当前上车用的值 ----
  double expo = 18000.0, gain = 0.3;

  // flip/mirror 与上车程序共用同一份配置, 保证这里看到的方向就是
  // detect_test / rb_auto_aim_debug 看到的方向。
  // 两者同时为 true 等价于旋转 180 度(相机倒装时需要)。
  bool flip = true, mirror = true;
  try
  {
    auto yaml = tools::load(CONFIG_PATH);
    expo = tools::read<double>(yaml, "exposure_us");
    gain = tools::read<double>(yaml, "gain");
    flip = tools::read<bool>(yaml, "flip");
    mirror = tools::read<bool>(yaml, "mirror");
    tools::logger()->info("已从 {} 读取初始参数", CONFIG_PATH);
  }
  catch (const std::exception & e)
  {
    tools::logger()->warn("读取 {} 失败({})，使用内置默认值", CONFIG_PATH, e.what());
  }
  tools::logger()->info("画面方向: flip={} mirror={} (按 f 键切换)", flip, mirror);

  expo = std::clamp(expo, cam->expo_min(), cam->expo_max());
  gain = std::clamp(gain, 0.0, 1.0);
  cam->set_exposure(expo);
  cam->set_gain(gain);
  tools::logger()->info("初始值 曝光 {:.0f}us  增益 {:.2f}", expo, gain);

  const int width = cam->width();
  const int height = cam->height();

  double best_sharp = 0.0;
  int saved_hint = 0;         // 存盘提示的剩余显示帧数
  bool show_binary = false;   // 是否显示二值化视图（b 键切换）
  const std::string WIN = "camera tuning";
  cv::namedWindow(WIN, cv::WINDOW_NORMAL);
  cv::resizeWindow(WIN, 1280, 960);

  cv::Mat img;
  while (true)
  {
    if (!cam->read(img)) continue;

    // 应用翻转, 口径与 io/hikrobot/hikrobot.cpp 一致:
    //   flip   -> cv::flip(.., 0) 绕水平轴, 上下颠倒
    //   mirror -> cv::flip(.., 1) 绕垂直轴, 左右镜像
    // 两个都开等于转 180 度。这里用 -1 一次完成, 少一次全图拷贝。
    if (flip && mirror)
      cv::flip(img, img, -1);
    else if (flip)
      cv::flip(img, img, 0);
    else if (mirror)
      cv::flip(img, img, 1);

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    double sharp = sharpness(gray);
    best_sharp = std::max(best_sharp, sharp);
    cv::Scalar mean, sd;
    cv::meanStdDev(gray, mean, sd);
    double total = gray.total();
    double over = cv::countNonZero(gray >= 250) / total * 100.0;
    double under = cv::countNonZero(gray <= 5) / total * 100.0;
    // 模拟 detector 的二值化：这些像素会成为灯条候选，占比越低干扰越少
    double above_th = cv::countNonZero(gray > DETECT_THRESHOLD) / total * 100.0;

    // ---- 叠加信息 ----
    cv::Mat show;
    if (show_binary)
    {
      // 和 detector 完全一致的二值化，再转回三通道以便叠加彩色文字
      cv::Mat bin;
      cv::threshold(gray, bin, DETECT_THRESHOLD, 255, cv::THRESH_BINARY);
      cv::cvtColor(bin, show, cv::COLOR_GRAY2BGR);
    }
    else
    {
      show = img.clone();
    }

    // 状态栏高度取画面高度的三成，且不超过 300px：
    // 原来硬编码 300，海康 1080 高没问题，但换成分辨率更小的相机
    // （或开了 ROI）时 Rect 会越界抛异常。
    int banner_h = std::min(300, show.rows / 3);
    cv::Mat banner = show(cv::Rect(0, 0, show.cols, banner_h));
    banner *= 0.3;  // 压暗做半透明底，保证白字可读

    auto put = [&](const std::string & s, int y, cv::Scalar c, double scale, int thick) {
      cv::putText(show, s, {25, y}, cv::FONT_HERSHEY_SIMPLEX, scale, c, thick);
    };

    // 清晰度放最大：调焦时唯一需要盯的数
    cv::Scalar sharp_color = sharp > SHARP_GOOD ? cv::Scalar(0, 255, 0) : (sharp > SHARP_FAIR ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255));
    put(fmt::format("SHARP {:.0f}", sharp), 75, sharp_color, 2.4, 5);
    put(fmt::format("(best {:.0f})   GOAL: over {:.0f} = green", best_sharp, SHARP_GOOD), 120, {200, 200, 200}, 0.95, 2);

    // 超阈值占比：调曝光时最该盯的数（等于 detector 二值化后的白色面积）
    cv::Scalar th_color = above_th < ABOVE_TH_LIMIT ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
    put(fmt::format("ABOVE {} : {:.2f}%   GOAL: < {:.0f}%", DETECT_THRESHOLD, above_th, ABOVE_TH_LIMIT), 168, th_color, 1.0, 2);

    cv::Scalar bright_color = (mean[0] > BRIGHT_LO && mean[0] < BRIGHT_HI) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
    put(fmt::format("bright {:.0f} (aim {:.0f}-{:.0f}, dark is OK)   expo {:.0f}us  gain {:.2f}", mean[0], BRIGHT_LO, BRIGHT_HI, expo, gain), 210,
        bright_color, 0.72, 2);
    put(fmt::format("saturated {:.1f}%   black {:.1f}%   [{}]  flip={} mirror={}", over, under, cam->brand(), flip ? "Y" : "N", mirror ? "Y" : "N"),
        248, {180, 180, 180}, 0.7, 2);
    put("b = binary view   f = rotate 180 (fix upside-down)", 275, {140, 140, 140}, 0.62, 1);

    if (saved_hint > 0)
    {
      put(fmt::format("SAVED -> {}", RESULT_PATH), 292, {0, 255, 0}, 0.72, 2);
      saved_hint--;
    }
    else
    {
      put("w/s expo  e/d gain  p save  space snapshot  r reset  q quit", 292, {140, 140, 140}, 0.62, 1);
    }

    // 中心十字：拧镜头时给个固定参照点
    cv::line(show, {width / 2 - 45, height / 2}, {width / 2 + 45, height / 2}, {0, 255, 255}, 2);
    cv::line(show, {width / 2, height / 2 - 45}, {width / 2, height / 2 + 45}, {0, 255, 255}, 2);

    cv::imshow(WIN, show);

    // ---- 按键处理。曝光步进 1000us、增益步进 0.05，都是手感调出来的粒度 ----
    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;

    if (key == 'f')
    {
      // 同时翻两个 = 转 180 度。相机正装/倒装只有这两种情况,
      // 所以不提供单独翻某一个轴 —— 那会让数字左右颠倒, 是个陷阱而非功能。
      flip = !flip;
      mirror = !mirror;
      tools::logger()->info("画面方向切换为: flip={} mirror={}", flip, mirror);
    }
    else if (key == 'b')
    {
      // 切换二值化视图：完全复刻 detector.cpp 第 57-64 行的处理，
      // 看到的就是检测器实际拿去 findContours 的图。
      // 理想状态：没装甲板时几乎全黑；有装甲板时只有灯条是白的。
      show_binary = !show_binary;
    }
    else if (key == 'w')
    {
      expo = std::min(expo + 1000, cam->expo_max());
      cam->set_exposure(expo);
    }
    else if (key == 's')
    {
      expo = std::max(expo - 1000, cam->expo_min());
      cam->set_exposure(expo);
    }
    else if (key == 'e')
    {
      gain = std::min(gain + 0.05, 1.0);
      cam->set_gain(gain);
    }
    else if (key == 'd')
    {
      gain = std::max(gain - 0.05, 0.0);
      cam->set_gain(gain);
    }
    else if (key == 'r')
    {
      best_sharp = 0.0;
    }
    else if (key == ' ')
    {
      cv::imwrite("tuning_snapshot.jpg", img);
      tools::logger()->info("快照已存 build/tuning_snapshot.jpg");
    }
    else if (key == 'p')
    {
      std::ofstream out(RESULT_PATH);
      if (out)
      {
        out << "# 相机调参结果 / camera tuning result\n";
        out << "# 由 build/camera_tuning 按 p 键生成，不会被程序读取。\n";
        out << "# 用法：把下面两行的值抄进 configs/calibration.yaml 生效那一段。\n";
        out << "#\n";
        out << fmt::format("# 相机：{} {}\n", cam->brand(), cam->model());
        out << fmt::format("# 当时画面指标：清晰度 {:.0f}（峰值 {:.0f}）  亮度 {:.0f}\n", sharp, best_sharp, mean[0]);
        out << fmt::format("# 超过 detector 阈值({})的像素占比 {:.2f}%  饱和 {:.2f}%  纯黑 {:.2f}%\n", DETECT_THRESHOLD, above_th, over, under);
        out << fmt::format("# 分辨率 {}x{}  增益原始值 {:.2f}（量程 {:.0f}-{:.0f}）\n", width, height, cam->gain_raw(gain), cam->gain_raw_min(),
                           cam->gain_raw_max());
        out << "#\n";
        out << "# ⚠️ gain 的换算方式两家不同（大恒映射到 dB 量程，海康乘以增益上限），\n";
        out << "#    所以同一个 gain 数字在另一家相机上亮度不同，换相机后需重新调。\n";
        out << "\n";
        out << fmt::format("exposure_us: {:.0f}\n", expo);
        out << fmt::format("gain: {:.2f}\n", gain);
        out << "\n";
        out << "# 画面方向。两者同时为 true 等价于转 180 度(相机倒装)。\n";
        out << "# 必须成对使用: 只改一个会让装甲板数字左右颠倒, 分类器认错。\n";
        out << fmt::format("flip: {}\n", flip ? "true" : "false");
        out << fmt::format("mirror: {}\n", mirror ? "true" : "false");
        out.close();
        tools::logger()->info("参数已存 {} : 曝光 {:.0f}us 增益 {:.2f}", RESULT_PATH, expo, gain);
      }
      else
      {
        tools::logger()->error("写入 {} 失败，检查是否在 build/ 目录下运行", RESULT_PATH);
      }
      saved_hint = 120;
    }
  }

  tools::logger()->info("退出。最终 曝光 {:.0f}us 增益 {:.2f}，清晰度峰值 {:.0f}", expo, gain, best_sharp);
  return 0;
}
