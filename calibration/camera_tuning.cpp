// ============================================================================
//  相机调参助手 / Camera tuning helper
// ----------------------------------------------------------------------------
//  用途：接上相机后实时看画面，同时把「对焦清晰度」和「亮度」量化成数字显示在
//        画面上，边拧镜头边看数字，不用靠肉眼猜。曝光/增益可用键盘实时改，
//        改完按 p 存到 configs/tuning_result.yaml，再手动抄进 calibration.yaml。
//
//  为什么不直接读写 configs/calibration.yaml：
//    那个文件里有 5 段相机配置（4 段注释掉的备用参数）、标定矩阵、手眼变换，
//    还夹着大量中文注释。让程序自动改这种文件容易改错位置且不易发现，
//    所以只写一个独立的结果文件，最后一步由人工确认后填入。
//
//  为什么不用工程的 io::Camera：
//    io::Camera 只在构造时设一次曝光（见 io/daheng/daheng.cpp 的
//    initialize_camera），运行期改不了。调参需要「改完立刻看到效果」，
//    所以这里直接调大恒 SDK，绕过工程封装。代价是只支持大恒相机 ——
//    调参是线下一次性工作，够用。
//
//  ⚠️ 相机是独占访问：本程序运行时 test_simple / rb_auto_aim_debug 都打不开
//     相机（报 -0x3ec），反之亦然。调完记得按 q 退出。
//
//  用法：
//    cd build && ./camera_tuning
//    窗口弹在小电脑接的显示器上（SSH 里跑需要先 export DISPLAY=:0）
// ============================================================================

#include <fmt/core.h>

#include <algorithm>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "GxIAPI.h"
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

  /// 大恒的增益是 dB 值（本机量程 0-24），而工程 yaml 里的 gain 是 0-1 归一化值，
  /// 由 io/daheng/daheng.cpp 换算成 dB。这里保持同样的换算口径，
  /// 保证 p 键存出来的数字可以直接抄进 yaml。
  double gain_to_db(double norm, const GX_FLOAT_RANGE & r) { return r.dMin + norm * (r.dMax - r.dMin); }
  double db_to_gain(double db, const GX_FLOAT_RANGE & r) { return (r.dMax > r.dMin) ? (db - r.dMin) / (r.dMax - r.dMin) : 0.0; }
}  // namespace

int main()
{
  // ---- 初始化 SDK 并打开相机 ----
  if (GXInitLib() != GX_STATUS_SUCCESS) {
    tools::logger()->error("大恒 SDK 初始化失败");
    return 1;
  }
  uint32_t dev_num = 0;
  GXUpdateDeviceList(&dev_num, 1000);
  if (dev_num == 0) {
    tools::logger()->error("没有找到相机，检查 USB 线");
    GXCloseLib();
    return 1;
  }
  GX_DEV_HANDLE handle = nullptr;
  GX_STATUS status = GXOpenDeviceByIndex(1, &handle);
  if (status != GX_STATUS_SUCCESS) {
    // -0x3ec(-1004) = 设备被占用。最常见原因是 test_simple 或自瞄程序还开着。
    tools::logger()->error("打开相机失败 status={:#x}，相机可能被其他程序占用", status);
    tools::logger()->error("先关掉 test_simple / rb_auto_aim_debug 再重试");
    GXCloseLib();
    return 1;
  }

  // ---- 初始曝光/增益：从 calibration.yaml 读，这样一启动就是当前上车用的值 ----
  double expo = 18000.0, gain = 0.3;
  try {
    auto yaml = tools::load(CONFIG_PATH);
    expo = tools::read<double>(yaml, "exposure_us");
    gain = tools::read<double>(yaml, "gain");
    tools::logger()->info("已从 {} 读取初始参数", CONFIG_PATH);
  }
  catch (const std::exception & e) {
    tools::logger()->warn("读取 {} 失败({})，使用内置默认值", CONFIG_PATH, e.what());
  }

  int64_t width = 0, height = 0, color_filter = 0;
  GXGetInt(handle, GX_INT_WIDTH, &width);
  GXGetInt(handle, GX_INT_HEIGHT, &height);
  GXGetEnum(handle, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter);

  GX_FLOAT_RANGE expo_range{}, gain_range{};
  GXGetFloatRange(handle, GX_FLOAT_EXPOSURE_TIME, &expo_range);
  GXSetEnum(handle, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
  GXGetFloatRange(handle, GX_FLOAT_GAIN, &gain_range);

  GXSetFloat(handle, GX_FLOAT_EXPOSURE_TIME, expo);
  GXSetFloat(handle, GX_FLOAT_GAIN, gain_to_db(gain, gain_range));
  GXSetEnum(handle, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
  GXSetEnum(handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  GXStreamOn(handle);

  tools::logger()->info("分辨率 {}x{}，曝光量程 {:.0f}-{:.0f}us，增益量程 {:.1f}-{:.1f}dB", width, height, expo_range.dMin, expo_range.dMax,
                        gain_range.dMin, gain_range.dMax);
  tools::logger()->info("初始值 曝光 {:.0f}us  增益 {:.2f}", expo, gain);

  // Bayer 转彩色。工程里用大恒的 DxRaw8toRGB24，但那个库(libdximageproc)
  // 小电脑上没装，调参只是看画面，用 OpenCV 的 demosaic 完全够。
  int bayer_code = cv::COLOR_BayerGR2BGR;
  switch (color_filter) {
    case GX_COLOR_FILTER_BAYER_RG: bayer_code = cv::COLOR_BayerRG2BGR; break;
    case GX_COLOR_FILTER_BAYER_GB: bayer_code = cv::COLOR_BayerGB2BGR; break;
    case GX_COLOR_FILTER_BAYER_BG: bayer_code = cv::COLOR_BayerBG2BGR; break;
    default: break;
  }

  std::vector<uint8_t> raw(width * height);
  GX_FRAME_DATA frame{};
  frame.pImgBuf = raw.data();

  double best_sharp = 0.0;
  int saved_hint = 0;      // 存盘提示的剩余显示帧数
  bool show_binary = false;  // 是否显示二值化视图（b 键切换）
  const std::string WIN = "camera tuning";
  cv::namedWindow(WIN, cv::WINDOW_NORMAL);
  cv::resizeWindow(WIN, 1280, 960);

  while (true) {
    if (GXGetImage(handle, &frame, 200) != GX_STATUS_SUCCESS) continue;
    if (frame.nStatus != 0) continue;

    cv::Mat bayer(height, width, CV_8UC1, frame.pImgBuf);
    cv::Mat img;
    cv::cvtColor(bayer, img, bayer_code);
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
    if (show_binary) {
      // 和 detector 完全一致的二值化，再转回三通道以便叠加彩色文字
      cv::Mat bin;
      cv::threshold(gray, bin, DETECT_THRESHOLD, 255, cv::THRESH_BINARY);
      cv::cvtColor(bin, show, cv::COLOR_GRAY2BGR);
    }
    else {
      show = img.clone();
    }
    cv::Mat banner = show(cv::Rect(0, 0, show.cols, 300));
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
    put(fmt::format("saturated {:.1f}%   black {:.1f}%", over, under), 248, {180, 180, 180}, 0.7, 2);
    put("b = toggle binary view (what detector sees)", 275, {140, 140, 140}, 0.62, 1);

    if (saved_hint > 0) {
      put(fmt::format("SAVED -> {}", RESULT_PATH), 292, {0, 255, 0}, 0.72, 2);
      saved_hint--;
    }
    else {
      put("w/s expo  e/d gain  p save  space snapshot  r reset  q quit", 292, {140, 140, 140}, 0.62, 1);
    }

    // 中心十字：拧镜头时给个固定参照点
    cv::line(show, {(int)width / 2 - 45, (int)height / 2}, {(int)width / 2 + 45, (int)height / 2}, {0, 255, 255}, 2);
    cv::line(show, {(int)width / 2, (int)height / 2 - 45}, {(int)width / 2, (int)height / 2 + 45}, {0, 255, 255}, 2);

    cv::imshow(WIN, show);

    // ---- 按键处理。曝光步进 1000us、增益步进 0.05，都是手感调出来的粒度 ----
    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;

    if (key == 'b') {
      // 切换二值化视图：完全复刻 detector.cpp 第 57-64 行的处理，
      // 看到的就是检测器实际拿去 findContours 的图。
      // 理想状态：没装甲板时几乎全黑；有装甲板时只有灯条是白的。
      show_binary = !show_binary;
    }

    if (key == 'w') {
      expo = std::min(expo + 1000, expo_range.dMax);
      GXSetFloat(handle, GX_FLOAT_EXPOSURE_TIME, expo);
    }
    else if (key == 's') {
      expo = std::max(expo - 1000, expo_range.dMin);
      GXSetFloat(handle, GX_FLOAT_EXPOSURE_TIME, expo);
    }
    else if (key == 'e') {
      gain = std::min(gain + 0.05, 1.0);
      GXSetFloat(handle, GX_FLOAT_GAIN, gain_to_db(gain, gain_range));
    }
    else if (key == 'd') {
      gain = std::max(gain - 0.05, 0.0);
      GXSetFloat(handle, GX_FLOAT_GAIN, gain_to_db(gain, gain_range));
    }
    else if (key == 'r') {
      best_sharp = 0.0;
    }
    else if (key == ' ') {
      cv::imwrite("tuning_snapshot.jpg", img);
      tools::logger()->info("快照已存 build/tuning_snapshot.jpg");
    }
    else if (key == 'p') {
      std::ofstream out(RESULT_PATH);
      if (out) {
        out << "# 相机调参结果 / camera tuning result\n";
        out << "# 由 build/camera_tuning 按 p 键生成，不会被程序读取。\n";
        out << "# 用法：把下面两行的值抄进 configs/calibration.yaml 生效那一段。\n";
        out << "#\n";
        out << fmt::format("# 当时画面指标：清晰度 {:.0f}（峰值 {:.0f}）  亮度 {:.0f}\n", sharp, best_sharp, mean[0]);
        out << fmt::format("# 超过 detector 阈值({})的像素占比 {:.2f}%  饱和 {:.2f}%  纯黑 {:.2f}%\n", DETECT_THRESHOLD, above_th, over, under);
        out << fmt::format("# 分辨率 {}x{}  增益原始值 {:.2f}dB（量程 {:.0f}-{:.0f}）\n", width, height, gain_to_db(gain, gain_range), gain_range.dMin,
                           gain_range.dMax);
        out << "\n";
        out << fmt::format("exposure_us: {:.0f}\n", expo);
        out << fmt::format("gain: {:.2f}\n", gain);
        out.close();
        tools::logger()->info("参数已存 {} : 曝光 {:.0f}us 增益 {:.2f}", RESULT_PATH, expo, gain);
      }
      else {
        tools::logger()->error("写入 {} 失败，检查是否在 build/ 目录下运行", RESULT_PATH);
      }
      saved_hint = 120;
    }
  }

  GXStreamOff(handle);
  GXCloseDevice(handle);
  GXCloseLib();
  tools::logger()->info("退出。最终 曝光 {:.0f}us 增益 {:.2f}，清晰度峰值 {:.0f}", expo, gain, best_sharp);
  return 0;
}
