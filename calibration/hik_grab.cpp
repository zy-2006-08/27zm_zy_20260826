// ============================================================================
// hik_grab - 海康相机独立取图工具
//
// 为什么单独写这个程序, 而不用现成的 capture / camera_tuning:
//   * capture.cpp        构造 io::Gimbal 连云台串口, 并且 cv::imshow 开窗口,
//                        SSH 无显示环境下跑不起来;
//   * camera_tuning.cpp  只链 gxiapi, 仅支持大恒相机;
//   * detect_test.cpp    要链 auto_aim, 依赖 OpenVINO 和权重文件。
// 本程序只依赖 OpenCV + 海康 SDK, 不碰串口/云台/推理, 纯粹用来验证
// "相机还活着、能出图、曝光增益合适不合适", 适合 SSH 远程调机。
//
// 前置条件 (缺一不可, 详见本文件末尾说明):
//   1. /opt/MVS 已安装 (提供 libMvUsb3vTL.so 等传输层插件);
//   2. udev 规则放通 idVendor 2bdf, 否则普通用户打不开设备;
//   3. usbfs_memory_mb 已调大, 16MB 会丢帧。
//
// 用法:
//   ./hik_grab [输出文件] [曝光us] [增益] [预热帧数]
//   默认:      shot.jpg   5000     10    5
// 例:
//   ./hik_grab /tmp/a.jpg 15000 15 8
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <string>

#include "MvCameraControl.h"

namespace
{
  // 海康 SDK 依赖若干环境变量, 但它对缺失的情况没有兜底, 后果很实际:
  //
  //   * ALLUSERSPROFILE 缺失 -> SDK 的 .cti 里存的是字面模板
  //     "$(ALLUSERSPROFILE)/GenICam/xml/cache", 展开不了就按字面用,
  //     于是在【当前工作目录】下建一个名叫 "$(ALLUSERSPROFILE)" 的目录
  //     存相机 XML 缓存。在仓库里跑就直接把仓库弄脏。
  //   * MVCAM_COMMON_RUNENV 缺失 -> 影响传输层插件定位。
  //
  // 官方 setup.sh 会把这些写进 shell 配置, 但那只在【登录 shell】生效。
  // 而本程序的典型用法恰恰都是非交互的:
  //     ssh host "./build/hik_grab ..."    cron    systemd    VSCode task
  // 这些都不加载 /etc/profile.d, 靠外部环境注定漏。
  //
  // 所以在这里自己补齐: 用 setenv(name, value, 0) —— 第三个参数 0 表示
  // "已存在则不覆盖", 因此用户显式指定的值仍然优先, 这里只做兜底。
  // 必须在任何 MV_CC_* 调用之前执行, 否则 SDK 已经读过变量了。
  void ensure_sdk_env()
  {
    const char * root = "/opt/MVS";

    setenv("MVCAM_SDK_PATH", root, 0);
    setenv("MVCAM_COMMON_RUNENV", "/opt/MVS/lib", 0);
    setenv("MVCAM_GENICAM_CLPROTOCOL", "/opt/MVS/lib/CLProtocol", 0);

    // 与官方 set_env_path.sh 的取值保持一致: ${MVCAM_SDK_PATH}/MVFG
    setenv("ALLUSERSPROFILE", "/opt/MVS/MVFG", 0);
    setenv("GENICAM_CACHE_V3_0", "/opt/MVS/MVFG/GenICam/xml/cache", 0);

    // 缓存目录不存在时 SDK 不会自己建父级, 这里递归建一遍。
    // 权限 0777: 这台机器上自瞄程序和调试工具可能以不同用户跑,
    // 缓存是可再生成的非敏感数据, 放开写权限省去一类偶发失败。
    const char * dirs[] = {"/opt/MVS/MVFG", "/opt/MVS/MVFG/GenICam",
                           "/opt/MVS/MVFG/GenICam/xml",
                           "/opt/MVS/MVFG/GenICam/xml/cache"};
    for (const char * d : dirs) mkdir(d, 0777);
  }

  // 把像素格式枚举翻译成可读名字, 方便一眼看出相机当前输出什么
  const char * pix_name(unsigned int t)
  {
    switch (t)
    {
      case PixelType_Gvsp_Mono8: return "Mono8";
      case PixelType_Gvsp_BayerGR8: return "BayerGR8";
      case PixelType_Gvsp_BayerRG8: return "BayerRG8";
      case PixelType_Gvsp_BayerGB8: return "BayerGB8";
      case PixelType_Gvsp_BayerBG8: return "BayerBG8";
      case PixelType_Gvsp_RGB8_Packed: return "RGB8_Packed";
      case PixelType_Gvsp_BGR8_Packed: return "BGR8_Packed";
      case PixelType_Gvsp_YUV422_Packed: return "YUV422_Packed";
      case PixelType_Gvsp_YUV422_YUYV_Packed: return "YUV422_YUYV_Packed";
      default: return "OTHER";
    }
  }
}  // namespace

int main(int argc, char ** argv)
{
  // 必须最先调用: 补齐 SDK 环境变量, 否则会污染当前工作目录
  ensure_sdk_env();

  const std::string out = (argc > 1) ? argv[1] : "shot.jpg";
  const double exposure = (argc > 2) ? atof(argv[2]) : 5000.0;
  const double gain = (argc > 3) ? atof(argv[3]) : 10.0;
  const int warmup = (argc > 4) ? atoi(argv[4]) : 5;

  // ---------- 1. 枚举设备 ----------
  // 同时枚举 USB 和 GigE: 这台是 USB3 的, 但保留 GigE 便于换相机时复用。
  // 若这里返回 0x80000006 (资源申请失败), 几乎一定是 /opt/MVS 没装,
  // 主库找不到 libMvUsb3vTL.so 这类传输层插件, 跟权限和代码都无关。
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(device_list));

  int ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
  if (ret != MV_OK)
  {
    printf("[FAIL] MV_CC_EnumDevices: 0x%x\n", ret);
    printf("       0x80000006 => 传输层插件缺失, 检查 /opt/MVS 是否安装\n");
    return 1;
  }
  if (device_list.nDeviceNum == 0)
  {
    printf("[FAIL] 未找到相机。检查 USB 连接与 udev 规则 (idVendor 2bdf)\n");
    return 1;
  }

  printf("找到 %u 台相机:\n", device_list.nDeviceNum);
  for (unsigned int i = 0; i < device_list.nDeviceNum; i++)
  {
    const MV_CC_DEVICE_INFO * info = device_list.pDeviceInfo[i];
    if (info->nTLayerType == MV_USB_DEVICE)
      printf("  [%u] USB3  型号=%s  SN=%s  厂商=%s\n", i,
             info->SpecialInfo.stUsb3VInfo.chModelName,
             info->SpecialInfo.stUsb3VInfo.chSerialNumber,
             info->SpecialInfo.stUsb3VInfo.chVendorName);
    else if (info->nTLayerType == MV_GIGE_DEVICE)
      printf("  [%u] GigE  型号=%s  SN=%s\n", i,
             info->SpecialInfo.stGigEInfo.chModelName,
             info->SpecialInfo.stGigEInfo.chSerialNumber);
  }

  // ---------- 2. 打开第 0 台 ----------
  void * handle = nullptr;
  ret = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[0]);
  if (ret != MV_OK)
  {
    printf("[FAIL] MV_CC_CreateHandle: 0x%x\n", ret);
    return 1;
  }

  ret = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
  if (ret != MV_OK)
  {
    // 最常见两种: 被别的进程(含 MVS 客户端/自瞄主程序)独占, 或没有设备节点权限
    printf("[FAIL] MV_CC_OpenDevice: 0x%x\n", ret);
    printf("       0x80000203 => 被其它进程占用   0x80000007 => 无权限\n");
    MV_CC_DestroyHandle(handle);
    return 1;
  }
  printf("相机已打开\n");

  // ---------- 3. 参数设置 ----------
  // 关触发 + 连续采集, 让相机自由出图, 不需要外部触发信号。
  // 曝光/增益都切手动, 否则自动曝光会覆盖下面设的值, 调参就失去意义。
  // 这几个 Set 不检查返回值: 不同型号支持的节点略有差异,
  // 个别节点设不上不该让整个取图流程失败, 真正的判据是最后的亮度自检。
  MV_CC_SetEnumValue(handle, "TriggerMode", 0);
  MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
  MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
  MV_CC_SetEnumValue(handle, "GainAuto", 0);
  MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 1);  // 彩色相机自动白平衡

  if (MV_CC_SetFloatValue(handle, "ExposureTime", static_cast<float>(exposure)) == MV_OK)
    printf("曝光 = %.0f us\n", exposure);
  if (MV_CC_SetFloatValue(handle, "Gain", static_cast<float>(gain)) == MV_OK)
    printf("增益 = %.1f\n", gain);

  // ---------- 4. 开始取流 ----------
  ret = MV_CC_StartGrabbing(handle);
  if (ret != MV_OK)
  {
    printf("[FAIL] MV_CC_StartGrabbing: 0x%x\n", ret);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 1;
  }

  // ---------- 5. 取帧 ----------
  // 丢弃前若干帧: 刚开流时自动白平衡还没收敛, 第一帧颜色通常偏。
  // 用 MV_CC_GetImageBuffer 而不是 MV_CC_GetOneFrameTimeout ——
  // 前者缓存由 SDK 内部分配, 不用自己算 buffer 大小, 效率也更高,
  // 代价是每帧必须配对调用 MV_CC_FreeImageBuffer 归还。
  cv::Mat bgr;
  bool ok = false;

  for (int i = 0; i <= warmup; i++)
  {
    MV_FRAME_OUT frame;
    memset(&frame, 0, sizeof(frame));

    ret = MV_CC_GetImageBuffer(handle, &frame, 2000);
    if (ret != MV_OK)
    {
      printf("[FAIL] MV_CC_GetImageBuffer(第%d帧): 0x%x\n", i, ret);
      break;  // 取流失败时没拿到缓存, 无需 Free
    }

    if (i == warmup)
    {
      const unsigned int w = frame.stFrameInfo.nWidth;
      const unsigned int h = frame.stFrameInfo.nHeight;
      const unsigned int pt = frame.stFrameInfo.enPixelType;

      printf("帧: %ux%u  像素格式=%s(0x%x)  长度=%u\n", w, h, pix_name(pt), pt,
             frame.stFrameInfo.nFrameLen);

      // 统一交给 SDK 转 BGR8, 不自己写 Bayer/YUV 分支。
      // 手写 cv::cvtColor 要选对 COLOR_BayerXX2BGR 的排列, 选错了颜色会翻,
      // 而 ConvertPixelType 直接按相机上报的 enPixelType 处理, 不会错。
      bgr.create(h, w, CV_8UC3);

      MV_CC_PIXEL_CONVERT_PARAM cvt;
      memset(&cvt, 0, sizeof(cvt));
      cvt.nWidth = w;
      cvt.nHeight = h;
      cvt.pSrcData = frame.pBufAddr;
      cvt.nSrcDataLen = frame.stFrameInfo.nFrameLen;
      cvt.enSrcPixelType = frame.stFrameInfo.enPixelType;
      cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      cvt.pDstBuffer = bgr.data;
      cvt.nDstBufferSize = w * h * 3;

      const int cret = MV_CC_ConvertPixelType(handle, &cvt);
      if (cret != MV_OK)
        printf("[FAIL] MV_CC_ConvertPixelType: 0x%x\n", cret);
      else
        ok = true;
    }

    MV_CC_FreeImageBuffer(handle, &frame);
  }

  // ---------- 6. 保存 + 亮度自检 ----------
  // SSH 下看不到图, 所以用统计量代替眼睛: 全黑说明镜头盖没取或曝光太低,
  // 大面积贴 255 说明过曝。这两种情况图存下来了也没用, 必须当场提示。
  if (ok && !bgr.empty())
  {
    cv::imwrite(out, bgr);

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    double mn = 0, mx = 0;
    cv::minMaxLoc(gray, &mn, &mx);
    const cv::Scalar m = cv::mean(bgr);

    printf("已保存: %s\n", out.c_str());
    printf("亮度自检: BGR均值=(%.1f, %.1f, %.1f)  灰度min=%.0f max=%.0f\n", m[0], m[1], m[2], mn, mx);

    if (mx < 5)
      printf("警告: 画面基本全黑, 检查镜头盖 / 加大曝光\n");
    else if (mn > 250)
      printf("警告: 画面过曝, 调低曝光或增益\n");
    else
      printf("成像正常\n");
  }

  // ---------- 7. 清理 ----------
  // 三步顺序固定: 停流 -> 关设备 -> 销毁句柄。
  // 少哪一步都可能让相机保持独占, 下次打开报 0x80000203。
  MV_CC_StopGrabbing(handle);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);
  printf("相机已关闭\n");

  return ok ? 0 : 1;
}

// ============================================================================
// 环境准备备忘 (2026-09 在小电脑上实际踩过的坑)
//
// 1) 必须装 /opt/MVS。本仓库 io/hikrobot/lib/<arch>/ 只有 libMvCameraControl.so
//    这一个主库, 而它运行时要 dlopen 传输层插件, USB 相机靠的是
//    libMvUsb3vTL.so。缺插件时 MV_CC_EnumDevices 直接返回 0x80000006,
//    设备数 0, root 跑也一样 —— 很容易误判成权限或相机坏了。
//    (readme 里"海康这一侧不需要额外安装系统级 SDK"的说法是不成立的)
//
// 2) 权限: MVS 安装脚本 bin/set_usb_priority.sh 会生成
//    /etc/udev/rules.d/80-drivers-SDK-2bdf.rules, 放通 idVendor 2bdf。
//    没有它, /dev/bus/usb/* 是 root:root, 普通用户 OpenDevice 失败。
//
// 3) USB 缓冲: /sys/module/usbcore/parameters/usbfs_memory_mb 默认 16, 太小。
//    已用 /etc/tmpfiles.d/usbfs-memory.conf 固定为 1000 (开机自动生效)。
//    注意本机 usbcore 是编进内核的, 不是模块, 所以 modprobe.d 的写法无效。
// ============================================================================
