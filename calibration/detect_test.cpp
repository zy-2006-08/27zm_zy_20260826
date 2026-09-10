// ============================================================================
//  相机识别测试 / Live armor detection test
// ----------------------------------------------------------------------------
//  用途：接上相机实时跑装甲板识别，识别到就把框画出来，没识别到就不画。
//        用来验证「相机 + 识别」这条链路，以及现场调参后能不能真的认出板子。
//
//  为什么需要它（工程里原本没有这个组合）：
//    · test_simple      接相机，但只显示原图，不跑识别
//    · auto_aim_test    跑识别并画框，但读的是 assets 里录好的视频，不接相机
//    · rb_auto_aim_debug 接相机、跑识别、画框，但它是上车主程序，
//                       第 45 行 io::Gimbal 要连 C 板串口(/dev/ttyACM1)，
//                       打不开就 exit(1)（见 io/gimbal/gimbal.cpp:31）。
//                       手上只有相机、没接 C 板时根本起不来。
//    本程序 = test_simple 的相机 + auto_aim_test 的识别画框，去掉 C 板依赖。
//
//  ⚠️ 只做 2D 识别，不做 3D 解算。
//     算装甲板在世界系的位置需要云台姿态四元数（solver.set_R_gimbal2world），
//     那个只能从 C 板拿。所以这里不构造 Solver / Tracker / Aimer，
//     只显示「框在哪、认成几号、置信度多少」——也就是「能不能识别」本身。q
//     要验证解算和跟踪，得接上 C 板跑 rb_auto_aim_debug，或用离线视频跑 auto_aim_test。
//
//  ⚠️ 相机独占：本程序运行时 camera_tuning / test_simple 都开不了相机，反之亦然。
//     用 Alt+M 启动时 scripts/run_current.sh 会自动清理占用进程。
//
//  配置：默认读 ../configs/detect_test.yaml（demo.yaml 的识别参数 +
//        calibration.yaml 的实机相机参数）。不能直接用 calibration.yaml ——
//        它没有 yolo_name/threshold 等识别参数，会抛 YAML TypedBadConversion。
//
//  用法：
//    cd build && ./detect_test              # 默认传统CV（与上车一致）
//    cd build && ./detect_test -y           # 改用 YOLO 神经网络
//    cd build && ./detect_test -y -m=yolo7  # 指定后端：yolov5/yolov8/yolo11/yolo7
//                                           # yolo7 = 修正关键点排序的 v8，见 yolos/yolo7.hpp
//    键盘：q 退出   空格 存当前帧   t 切换传统CV/YOLO   p 打印一次详细信息
//
//  两条识别路径的区别（调参前必须搞清）：
//    传统CV  二值化 -> 找轮廓 -> 筛灯条 -> 配对 -> 筛装甲板 -> 认数字
//            受 configs 里 threshold / min_lightbar_* / max_*_ratio 等 10 个参数控制，
//            这些就是调参时要动的旋钮。★上车跑的是这条。
//    YOLO    神经网络直接框出装甲板，再送同一个数字分类器。
//            只受 min_confidence 影响，改上面那些几何参数对它无效。
//            离线测试 tests/auto_aim_test.cpp:85 走的是这条。
// ============================================================================

#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                             | 输出命令行参数说明}"
  "{@config-path c  | ../configs/detect_test.yaml  | yaml配置文件路径 }"
  "{yolo y         |                             | 用YOLO识别；不加则用传统CV（与上车一致）}"
  "{model m        |                             | 覆盖yaml里的yolo_name：yolov5/yolov8/yolo11/yolo7}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  // 默认传统CV，因为上车主程序跑的就是它：
  // src/rb_auto_aim_debug.cpp:178 调的是 detector.detect()，
  // 那里的注释写明「★上场跑的是传统CV，不是 YOLO，上面构造的 yolo 对象从未被调用」。
  // 而 configs 里那 10 个识别参数（threshold / min_lightbar_ratio / max_side_ratio ...）
  // 全都只作用于传统CV。默认给 YOLO 会让人调参调半天发现"改了没反应"。
  bool use_traditional = !cli.has("yolo");

  // -m 指定模型时，把 yaml 复制一份改掉 yolo_name 再用，避免手改配置文件。
  // 这样能一条命令切换 yolov8 / yolo7 做对比，原始 yaml 保持不动。
  auto model_override = cli.get<std::string>("model");
  if (!model_override.empty()) {
    auto tmp_path = "/tmp/detect_test_override.yaml";
    std::ifstream src(config_path);
    std::stringstream buf;
    buf << src.rdbuf();
    auto content = buf.str();
    auto pos = content.find("yolo_name:");
    if (pos != std::string::npos) {
      auto eol = content.find('\n', pos);
      content.replace(pos, eol - pos, "yolo_name: " + model_override);
      std::ofstream dst(tmp_path);
      dst << content;
      dst.close();
      config_path = tmp_path;
      tools::logger()->info("已覆盖 yolo_name 为 {}", model_override);
    }
    else {
      tools::logger()->warn("配置里找不到 yolo_name，忽略 -m");
    }
  }

  io::Camera::initSDK();
  io::Camera camera(config_path);

  // 两个识别器都构造出来，运行时按 t 键切换。
  // debug=false：两者的 debug 模式会自己弹调试窗口（传统CV 还会无条件弹 binary_img），
  // 关掉以免和本程序的主窗口抢焦点、看不清。
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Detector detector(config_path, false);

  tools::logger()->info("识别器：{}（按 t 切换）", use_traditional ? "传统CV" : "YOLO");
  tools::logger()->info("按 q 退出，空格存图，p 打印详情");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;
  int detected_frames = 0;  // 有识别到东西的帧数，用来算命中率
  bool print_once = false;

  const std::string WIN = "detect test";
  cv::namedWindow(WIN, cv::WINDOW_NORMAL);
  cv::resizeWindow(WIN, 1280, 960);

  while (true) {
    camera.read(img, timestamp);
    if (img.empty()) continue;
    frame_count++;

    auto t_start = std::chrono::steady_clock::now();
    auto armors = use_traditional ? detector.detect(img, frame_count) : yolo.detect(img, frame_count);
    auto t_end = std::chrono::steady_clock::now();
    double detect_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    if (!armors.empty()) detected_frames++;

    cv::Mat show = img.clone();

    // ---- 画框：识别到才画，没识别到什么都不画 ----
    for (const auto & armor : armors) {
      // armor.points 是 PnP 用的 4 个角点，顺序 {左上, 右上, 右下, 左下}
      // （见 tasks/auto_aim/armor.hpp 的注释）。按这个顺序连线就是装甲板轮廓。
      // 红蓝按识别出的颜色画，方便确认 get_color 判对没有。
      cv::Scalar box_color = (armor.color == auto_aim::Color::red) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 128, 0);
      for (size_t i = 0; i < armor.points.size(); i++) {
        cv::line(show, armor.points[i], armor.points[(i + 1) % armor.points.size()], box_color, 3);
      }
      // 四个角点单独标出来，角点错位是解算跑偏的常见原因，肉眼能看出来
      tools::draw_points(show, armor.points, {0, 255, 255}, 1);

      // 标注：颜色 + 兵种/数字 + 分类置信度
      std::string label = fmt::format("{} {} {:.2f}", auto_aim::COLORS[static_cast<int>(armor.color)],
                                      auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)], armor.confidence);
      // 顺带量出这块板在图像里的 roll 角（左右灯条中心连线相对水平线的夹角）。
      // 这是排查「斜着识别不了」时最该看的数：
      // yolov8.cpp:300 的 sort_keypoints 按 y 坐标分上下两组，
      // 而小装甲板高宽比只有 55:135，roll 超过 arctan(55/135)=22 度后
      // 左下角的 y 就比右上角小了，四个角点的对应关系整体错位一格，
      // 导致 armor.cpp 里算出的 ratio / rectangular_error 全错、板子被筛掉。
      auto lc = (armor.points[0] + armor.points[3]) / 2;
      auto rc = (armor.points[1] + armor.points[2]) / 2;
      double roll_deg = std::atan2(rc.y - lc.y, rc.x - lc.x) * 57.3;
      tools::draw_text(show, label, {(int)armor.center.x - 60, (int)armor.center.y - 20}, box_color, 0.8, 2);
      tools::draw_text(show, fmt::format("roll {:.0f}deg", roll_deg), {(int)armor.center.x - 60, (int)armor.center.y + 45}, {0, 255, 255}, 0.7, 2);
      tools::draw_point(show, armor.center, {0, 255, 0}, 4);
    }

    // ---- 顶部状态栏 ----
    cv::Mat banner = show(cv::Rect(0, 0, show.cols, 160));
    banner *= 0.3;
    auto put = [&](const std::string & s, int y, cv::Scalar c, double sc, int th) {
      cv::putText(show, s, {25, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, th);
    };

    // 识别到几个：这是本程序最该看的数。0 = 没认出来
    cv::Scalar n_color = armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    put(fmt::format("ARMORS: {}", armors.size()), 60, n_color, 1.8, 4);
    put(fmt::format("{}   {:.1f}ms   frame {}", use_traditional ? "traditional CV" : "YOLO", detect_ms, frame_count), 105, {255, 255, 255}, 0.8, 2);
    put(fmt::format("hit rate {:.1f}%  ({}/{})   t=switch  space=save  q=quit", frame_count ? 100.0 * detected_frames / frame_count : 0.0,
                    detected_frames, frame_count),
        140, {180, 180, 180}, 0.65, 2);

    if (print_once) {
      print_once = false;
      tools::logger()->info("--- 第 {} 帧，识别到 {} 个装甲板 ---", frame_count, armors.size());
      for (const auto & a : armors) {
        tools::logger()->info("  {} {}  置信度 {:.3f}  中心({:.0f},{:.0f})  ratio {:.2f} side_ratio {:.2f}",
                              auto_aim::COLORS[static_cast<int>(a.color)], auto_aim::ARMOR_NAMES[static_cast<int>(a.name)], a.confidence, a.center.x,
                              a.center.y, a.ratio, a.side_ratio);
      }
      if (armors.empty()) tools::logger()->warn("  没识别到。检查：灯条是否通电发亮 / 曝光是否过暗 / 距离是否太远");
    }

    cv::imshow(WIN, show);
    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == 't') {
      use_traditional = !use_traditional;
      frame_count = detected_frames = 0;  // 切换后重新统计命中率
      tools::logger()->info("切换到 {}", use_traditional ? "传统CV" : "YOLO");
    }
    if (key == ' ') {
      // 带时间戳，避免连按几次只留下最后一张（之前用固定文件名，前面的全被覆盖）。
      // 同时把带标注的画面和原始画面都存一份：
      //   *_marked.jpg 看识别结果，*_raw.jpg 可以重新喂给别的参数复现
      auto now = std::chrono::system_clock::now();
      auto tt = std::chrono::system_clock::to_time_t(now);
      char buf[32];
      std::strftime(buf, sizeof(buf), "%H%M%S", std::localtime(&tt));
      auto tag = fmt::format("shot_{}_{}", buf, use_traditional ? "trad" : "yolo");
      cv::imwrite(fmt::format("{}_marked.jpg", tag), show);
      cv::imwrite(fmt::format("{}_raw.jpg", tag), img);
      tools::logger()->info("已存 build/{}_marked.jpg 和 _raw.jpg（{} 识别到 {} 个）", tag, use_traditional ? "传统CV" : "YOLO", armors.size());
      for (const auto & a : armors) {
        tools::logger()->info("    {} {} conf={:.3f} ratio={:.2f} side_ratio={:.2f} rect_err={:.1f}deg",
                              auto_aim::COLORS[static_cast<int>(a.color)], auto_aim::ARMOR_NAMES[static_cast<int>(a.name)], a.confidence, a.ratio,
                              a.side_ratio, a.rectangular_error * 57.3);
      }
    }
    if (key == 'p') print_once = true;
  }

  tools::logger()->info("退出。共 {} 帧，{} 帧有识别结果", frame_count, detected_frames);
  return 0;
}
