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
#include <thread>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

// rerun：把数据画成随时间变化的波形。
// USE_RERUN 由 CMakeLists 在找到 SDK 时定义；没有 SDK 时整段编译掉，
// 程序退化成「只跑识别、不出任何窗口」，仍然能编过。
#ifdef USE_RERUN
#include <rerun.hpp>

// 每秒最多往 rerun 发几张画面。
//
// ★为什么按「时间」限流而不是按「帧数」限流（踩过的坑）：
//   原来写的是 frame_count % 5 == 0，按帧数隔 5 帧发一张。
//   但这个相机实测跑 94 fps（不是我假设的 30fps），
//   所以实际每秒发 94/5 = 18 张，而不是预期的 6 张 ——
//   16 MB/s 灌进 Viewer，700MB 限额 43 秒就满，
//   之后 Viewer 边收边丢最老数据，CPU 飙到 104%，画面就卡了。
//
//   按帧数限流的根本问题：相机跑多快就发多快，帧率一变就失控。
//   按时间限流则不管相机多少 fps，每秒固定只发这么多张。
//
// ★为什么是 30 而不是 10：
//   限流本身会带来「采样延迟」——两次发送之间的动作要等下一次才被拍到。
//   10fps 时这个延迟最坏 100ms，在镜头前比个手势明显感觉到迟滞。
//   30fps 降到 33ms，基本感觉不出来。
//
//   之所以敢开到 30，是因为下面改用了 JPEG：一张只有约 40KB，
//   30fps 也才 1.2 MB/s。而原来发原始 RGB 时一张 960KB，
//   30fps 就是 28 MB/s —— 那是撑不住的。
constexpr int IMAGE_FPS = 30;

// JPEG 质量 (0~100)。
//
// 为什么用 JPEG 而不是原始 RGB：
//   640x512 原始 RGB = 960 KB/张，10fps 就是 9.4 MB/s。
//   同尺寸 JPEG(q=70) 约 40 KB/张，10fps 只有 0.4 MB/s —— 降到 1/24。
//   700MB 限额能存约 30 分钟，而且 Viewer 不用再忙着丢数据。
//
// 70 是画质和体积的平衡点：装甲板框、角点、状态栏文字都清晰可读。
// 真要看灯条像素级细节，那是 camera_tuning 的活，不该在这里看。
constexpr int JPEG_QUALITY = 70;
#endif

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

  // ---- 复刻 tracker.cpp:326 的四态机 ----
  // hit rate 是累计平均值，看不出漏检是「零散分布」还是「连续成片」，
  // 而这两者对上车的影响完全不同：
  //   零散漏 1~2 帧  temp_lost 兜住，云台照常跟
  //   连续漏 >11 帧  真丢跟踪，回 lost，重新发现还要再攒 5 帧
  // 所以这里按 tracker 的规则实时算出「现在到底算不算跟上了」。
  // 参数取自 configs 的 min_detect_count / max_temp_lost_count。
  int min_detect_count = 5;
  int max_temp_lost_count = 11;
  try {
    auto ty = tools::load(config_path);
    min_detect_count = tools::read<int>(ty, "min_detect_count");
    max_temp_lost_count = tools::read<int>(ty, "max_temp_lost_count");
  }
  catch (const std::exception & e) {
    tools::logger()->warn("读取 tracker 参数失败({})，用默认 5/11", e.what());
  }
  tools::logger()->info("状态机参数：min_detect_count={} max_temp_lost_count={}", min_detect_count, max_temp_lost_count);

  // ---- ROI 配置：开启后网络只看这一块 ----
  // 板子跑到框外就完全检不到，而画面上原本没有任何提示，
  // 很容易把「板子出框」误判成「ROI 让识别变差了」。所以把框画出来。
  bool use_roi = false;
  cv::Rect roi_rect;
  try {
    auto ry = tools::load(config_path);
    use_roi = tools::read<bool>(ry, "use_roi");
    if (use_roi) {
      auto node = ry["roi"];
      roi_rect = cv::Rect(node["x"].as<int>(), node["y"].as<int>(), node["width"].as<int>(), node["height"].as<int>());
      tools::logger()->info("ROI 已开启 ({},{}) {}x{} —— 网络只看框内，板子出框就检不到", roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
    }
    else {
      tools::logger()->info("ROI 关闭，网络看整幅画面");
    }
  }
  catch (const std::exception & e) {
    tools::logger()->warn("读取 ROI 配置失败：{}", e.what());
  }

  std::string tstate = "lost";
  int detect_cnt = 0;       // detecting 阶段连续命中计数
  int lost_cnt = 0;         // temp_lost 阶段连续丢失计数
  int miss_streak = 0;      // 当前连续漏检帧数
  int worst_miss = 0;       // 历史最长连续漏检
  int tracking_frames = 0;  // 处于 tracking/temp_lost（即真正在瞄）的帧数
  int lost_events = 0;      // 掉回 lost 的次数
  bool print_once = false;

  // ---- 图像窗口已关闭 ----
  // 原来这里建 cv::namedWindow + resizeWindow，配合下面的 cv::imshow 显示画面。
  // 现在只看 rerun 波形，图像窗口关掉：
  //   · 少一次每帧的图像拷贝(show = img.clone())和几十次绘图调用，帧率更高
  //   · 不再抢焦点，rerun 窗口能独占屏幕
  // 想恢复图像窗口的话，把这三行和下面 cv::imshow 那段的注释一起去掉即可。
  // const std::string WIN = "detect test";
  // cv::namedWindow(WIN, cv::WINDOW_NORMAL);
  // cv::resizeWindow(WIN, 1280, 960);

#ifdef USE_RERUN
  // ---- rerun 初始化 ----
  // "detect_test" 是 application_id，Viewer 标题栏显示的名字。
  // spawn() 会自动启动一个 rerun Viewer 进程并连上去；
  // 找不到 rerun 命令时它返回错误而不是崩溃，所以下面要检查。
  const auto rec = rerun::RecordingStream("detect_test");

  // ★内存上限必须显式设置，这是防止小电脑被拖死的关键。
  //
  // 踩过的坑（两次卡死到 SSH 都连不上，只能强制重启）：
  //   默认 memory_limit="75%"，即允许 Viewer 吃掉 11GB(总16GB)。
  //   Viewer 会把收到的每一帧都缓存在内存里（为了支持拖游标回看历史），
  //   1280x1024 的 RGB 图 3.9MB/帧，30fps 就是 118MB/s，
  //   一分半钟吃满 11GB，系统开始换页，整机失去响应。
  //
  // ★更隐蔽的坑：spawn() 遇到「Viewer 已经在跑」时只会连上去，
  //   不会拿新参数重启它。所以之前跑过 demo.py 留下的那个
  //   默认 75% 限额的 Viewer 一直生效，这里设 2GB 也没用 ——
  //   这就是降采样到 1/20 之后仍然卡死的真正原因。
  //   解决办法在使用流程里：跑本程序前先 pkill -f 'rerun --port'。
  //
  // 700MB 够存约 10 分钟历史（按下面降采样后的数据量算），调车足够。
  // 超限时 Viewer 自动丢弃最老的数据，绝不会再吃满内存。
  rerun::SpawnOptions spawn_opts;
  spawn_opts.memory_limit = "700MB";
  spawn_opts.server_memory_limit = "256MB";
  // 欢迎页对调车没用，每次启动还要多点一下才能看到数据
  spawn_opts.hide_welcome_screen = true;

  if (rec.spawn(spawn_opts).is_err()) {
    // 这里必须用 fmt 直接打印而不是 tools::logger —— 此时 rerun 起不来，
    // 而 logger 的输出后面被我们关掉了，不提示的话用户只会看到「没反应」。
    fmt::print("rerun Viewer 启动失败：确认 ~/.local/bin/rerun 存在且在 PATH 里\n");
    fmt::print("  检查命令: rerun --version\n");
    return 1;
  }

  // 状态机是字符串，而波形图只能画数字，所以映射成 0~3。
  // 数值大小刻意按「是否在瞄」排序，这样波形越高 = 越接近稳定跟踪：
  //   3 tracking    稳定跟上，Target 正常交给上层打
  //   2 temp_lost   短暂丢失，还在用预测继续瞄（见 tracker.cpp 状态机）
  //   1 detecting   正在攒连续命中帧，还没开始瞄
  //   0 lost        完全没跟上
  // 波形从 3 掉到 0 就是一次真丢跟踪，一眼能看出发生在第几帧。
  auto state_to_num = [](const std::string & s) -> double {
    if (s == "tracking") return 3.0;
    if (s == "temp_lost") return 2.0;
    if (s == "detecting") return 1.0;
    return 0.0;  // lost
  };

  // 静态样式只需设一次（log_static 表示不随时间变化的数据）。
  // 不设的话两条曲线颜色随机，每次启动都不一样，看久了容易认错。
  //
  // 颜色用 Rgba32 而不是 Color —— 官方 0.37 示例(series_lines.hpp 文档注释)
  // 用的就是 Rgba32{r,g,b}，照抄能避免类型转换歧义。
  //
  // 路径名用 ASCII（tstate / roll_deg）而不是中文：
  // Blueprint 的 contents 是路径匹配表达式，中文路径匹配时容易出意外；
  // 而且要和 rerun_pkg/layout.py 里写的路径完全一致，布局才能对上。
  rec.log_static(
    "tstate",
    rerun::SeriesLines().with_colors(rerun::Rgba32{0, 220, 80}).with_names("tstate").with_widths(2.0f));
  rec.log_static(
    "roll_deg",
    rerun::SeriesLines().with_colors(rerun::Rgba32{255, 200, 0}).with_names("roll_deg").with_widths(2.0f));
#endif

  while (true) {
    camera.read(img, timestamp);
    if (img.empty()) continue;
    frame_count++;

    auto t_start = std::chrono::steady_clock::now();
    auto armors = use_traditional ? detector.detect(img, frame_count) : yolo.detect(img, frame_count);
    auto t_end = std::chrono::steady_clock::now();
    double detect_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    bool found = !armors.empty();
    if (found) detected_frames++;

    // 连续漏检统计
    if (found) miss_streak = 0;
    else { miss_streak++; worst_miss = std::max(worst_miss, miss_streak); }

    // 状态机（与 tracker.cpp:326 的 state_machine 同规则）
    if (tstate == "lost") {
      if (found) { tstate = "detecting"; detect_cnt = 1; }
    }
    else if (tstate == "detecting") {
      // ★注意：detecting 要求「连续」命中，断一帧立刻掉回 lost
      if (found) {
        detect_cnt++;
        if (detect_cnt >= min_detect_count) { tstate = "tracking"; lost_cnt = 0; }
      }
      else { tstate = "lost"; detect_cnt = 0; }
    }
    else if (tstate == "tracking") {
      if (found) lost_cnt = 0;
      else { tstate = "temp_lost"; lost_cnt = 1; }
    }
    else if (tstate == "temp_lost") {
      if (found) { tstate = "tracking"; lost_cnt = 0; }
      else {
        lost_cnt++;
        if (lost_cnt > max_temp_lost_count) { tstate = "lost"; detect_cnt = 0; lost_events++; }
      }
    }
    if (tstate == "tracking" || tstate == "temp_lost") tracking_frames++;

#ifdef USE_RERUN
    // ---- 时间轴 ----
    // 必须先设时间轴，再 log 数据，否则所有点会堆在同一时刻、画不出波形。
    // 用 frame_count 而不是真实时间戳：横轴就是帧号，跟 detector 的
    // frame_count 完全对齐，看到「第 137 帧掉了」可以直接去查那一帧。
    rec.set_time_sequence("frame", frame_count);

    // ---- 数据 1/2：识别状态 ----
    // 每帧都发，因为状态机每帧都有确定值（不像 roll 会缺）。
    // 波形高低含义：3=tracking 2=temp_lost 1=detecting 0=lost
    rec.log("tstate", rerun::Scalars(state_to_num(tstate)));

    // ---- 加新变量就是这么简单：一行 ----
    // layout.py 里用的是通配符 contents=["+ /**", "- /camera/**"]，
    // 所以任何新 log 的标量都会自动出现在波形图和数值表里，
    // 不需要回去改 layout.py。删掉这行，曲线就自动消失。
    //
    // detect_ms 是识别耗时（毫秒），拿来当示例正好 ——
    // 它能看出 YOLO 和传统CV 的性能差距，也能发现偶发的卡顿帧。
    rec.log("detect_ms", rerun::Scalars(detect_ms));

    // ★这里原本还发了一路 rerun::TextLog(tstate) 显示状态原文，已去掉。
    //   原因：Viewer 会给 TextLog 自动分配一个文本视图，屏幕上出现一整列
    //   重复的 "tracking" 文字，占掉约 1/4 界面，把波形挤小了。
    //   状态名靠波形高度就能读出来（3/2/1/0 四档），不需要文字列。
    //   真要看原文的话，把下面这行的注释去掉：
    // rec.log("tstate/text", rerun::TextLog(tstate));
#endif

    // ---- 这一帧要不要出图？----
    // ★CPU 优化的核心判断，放在所有画面相关计算之前。
    //
    // 原来的写法把「算统计 + clone + 画框 + 画状态栏」全部无条件执行，
    // 然后在最后才判断要不要发图。相机跑 94fps 而只发 30fps ——
    // 意味着约 2/3 的图像处理白做了，实测 detect_test 占 374% CPU(3.7核)。
    //
    // 现在提前判断：不出图的帧完全跳过这些工作，只跑识别 + 发标量波形。
    // 波形数据不受影响（每帧都发，一个 double 才 8 字节），
    // 所以横轴帧号仍然连续、不缺帧。
    bool draw_this_frame = true;
#ifdef USE_RERUN
    {
      static auto last_img_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration<double, std::milli>(now - last_img_time).count();
      draw_this_frame = elapsed_ms >= 1000.0 / IMAGE_FPS;
      if (draw_this_frame) last_img_time = now;
    }
#endif

    // ---- YOLO 专用画面指标 ----
    // ★与传统CV 的判据相反，别混用：
    //   传统CV 只需要「灯条灰度 > threshold(150)」，中间数字亮不亮无所谓，
    //          二值化后都是黑的，所以它偏好极暗画面。
    //   YOLO   看的是「整块装甲板长什么样」= 两条灯条 + 中间的数字图案。
    //          曝光压太低时数字被压成纯黑，网络看到「两条发光竖线，中间空白」，
    //          不像训练时见过的装甲板，分数掉到 score_threshold_ 以下就漏检。
    // 实测（同相机不同曝光，取网络原始最高分）：
    //   中心区亮度 60~92 -> 网络分数 0.93~0.96
    //   中心区亮度 17~33 -> 网络分数 0.86~0.91
    // 调 YOLO 曝光要盯【中心区亮度】，目标 50~90，不是越暗越好。
    //
    // 这几个统计只用于「画在状态栏上」，不出图的帧算了也没人看，所以跳过。
    // 三次全图遍历(cvtColor/meanStdDev/countNonZero)在 1280x1024 上不便宜。
    cv::Scalar g_mean, c_mean;
    double sat_pct = 0.0;
    if (draw_this_frame) {
      cv::Mat gray_stat;
      cv::cvtColor(img, gray_stat, cv::COLOR_BGR2GRAY);
      cv::Scalar g_sd;
      cv::meanStdDev(gray_stat, g_mean, g_sd);
      cv::Rect crect((gray_stat.cols - 400) / 2, (gray_stat.rows - 300) / 2, 400, 300);
      crect &= cv::Rect(0, 0, gray_stat.cols, gray_stat.rows);
      cv::Scalar c_sd;
      cv::meanStdDev(gray_stat(crect), c_mean, c_sd);
      sat_pct = cv::countNonZero(gray_stat >= 250) / (double)gray_stat.total() * 100;
    }

    // clone 是整幅图的深拷贝(1280x1024x3 = 3.9MB)，94fps 下就是 370MB/s 的
    // 内存带宽。不出图的帧不需要画布，所以空着。
    cv::Mat show;
    if (draw_this_frame) show = img.clone();

#ifdef USE_RERUN
    // 每帧重置：用来区分同一帧里的第几块装甲板，
    // 第 0 块走主曲线「roll角度」，其余的各自分开一条，互不干扰。
    int roll_idx = 0;
#endif

    // ---- 画框：识别到才画，没识别到什么都不画 ----
    // ★注意这个循环有两个职责，不能整体跳过：
    //     画框     只在出图的帧需要 -> 用 draw_this_frame 保护
    //     发 roll_deg 每帧都要 -> 必须无条件执行，否则波形会缺帧
    //   所以下面是「算always、画conditional」的结构。
    for (const auto & armor : armors) {
      // 顺带量出这块板在图像里的 roll 角（左右灯条中心连线相对水平线的夹角）。
      // 这是排查「斜着识别不了」时最该看的数：
      // yolov8.cpp:300 的 sort_keypoints 按 y 坐标分上下两组，
      // 而小装甲板高宽比只有 55:135，roll 超过 arctan(55/135)=22 度后
      // 左下角的 y 就比右上角小了，四个角点的对应关系整体错位一格，
      // 导致 armor.cpp 里算出的 ratio / rectangular_error 全错、板子被筛掉。
      //
      // 这几行必须在 draw_this_frame 之外 —— 波形要每帧都有数据。
      auto lc = (armor.points[0] + armor.points[3]) / 2;
      auto rc = (armor.points[1] + armor.points[2]) / 2;
      double roll_deg = std::atan2(rc.y - lc.y, rc.x - lc.x) * 57.3;

      if (draw_this_frame) {
        // armor.points 是 PnP 用的 4 个角点，顺序 {左上, 右上, 右下, 左下}
        // （见 tasks/auto_aim/armor.hpp 的注释）。按这个顺序连线就是装甲板轮廓。
        // 红蓝按识别出的颜色画，方便确认 get_color 判对没有。
        cv::Scalar box_color =
          (armor.color == auto_aim::Color::red) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 128, 0);
        for (size_t i = 0; i < armor.points.size(); i++) {
          cv::line(show, armor.points[i], armor.points[(i + 1) % armor.points.size()], box_color, 3);
        }
        // 四个角点单独标出来，角点错位是解算跑偏的常见原因，肉眼能看出来
        tools::draw_points(show, armor.points, {0, 255, 255}, 1);

        // 标注：颜色 + 兵种/数字 + 分类置信度
        std::string label = fmt::format(
          "{} {} {:.2f}", auto_aim::COLORS[static_cast<int>(armor.color)],
          auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)], armor.confidence);
        tools::draw_text(show, label, {(int)armor.center.x - 60, (int)armor.center.y - 20}, box_color, 0.8, 2);
        tools::draw_text(show, fmt::format("roll {:.0f}deg", roll_deg), {(int)armor.center.x - 60, (int)armor.center.y + 45}, {0, 255, 255}, 0.7, 2);
        tools::draw_point(show, armor.center, {0, 255, 0}, 4);
      }

#ifdef USE_RERUN
      // ★这里是 roll_deg 唯一存在的地方 —— 它在 armors 循环内算出来，
      //   没识别到装甲板的帧这个循环压根不执行，所以那些帧不会有数据点。
      //   这是刻意的：波形上的空缺就是漏检位置，一眼可见。
      //   若改成漏检时补 0，会画出假的「roll 突然归零」尖峰，
      //   调参时容易误判成「板子转正了」。
      //
      //   只发第一块板（roll_idx==0）。多块板时后面的板另开 roll_deg_armor1 等路径，
      //   避免同一条曲线在两块板之间反复跳变、看不出单块板的真实趋势。
      if (roll_idx == 0) {
        rec.log("roll_deg", rerun::Scalars(roll_deg));
      }
      else {
        rec.log(fmt::format("roll_deg_armor{}", roll_idx), rerun::Scalars(roll_deg));
      }
      roll_idx++;
#endif
    }

    // ---- 顶部状态栏 ----
    // ★整段用 draw_this_frame 包住。不出图的帧 show 是空 Mat(rows=0)，
    //   show(cv::Rect(0,0,cols,280)) 会抛
    //       (-215:Assertion failed) roi.y + roi.height <= m.rows
    //   然后 terminate，退出码 134。这是 CPU 优化时漏掉的保护 ——
    //   之前被 Qt 的 xcb 报错掩盖了，用 QT_QPA_PLATFORM=offscreen 才暴露出来。
    if (draw_this_frame) {
      cv::Mat banner = show(cv::Rect(0, 0, show.cols, 280));
      banner *= 0.3;
      auto put = [&](const std::string & s, int y, cv::Scalar c, double sc, int th) {
        cv::putText(show, s, {25, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, th);
      };

      // 识别到几个：这是本程序最该看的数。0 = 没认出来
      cv::Scalar n_color = armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      put(fmt::format("ARMORS: {}", armors.size()), 60, n_color, 1.8, 4);
      put(fmt::format("{}   {:.1f}ms   frame {}", use_traditional ? "traditional CV" : "YOLO", detect_ms, frame_count), 105, {255, 255, 255}, 0.8, 2);
      put(fmt::format("hit rate {:.1f}%  ({}/{})   Ctrl+C to quit", frame_count ? 100.0 * detected_frames / frame_count : 0.0,
                      detected_frames, frame_count),
          140, {180, 180, 180}, 0.65, 2);

      // ---- 状态机行：这才是「云台到底瞄不瞄」的真实答案 ----
      // tracking / temp_lost 才会把 Target 交给上层去打（见 tracker.cpp 状态机注释）
      cv::Scalar sc_color;
      std::string extra;
      if (tstate == "tracking") { sc_color = {0, 255, 0}; extra = "AIMING"; }
      else if (tstate == "temp_lost") {
        sc_color = {0, 200, 255};
        extra = fmt::format("AIMING (predict {}/{})", lost_cnt, max_temp_lost_count);
      }
      else if (tstate == "detecting") {
        sc_color = {0, 255, 255};
        extra = fmt::format("NOT aiming (need {}/{})", detect_cnt, min_detect_count);
      }
      else { sc_color = {0, 0, 255}; extra = "NOT aiming"; }
      put(fmt::format("{}  ->  {}", tstate, extra), 190, sc_color, 1.0, 2);
      put(fmt::format("aim rate {:.1f}%   miss streak {} (worst {})   lost x{}",
                      frame_count ? 100.0 * tracking_frames / frame_count : 0.0, miss_streak, worst_miss, lost_events),
          228, {170, 170, 170}, 0.62, 2);

      // 中心区亮度：调 YOLO 曝光时唯一要盯的数
      cv::Scalar cb_color = (c_mean[0] >= 50 && c_mean[0] <= 90) ? cv::Scalar(0, 255, 0)
                            : (c_mean[0] < 50)                   ? cv::Scalar(0, 128, 255)
                                                                 : cv::Scalar(0, 255, 255);
      put(fmt::format("center bright {:.0f} GOAL 50-90 [{}]   frame {:.0f}   sat {:.1f}%", c_mean[0],
                      c_mean[0] < 50 ? "RAISE expo" : (c_mean[0] > 90 ? "LOWER expo" : "OK"), g_mean[0], sat_pct),
          262, cb_color, 0.62, 2);
    }

    // ---- 按 p 打印详情：已关闭 ----
    // 原来这段用 tools::logger 往终端刷详细信息。现在终端要保持干净，
    // 而且这些数据在 rerun 波形里看得更清楚（能看趋势，不只是某一帧的值）。
    // if (print_once) {
    //   print_once = false;
    //   tools::logger()->info("--- 第 {} 帧，识别到 {} 个装甲板 ---", frame_count, armors.size());
    //   tools::logger()->info("  画面：中心区亮度 {:.1f}（YOLO 目标 50-90）  整体 {:.1f}  饱和 {:.2f}%", c_mean[0], g_mean[0], sat_pct);
    //   for (const auto & a : armors) {
    //     tools::logger()->info("  {} {}  置信度 {:.3f}  中心({:.0f},{:.0f})  ratio {:.2f} side_ratio {:.2f}",
    //                           auto_aim::COLORS[static_cast<int>(a.color)], auto_aim::ARMOR_NAMES[static_cast<int>(a.name)], a.confidence, a.center.x,
    //                           a.center.y, a.ratio, a.side_ratio);
    //   }
    //   if (armors.empty()) tools::logger()->warn("  没识别到。检查：灯条是否通电发亮 / 曝光是否过暗 / 距离是否太远");
    // }
    (void)print_once;  // 变量保留，避免 -Wunused 警告；恢复上面那段就能直接用

    // ---- 画 ROI 边界，框外压暗 ----
    if (use_roi) {
      cv::Rect r = roi_rect & cv::Rect(0, 0, show.cols, show.rows);
      if (r.width > 0 && r.height > 0) {
        cv::Mat mask = cv::Mat::zeros(show.size(), CV_8UC1);
        cv::rectangle(mask, r, 255, cv::FILLED);
        cv::Mat dark = show * 0.45;
        dark.copyTo(show, 255 - mask);
        cv::rectangle(show, r, {0, 200, 255}, 2);
        tools::draw_text(show, "ROI", {r.x + 8, r.y + 30}, {0, 200, 255}, 0.8, 2);
        for (const auto & a : armors)
          if (!r.contains(cv::Point(a.center)))
            tools::draw_text(show, "armor outside ROI", {r.x, std::max(20, r.y - 12)}, {0, 0, 255}, 0.7, 2);
      }
    }

    // ---- 图像显示已关闭 ----
    // 原来是 cv::imshow(WIN, show)。现在画面改从 rerun 里看（见下面的 /camera），
    // 好处是画面、波形、数值表在同一个窗口里，不用在两个窗口间来回切。
    //
    // ★连带影响：cv::waitKey 只有在有 OpenCV 窗口时才能收到键盘事件。
    //   窗口关掉后 q/t/空格/p 全部失效，所以：
    //     退出   -> 在终端按 Ctrl+C
    //     切换识别器 -> 用命令行参数：./detect_test -y 用 YOLO，不加则传统CV
    //   这里保留 waitKey(1) 是必须的：它同时承担「每帧让出 1ms CPU」的作用，
    //   去掉会变成满速空转，把一个核吃满。
    // cv::imshow(WIN, show);
    //
    // ★这里原来是 cv::waitKey(1)，已换成纯 sleep —— 因为它会让程序崩溃。
    //   实测报错：
    //       qt.qpa.xcb: could not connect to display
    //       This application failed to start because no Qt platform plugin
    //       could be initialized.
    //   退出码 134 (SIGABRT)。
    //
    //   原因：本机 OpenCV 是带 Qt 后端编译的，cv::waitKey 即使不配合 imshow，
    //   自己也要初始化 Qt 窗口系统。没有可用 DISPLAY 时（SSH 会话、
    //   或者 X 授权没配好）直接 abort。
    //   注释掉 imshow 却留着 waitKey，等于保留了 GUI 依赖却没有 GUI 用途。
    //
    //   waitKey(1) 原本承担「每帧让出 1ms CPU」的作用，这个必须保留 ——
    //   去掉会变成满速空转吃满一个核。sleep_for 语义更准确，
    //   而且彻底摆脱 GUI 依赖，纯终端/SSH 下也能跑。
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

#ifdef USE_RERUN
    // ---- 把带标注的画面发到 rerun（JPEG 压缩）----
    // show 上面已经画好了装甲板框、角点、roll 角标注和顶部状态栏，
    // 所以这里发的是「和原来 imshow 一模一样的画面」，UI 一个不少。
    //
    // 限流判断已经在循环开头的 draw_this_frame 做过了 —— 这里不能再判一次，
    // 否则两处 static 计时器互相错开，会变成隔两次才真正发出一张。
    if (draw_this_frame) {
      cv::Mat small;
      cv::resize(show, small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

      // JPEG 编码。cv::imencode 吃的是 BGR（OpenCV 原生顺序），
      // 所以这里不需要 cvtColor —— 省一次全图拷贝。
      // 而且 JPEG 里记录了正确的颜色顺序，Viewer 解码后颜色是对的，
      // 不会出现之前发原始 RGB 时那种红蓝互换问题。
      std::vector<uchar> jpeg_buf;
      std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
      if (cv::imencode(".jpg", small, jpeg_buf, params)) {
        rec.log(
          "camera",
          rerun::EncodedImage::from_bytes(
            rerun::borrow(jpeg_buf.data(), jpeg_buf.size()), rerun::MediaType::jpeg()));
      }
    }
#endif

    // ---- 键盘响应已关闭（无窗口收不到按键）----
    // if (key == 'q' || key == 27) break;
    // if (key == 't') {
    //   use_traditional = !use_traditional;
    //   frame_count = detected_frames = 0;  // 切换后重新统计命中率
    //   tstate = "lost";
    //   detect_cnt = lost_cnt = miss_streak = worst_miss = tracking_frames = lost_events = 0;
    // }
    // if (key == ' ') {
    //   auto now = std::chrono::system_clock::now();
    //   auto tt = std::chrono::system_clock::to_time_t(now);
    //   char buf[32];
    //   std::strftime(buf, sizeof(buf), "%H%M%S", std::localtime(&tt));
    //   auto tag = fmt::format("shot_{}_{}", buf, use_traditional ? "trad" : "yolo");
    //   cv::imwrite(fmt::format("{}_marked.jpg", tag), show);
    //   cv::imwrite(fmt::format("{}_raw.jpg", tag), img);
    // }
    // if (key == 'p') print_once = true;
  }

  // ---- 退出统计：已关闭 ----
  // 这些累计值在 rerun 波形里都能直接看出来：
  //   命中率     -> 「识别状态」波形在 0 以上的占比
  //   最长漏检   -> 波形上最宽的那段缺口
  //   掉 lost 次数 -> 波形从 3 跌到 0 的次数
  // tools::logger()->info("退出。共 {} 帧，{} 帧有识别结果（{:.1f}%）", frame_count, detected_frames,
  //                       frame_count ? 100.0 * detected_frames / frame_count : 0.0);
  // tools::logger()->info("其中 {} 帧处于 tracking/temp_lost（真正在瞄）= {:.1f}%", tracking_frames,
  //                       frame_count ? 100.0 * tracking_frames / frame_count : 0.0);
  // tools::logger()->info("最长连续漏检 {} 帧（超过 max_temp_lost_count={} 就会真丢跟踪）", worst_miss, max_temp_lost_count);
  // tools::logger()->info("掉回 lost 共 {} 次", lost_events);
  return 0;
}
