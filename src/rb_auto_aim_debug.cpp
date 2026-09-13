#include <fmt/core.h>  //字符串格式化

#include <atomic>
#include <chrono>              // C++ 的时间库
#include <nlohmann/json.hpp>   //JSON 库。JSON 就是一种文本格式的数据打包方式，长这样：{"yaw": 12.3, "pitch": 4.5}。这里用来把调试数据打包，发给画曲线的工具（类似上位机示波器）。
#include <opencv2/opencv.hpp>  //OpenCV，图像处理库。OpenCV 是什么：一个装满了"图像操作函数"的工具箱。读图、画线、画圆、缩放、显示窗口，全靠它
#include <thread>              //线程库

//#include <xxx> 是"系统/第三方库"，#include "xxx" 是"本工程自己的文件"。

#include "io/camera.hpp"                       // 第 9 行：相机驱动，负责从大恒相机取图
#include "io/gimbal/gimbal.hpp"                // 第 10 行：串口通信，跟你的 C 板收发数据
#include "tasks/auto_aim/detector.hpp"         // 第 12 行：传统CV装甲板识别器
#include "tasks/auto_aim/planner/planner.hpp"  // 第 13 行：弹道规划 + 火控决策
#include "tasks/auto_aim/shooter.hpp"          // 第 14 行：发射相关（本文件没用到）
#include "tasks/auto_aim/solver.hpp"           // 第 15 行：坐标解算（像素 → 三维世界坐标）
#include "tasks/auto_aim/tracker.hpp"          // 第 16 行：目标跟踪 + 卡尔曼滤波
#include "tasks/auto_aim/yolo.hpp"             // 第 17 行：神经网络识别器
#include "tools/exiter.hpp"                    // 第 18 行：捕获 Ctrl+C，让程序优雅退出
#include "tools/img_tools.hpp"                 // 第 19 行：画字、画点的小工具
#include "tools/logger.hpp"                    // 第 20 行：打日志（等于串口 printf）
#include "tools/math_tools.hpp"                // 第 21 行：数学工具（四元数转欧拉角、算时间差）
#include "tools/plotter.hpp"                   // 第 22 行：把数据发出去画曲线
#include "tools/thread_safe_queue.hpp"         // 第 23 行：线程安全队列

using namespace std::chrono_literals;  //这行让你能直接写 5ms、3ms 这种字面量
using namespace tools;
//参数名 | 默认值 | 说明文字
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明，，，，，，，，}"
  "{@config-path   | ../configs/sb_long.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);  //把上面那张 keys 表和命令行参数一起交给 OpenCV，让它帮忙解析。cv:: 前缀表示这是 OpenCV 命名空间里的东西。
  auto config_path = cli.get<std::string>(0);   // 取出第 0 个位置参数（也就是 @config-path），当成字符串
  if (cli.has("help") || config_path.empty())   //。如果用户敲了 --help，或者配置路径是空的，就打印那张参数说明表然后退出。
  {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);  //创建云台通信对象，这是你最熟的部分。它内部：
                                   // 打开串口（设备名和波特率从 yaml 读）
                                   // 开一条后台接收线程，不停地从串口读 29 字节的包，校验帧头 0x5A 0x53 和 CRC16，解出 mode、color、四元数 q[4]、bullet_speed、bullet_count
                                   // 把每一帧的四元数 + 收包时间戳压进一个长度 1000 的队列，供后面按时间查询
                                   // ⚠️ 它的构造函数末尾会等第一帧数据，所以 C 板不通电 / 串口没插，这个程序会卡死在第 45 行。这是最常见的"程序没反应"原因。
  io::Camera camera(config_path);  //打开相机。会自动探测是大恒还是海康，按 yaml 里的曝光、增益、gamma 去配置传感器。

  auto_aim::YOLO yolo(config_path, true);  //true 通常是 "debug 模式"开关，会让它多画点东西/多打日志
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);             //坐标解算器，这是视觉最核心也最难理解的一块。它干的事：
                                                    // 已知装甲板在图像上的 4 个角点（像素坐标），已知装甲板的真实物理尺寸（比如 135mm × 55mm），已知相机的内参矩阵和畸变系数（标定标出来的），就能反算出"这块装甲板在相机前方多远、偏左偏右多少、朝向如何"。这个过程叫 PnP（Perspective-n-Point）。
  auto_aim::Tracker tracker(config_path, &solver);  //创建跟踪器，并把 solver 的地址交给它（&solver 取地址，跟 C 一样），因为跟踪器需要用解算器把装甲板转成世界坐标。
                                                    // 跟踪器干什么：识别器每帧只告诉你"这一帧看到了几块板，各在哪"。跟踪器负责把连续多帧的观测串成"同一辆车的运动轨迹"，并用 EKF（扩展卡尔曼滤波） 估计出这辆车的完整状态：中心位置、速度、自转角度、自转角速度、半径。
  tracker.set_gimbal(&gimbal);                      //把云台对象也给跟踪器，它需要读云台状态（比如敌方颜色，好过滤掉自己队友的板）。

  auto_aim::Planner planner(config_path);  //创建规划器。它干两件事：
                                           // 弹道解算：子弹飞行有时间、有重力下坠、有空气阻力。所以不能瞄"目标现在在哪"，要瞄"子弹飞到那儿时目标会在哪"。这里用 MPC（模型预测控制）解出一条云台运动轨迹。
                                           // 火控决策：判断当前枪口是不是对准了，对准了才置 fire = true。

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);  //这三个设定合起来是什么意思：这不是一个"缓冲队列"，而是一个只保存最新值的信箱。视觉线程每算出一个目标就往里塞，塞满了就把旧的挤掉。规划线程随时来取，永远拿到最新的那一个，绝不会拿到几百毫秒前的陈旧数据。
  target_queue.push(std::nullopt);                                                //必须有这行

  // ============================================================================
  // 【双线程结构，这是上场主程序和离线测试最大的不同】
  // 离线的 auto_aim_test 是单线程 for 循环：读一帧、算一帧、显示一帧。
  // 这里拆成两条独立的线程，靠上面那个容量为 1 的 target_queue 传递最新目标：
  //
  //   视觉线程（下面的 while(!exiter.exit()) 主循环）
  //     节奏 = 相机帧率，取到图才干活。做识别 -> 解算 -> 跟踪，把 Target 塞进队列。
  //
  //   plan_thread（本线程）
  //     节奏 = 固定 5ms（200Hz），与相机无关。每轮都重新做弹道规划并发送云台指令。
  //
  // 【为什么必须拆开】相机大约 100~200fp
  // s，但云台控制需要更高、更稳定的更新率。
  // 如果规划跟着视觉走，一旦某帧识别慢了或丢帧，云台指令就会跟着卡顿，
  // 表现为准星一顿一顿。拆开后即使**没有新的一帧图像**，本线程也会用 EKF 把目标
  // 往前预测到当前时刻再重新规划，所以指令始终是平滑连续的。
  // 电控类比：视觉线程像"传感器采样任务"，plan_thread 像固定周期的控制中断，
  // 两者用一个共享变量解耦——只是这里的共享变量做了线程安全封装。
  // ============================================================================
  std::atomic<bool> quit = false;  //线程退出标志。用 atomic 因为主线程写它、plan_thread 读它。等于 volatile bool，但保证了原子性。
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();  //记下线程启动时刻，作为"零点"。后面第 112 行画曲线时用 当前时刻 - t0 得到相对秒数当横轴。
    uint16_t last_bullet_count = 0;              //记住上一轮的子弹计数，用来检测"是不是刚打出去一发"

    while (!quit)
    {
      //敌人在哪
      auto target = target_queue.front();  //从信箱里"看一眼"最新目标。注意是 front() 不是 pop()——只看不取走。所以只要视觉线程不塞新的，这里每 5ms 都会拿到同一个目标，然后靠后面的预测把它推到当前时刻。
      //我的炮口现在朝哪、弹速多少                                     // target 的类型是 std::optional<Target>，可能有值也可能没值。
      auto gs = gimbal.state();            //读一份云台当前状态的快照（内部加锁拷贝，所以是安全的）。gs 里有：yaw/pitch（单位度，注意，gimbal.cpp 里已经乘过 57.3）、bullet_speed、bullet_count、mode、enemy_color。

      //MPC预测以及+自家火控
      auto plan = planner.plan(target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);  //核心一行。把目标、弹速、当前云台 yaw、开火策略交给规划器，得到一个 Plan 结构体。
                                                                                                                       // 看 planner.hpp 里这个函数的实现，它内部依次做：
                                                                                                                       // 如果 target 没值，直接返回 {false}（control = false，其余全 0）——即"不控制云台"
                                                                                                                       // 根据目标自转速度选一个延迟时间（转得快用 high_speed_delay_time_，慢用 low_speed_delay_time_）。这个延迟代表"从现在到子弹真正出膛的总延时"
                                                                                                                       // target->predict(future) —— 用 EKF 把目标状态推进到未来那个时刻
                                                                                                                       // 按策略 rbSuppressiveFire 走 rbplan()，算出云台目标角和是否开火

      // 1. 设置默认值
      uint8_t name = 0;
      float tx = 0.0f;
      float ty = 0.0f;

      // 2. 只有在 target 有值时才去提取数据
      // if (target.has_value())
      // {
      //   name = static_cast<uint8_t>(target->name) + 1;
      //   tx = target->ekf_x()[0];
      //   ty = target->ekf_x()[2];

      //   // tools::logger()->info("{},{},{}", name,tx,ty);
      // }
      // 发串口。这是整个视觉程序唯一的输出。
      gimbal.sb_send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc, tx, ty, name);
      // 边沿检测，跟你检测按键上升沿一个套路：子弹累计计数变大了，说明刚打出去一发。fired 只用于画曲线，方便你对比"下令开火"和"实际打出"之间的延迟。
      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;
      data["q2yaw"] = gs.q2yaw;
      data["q2pitch"] = gs.pitch;

      if (target.has_value())
      {
        data["plan_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;
        data["plan_yaw"] = plan.yaw / CV_PI * 180.;
        data["plan_yaw_vel"] = plan.yaw_vel;
        data["plan_yaw_acc"] = plan.yaw_acc;

        data["plan_pitch"] = plan.pitch * 57.3;
        data["plan_pitch_vel"] = plan.pitch_vel;
        data["plan_pitch_acc"] = plan.pitch_acc;

        data["fire"] = plan.fire ? 1 : 0;
        data["fired"] = fired ? 1 : 0;

        data["target_yaw"] = plan.target_yaw;
        data["target_pitch"] = plan.target_pitch;
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
        data["tower_h1"] = target->tower_armor_hs[0];
        data["tower_h2"] = target->tower_armor_hs[1];
        data["tower_h3"] = target->tower_armor_hs[2];
        data["tower_armor_h"] = target->tower_armor_h;

        const auto ekf_satic = target->ekf_x();
        data["ekf_x"] = ekf_satic(0);
        data["ekf_vx"] = ekf_satic(1);
        data["ekf_y"] = ekf_satic(2);
        data["ekf_vy"] = ekf_satic(3);
        data["ekf_z"] = ekf_satic(4);
        data["ekf_vz"] = ekf_satic(5);
        data["ekf_yaw"] = ekf_satic(6) * 57.3;
        data["ekf_vyaw"] = ekf_satic(7) * 57.3;
        data["ekf_r"] = ekf_satic(8);
      }

      plotter.plot(data);

      std::this_thread::sleep_for(5ms);
    }
  });

  cv::Mat img;  //*cv::Mat 是 OpenCV 最核心的类型：一张图。*
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t;

  // 视觉线程（主线程）。节奏由相机决定：camera.read 阻塞等下一帧。
  // 与离线测试的对应关系：这里的 detector.detect 对应那边的 yolo.detect，
  // tracker.track 对应那边的 test_track，之后不走 Aimer 而是把 Target 交给 plan_thread。
  while (!exiter.exit())  //主循环，直到你按 Ctrl+C。
  {
    camera.read(img, t);  //取一帧图。这行会阻塞——等到相机真的吐出一帧才返回。所以整个循环的节奏由相机帧率决定。
    // 取 t-3ms 时刻的云台姿态：图像有曝光和传输延迟，所以要用"稍早一点"的姿态才对得上这帧画面。
    // 这 3ms 是实测的经验值，姿态队列会按时间戳插值（见 io/gimbal 的 q()）。
    auto q = gimbal.q(t - 3ms);  //取 t 减 3ms 时刻的云台姿态四元数

    // 与离线一样，必须先喂姿态再做识别，顺序不能反
    solver.set_R_gimbal2world(q);  //把这一帧的云台姿态喂给解算器。顺序绝对不能反——必须在 detect 之前。
                                   // 这行的意义：PnP 只能算出"装甲板相对相机的位置"。但云台在转，相机跟着转，所以相机坐标系是动的。要把结果转到固定的世界坐标系（这样车的运动才是连续可预测的，EKF 才能工作），就需要知道"相机现在朝哪"。这个四元数就是那个旋转关系。
                                   // R_gimbal2world = "从云台坐标系到世界坐标系的旋转矩阵"。命名规范是 R_A2B 表示"把 A 系的向量转到 B 系"。
                                   // 为什么必须在 detect 之前：detect → 内部调 solver 做 PnP → solver 用当前设的 R 转世界系。如果先 detect 再设 R，用的就是上一帧的姿态，云台转得快时误差巨大。
    // ★上场跑的是传统CV，不是 YOLO。上面构造的 yolo 对象在本文件中从未被调用。
    auto armors = detector.detect(img);       //识别装甲板。返回 std::list<Armor> —— 这一帧找到的所有装甲板的链表。
    auto targets = tracker.track(armors, t);  //跟踪 + EKF 更新。输入这一帧的装甲板列表和时间戳，输出 std::list<Target> —— 当前正在跟踪的车。
    // recor.record(img, q, t);

    auto now = std::chrono::steady_clock::now();
    double fps = 1. / tools::delta_time(now, last_t);
    // tools::draw_text(img, "fps: "+std::to_string(fps), cv::Point(40, 130));
    last_t = now;
    tools::logger()->info("fps:: {:.2f}", fps);  //并打印帧率。

    auto ypr = tools::eulers(q, 2, 1, 0);  //四元数转欧拉角

    float yaw_deg = ypr[0] * 180.0 / M_PI;  //弧度转度
    float pitch_deg = ypr[1] * 180.0 / M_PI;
    float roll_deg = ypr[2] * 180.0 / M_PI;
    // std::cout << "DK_Yaw: " << yaw_deg << std::endl;
    // std::cout << "DK_Pitch: " << pitch_deg << std::endl;
    if (yaw_deg == 0 || pitch_deg == 0) std::cout << "shit" << std::endl;
    //     在图上写字，方便你在窗口里直接看到云台角度。
    // fmt::format("rb_Yaw {:.2f}", yaw_deg)：生成字符串，比如 "rb_Yaw 12.35"。
    // {40, 40}：位置，图像坐标 (x=40, y=40)。图像坐标原点在左上角，x 往右，y 往下（不是数学里的左下原点，这点常搞错）。所以 (40,40) 是左上角附近。第二行 y=80，往下 40 像素。
    // {0, 128, 255}：颜色，BGR 顺序（不是 RGB）。B=0, G=128, R=255 → 橙色。下一行 {0,255,255} → B=0,G=255,R=255 → 黄色。
    tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
    tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});
    // std::cout << "Roll: " << roll_deg << std::endl;
    //画 EKF 调试信息
      if (!targets.empty())
      {
      target_queue.push(targets.front());//有目标的话，把第一个目标塞进信箱交给 plan_thread

      auto & target = targets.front();

      // 获取EKF状态向量
      Eigen::VectorXd ekf_x = target.getEKFXest();

      // 取出 x, y, z 组成整车旋转中心的三维世界坐标。注意下标是 0、2、4（因为位置速度交错排列）。
      // 注意这是旋转中心，不是装甲板。小陀螺时装甲板绕着这个中心转。
      Eigen::Vector3d center_world(ekf_x[0], ekf_x[2], ekf_x[4]);

      //*纯粹为了画一根"速度箭头"*：算出"如果按当前速度匀速走 0.5 秒，中心会到哪"，然后画一条从中心到那个点的线，线的长短方向就直观表示了速度。
      double dt = 0.5;            // 预测时间
      double scale_factor = 1.0; 
      Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
      Eigen::Vector3d pred_center = center_world + velocity * dt * scale_factor;

      // 同理画一根表示自转角速度的线：从中心往 z 方向（高度方向）伸出 w * 0.1 那么长。转得快线就长，方向（上/下）表示转向。
      double w = ekf_x[7];  // 判断小陀螺的关键量。经验上超过某个阈值就认为在转陀螺，要切换打法。
      Eigen::Vector3d v_yaw_axis_tvec = center_world;
      v_yaw_axis_tvec[2] += w * 0.1;  // 在y方向加上角速度的影响

      double speed_magnitude = std::sqrt(ekf_x[1] * ekf_x[1] + ekf_x[3] * ekf_x[3] + ekf_x[5] * ekf_x[5]);

      // std::cout << "角速度大小: " << w * 57.3 << " °/s" << std::endl;
      // 5. 输出速度大小到控制台
      // std::cout << "速度大小: " << speed_magnitude << " m/s" << std::endl;

      // 4. 将世界坐标转换为图像坐标
      // 这里需要将世界坐标转换为相机坐标，然后再投影到图像
      // 假设solver有一个将世界坐标转换为图像坐标的函数
      // 如果没有，你可以创建一个简单的投影函数

      // 方法1: 如果solver有直接投影点的函数
      // auto center_img = solver.reproject_point(center_world);
      // auto pred_point_img = solver.reproject_point(pred_center);
      // auto v_yaw_axis_point_img = solver.reproject_point(v_yaw_axis_tvec);

      // 重投影：把三维世界坐标算回图像上的像素坐标。
      // 这是 PnP 的逆运算。PnP 是"像素 → 三维"，重投影是"三维 → 像素"。做重投影的目的就是验证解算对不对：如果 EKF 估计准确，画出来的框会稳稳贴在真实装甲板上；如果框飘在旁边，就说明解算或跟踪有问题。这是调自瞄最直观的验证手段。
      auto center_img = solver.reproject_armor(center_world, 0.0, target.armor_type, target.name);
      auto pred_point_img = solver.reproject_armor(pred_center, 0.0, target.armor_type, target.name);
      auto v_yaw_axis_point_img = solver.reproject_armor(v_yaw_axis_tvec, 0.0, target.armor_type, target.name);

      //判空保护，必须有
      if (!center_img.empty() && !pred_point_img.empty())
      {
        // 绘制旋转中心
        cv::circle(img, center_img[0], 5, cv::Scalar(51, 153, 237), -1);

        // 绘制预测点（速度方向）
        cv::circle(img, pred_point_img[0], 8, cv::Scalar(0, 0, 255), -1);

        // 绘制速度方向线
        cv::line(img, center_img[0], pred_point_img[0], cv::Scalar(0, 255, 255), 2);

        // 绘制角速度方向线
        if (!v_yaw_axis_point_img.empty())
        {
          cv::line(img, center_img[0], v_yaw_axis_point_img[0], cv::Scalar(0, 255, 0), 2);
        }
      }
    }
    // *没有目标时，往信箱塞一个"空"*。
    // 这行很重要：如果不塞，plan_thread 会一直拿到最后一次的目标，目标丢了云台还在追着一个不存在的东西转。塞 nullopt 之后，planner.plan 第一行就 return {false}，云台停止控制。
    else
      target_queue.push(std::nullopt);

    if (!targets.empty())//  有shi
    {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list)
      {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {235, 206, 135});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }
    // 所以画面上的颜色约定是：
    // - 淡蓝框 = 所有装甲板的估计位置
    // - 红框 = 规划器决定要打的那一块
    // - 橙点 = 整车旋转中心
    // - 黄线 = 速度方向
    // - 绿线 = 自转角速度大小
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);//这行有两个作用，第二个很多人不知道：
    // 等 1 毫秒看有没有按键，返回按键的 ASCII 码（没按返回 -1）
    // 它同时负责刷新窗口。OpenCV 的 imshow 只是提交图像，真正的绘制发生在 waitKey 里。没有 waitKey，窗口会一片空白或卡死。 这是 OpenCV 的固定套路：imshow 后面必须跟 waitKey。
    if (key == 'q') break;
    if (key == 'r')
    {  //TUDO :右键手动更改
      io::GimbalState * g_demo = gimbal.set_state_();
      g_demo->mode = !g_demo->mode;
    }
    // if(key == 's') {
    //   stopkey = !stopkey;
    // }
  }
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();

  // 获取当前下位机发来的云台状态数据
  auto current_state = gimbal.state();

  // 最后发一包"停止控制"给电控。
  gimbal.sb_send(false, false, current_state.yaw / 57.3f, 0.0f, 0.0f, current_state.pitch / 57.3f, 0.0f, 0.0f, 0, 0, 0);

  return 0;
}