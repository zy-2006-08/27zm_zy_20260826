#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;
using namespace tools;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明，，，，，，，，}"
  "{@config-path   | ../configs/sb_long.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty())
  {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  tracker.set_gimbal(&gimbal);

  auto_aim::Planner planner(config_path);
  bool stopkey = false;

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

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
  // 【为什么必须拆开】相机大约 100~200fps，但云台控制需要更高、更稳定的更新率。
  // 如果规划跟着视觉走，一旦某帧识别慢了或丢帧，云台指令就会跟着卡顿，
  // 表现为准星一顿一顿。拆开后即使**没有新的一帧图像**，本线程也会用 EKF 把目标
  // 往前预测到当前时刻再重新规划，所以指令始终是平滑连续的。
  // 电控类比：视觉线程像"传感器采样任务"，plan_thread 像固定周期的控制中断，
  // 两者用一个共享变量解耦——只是这里的共享变量做了线程安全封装。
  // ============================================================================
  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit)
    {
      auto target = target_queue.front();
      auto gs = gimbal.state();

      //MPC预测以及+自家火控
      auto plan = planner.plan(target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);

      // 1. 设置默认值
      uint8_t name = 0;
      float tx = 0.0f;
      float ty = 0.0f;

      // 2. 只有在 target 有值时才去提取数据
      if (target.has_value())
      {
        name = static_cast<uint8_t>(target->name) + 1;
        tx = target->ekf_x()[0];
        ty = target->ekf_x()[2];

        // tools::logger()->info("{},{},{}", name,tx,ty);
      }

      gimbal.sb_send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc, tx, ty, name);

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

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t;

  // 视觉线程（主线程）。节奏由相机决定：camera.read 阻塞等下一帧。
  // 与离线测试的对应关系：这里的 detector.detect 对应那边的 yolo.detect，
  // tracker.track 对应那边的 test_track，之后不走 Aimer 而是把 Target 交给 plan_thread。
  while (!exiter.exit())
  {
    camera.read(img, t);
    // 取 t-3ms 时刻的云台姿态：图像有曝光和传输延迟，所以要用"稍早一点"的姿态才对得上这帧画面。
    // 这 3ms 是实测的经验值，姿态队列会按时间戳插值（见 io/gimbal 的 q()）。
    auto q = gimbal.q(t - 3ms);

    // 与离线一样，必须先喂姿态再做识别，顺序不能反
    solver.set_R_gimbal2world(q);
    // ★上场跑的是传统CV，不是 YOLO。上面构造的 yolo 对象在本文件中从未被调用。
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    // recor.record(img, q, t);

    auto now = std::chrono::steady_clock::now();
    double fps = 1. / tools::delta_time(now, last_t);
    // tools::draw_text(img, "fps: "+std::to_string(fps), cv::Point(40, 130));
    last_t = now;
    tools::logger()->info("fps:: {:.2f}", fps);

    auto ypr = tools::eulers(q, 2, 1, 0);

    float yaw_deg = ypr[0] * 180.0 / M_PI;
    float pitch_deg = ypr[1] * 180.0 / M_PI;
    float roll_deg = ypr[2] * 180.0 / M_PI;
    // std::cout << "DK_Yaw: " << yaw_deg << std::endl;
    // std::cout << "DK_Pitch: " << pitch_deg << std::endl;
    if (yaw_deg == 0 || pitch_deg == 0) std::cout << "shit" << std::endl;
    tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
    tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});
    // std::cout << "Roll: " << roll_deg << std::endl;

    if (!targets.empty())
    {
      target_queue.push(targets.front());

      auto & target = targets.front();

      // 获取EKF状态向量
      Eigen::VectorXd ekf_x = target.getEKFXest();

      // 1. 计算旋转中心的世界坐标
      // EKF状态: [x, vx, y, vy, z, vz, yaw, vyaw, r, r_, z_]  (定义见 target.cpp:51-54)
      //   r_ 是半径补偿量，z_ 是高度补偿量，都只对 4 板车的 1、3 号板生效。
      //   曲线里这两维仍沿用旧名 l 和 h，yaw/vyaw 则叫 a 和 w，看图时按此对照。
      // 旋转中心: (x, y, z) = (ekf_x[0], ekf_x[2], ekf_x[4])
      Eigen::Vector3d center_world(ekf_x[0], ekf_x[2], ekf_x[4]);

      // 2. 计算速度终点（预测0.5秒后的位置）
      double dt = 0.5;            // 预测时间
      double scale_factor = 1.0;  // 放大2倍
      Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
      Eigen::Vector3d pred_center = center_world + velocity * dt * scale_factor;

      // 3. 计算角速度方向终点
      double w = ekf_x[7];  // 角速度
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

      // 方法2: 使用reproject_armor函数（需要一个虚拟的装甲板）
      // 这里假设我们有一个虚拟装甲板用于投影
      auto center_img = solver.reproject_armor(center_world, 0.0, target.armor_type, target.name);
      auto pred_point_img = solver.reproject_armor(pred_center, 0.0, target.armor_type, target.name);
      auto v_yaw_axis_point_img = solver.reproject_armor(v_yaw_axis_tvec, 0.0, target.armor_type, target.name);

      // 5. 绘制速度和角速度方向
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

    else
      target_queue.push(std::nullopt);

    if (!targets.empty())
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

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
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

  // 发送当前数据（注意：由于 gimbal.cpp 中接收时乘了 57.3 转成了角度，发回下位机时需要除以 57.3 转回弧度）
  // 因为下位机没有发来速度和加速度数据，所以 vel 和 acc 继续填 0 即可
  gimbal.sb_send(false, false, current_state.yaw / 57.3f, 0.0f, 0.0f, current_state.pitch / 57.3f, 0.0f, 0.0f, 0, 0, 0);

  return 0;
}