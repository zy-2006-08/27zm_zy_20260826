#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | ../configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | ../assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help"))
  {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;
//读视频
  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  // ===== 五个流水线对象，构造时各自去读同一个 yaml =====
  // 注意：tools/yaml.hpp 读不到 key 时直接 exit(1)，不抛异常。
  // 所以少写一个 key 的表现是"程序莫名死掉 + 一行日志"，不是崩栈。
  auto_aim::YOLO yolo(config_path);                   // 神经网络识别（本离线程序走的是这条）
  // auto_aim::Detector traditional(config_path, true);  // 传统CV识别（这里构造了但下面没调，见 :86）
  auto_aim::Solver solver(config_path);               // 坐标变换 + PnP：像素 -> 世界坐标
  auto_aim::Tracker tracker(config_path, &solver);    // 关联 + EKF 跟踪：装甲板 -> 整车目标
  auto_aim::Aimer aimer(config_path);                 // 火控：目标 -> yaw/pitch/开火

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  // auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++)
  {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++)
  {
    if (end_index > 0 && frame_count > end_index) break;
    // auto inshow_start = std::chrono::steady_clock::now();
    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));
    //取姿态
    solver.set_R_gimbal2world({w, x, y, z});                              //四元数
    // 识别
    auto yolo_start = std::chrono::steady_clock::now();              //神经网络框出啊
    auto armors = yolo.detect(img, frame_count);
    // auto traditional_start = std::chrono::steady_clock::now();
    // auto armors = traditional.detect(img, frame_count);
    //跟踪
    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.test_track(armors, timestamp);

    //火控
    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    if (!targets.empty() && aimer.debug_aim_point.valid && std::abs(command.yaw - last_command.yaw) * 57.3 < 2) command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出（日志）
    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count, tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3, tools::delta_time(finish, aimer_start) * 1e3);
      //视频ui
    tools::draw_text(
      img, fmt::format("command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3, command.pitch * 57.3, command.shoot), {10, 60},
      {154, 50, 205});

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(img, fmt::format("gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]), {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty())
    {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    
    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty())
    {
      auto target = targets.front();

      if (last_t == -1)
      {
        // last_target = target;
        last_t = t;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list)
      {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      //波形
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];                  // 旋转中心 世界系 x，单位米
      data["vx"] = x[1];                 // 旋转中心 x 方向速度，m/s
      data["y"] = x[2];                  // 旋转中心 世界系 y，单位米
      data["vy"] = x[3];                 // 旋转中心 y 方向速度，m/s
      data["z"] = x[4];                  // 旋转中心 世界系 z（高度），单位米
      data["vz"] = x[5];                 // z 方向速度，m/s
      data["a"] = x[6] * 57.3;           // = 状态 yaw，整车朝向。乘 57.3 是弧度转度，方便看曲线
      data["w"] = x[7];                  // = 状态 vyaw，整车旋转角速度，rad/s（未转度）
      data["r"] = x[8];                  // 基础半径：旋转中心到 0、2 号板的距离，单位米
      data["l"] = x[9];                  // = 状态 r_，半径补偿量。1、3 号板的半径 = x[8]+x[9]，单位米
      data["h"] = x[10];                 // = 状态 z_，高度补偿量。1、3 号板的高度 = x[4]+x[10]，单位米
      data["last_id"] = target.last_id;  // 上一次匹配上的装甲板编号，用来对照 r/r_ 该取哪个

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(10);
    if (key == 'q') break;

    //  tools::logger()->info(
    //     "imshow : {:.1f}ms",  tools::delta_time(std::chrono::steady_clock::now(), inshow_start) * 1e3);
  }

  return 0;
}