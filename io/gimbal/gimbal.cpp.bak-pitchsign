#include <chrono>
#include <string>
#include <fmt/core.h>
#include "gimbal.hpp"

#include <opencv2/opencv.hpp>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
  //                构造函数初始化
  Gimbal::Gimbal(const std::string & config_path)
  {
    auto yaml = tools::load(config_path);
    auto com_port = tools::read<std::string>(yaml, "com_port");

    this->gimbal_yaw2vision = tools::read<int>(yaml, "gimbal_y1");
    this->gimbal_pitch2vision = tools::read<int>(yaml, "gimbal_p2");
    this->gimbal_roll2vision = tools::read<int>(yaml, "gimbal_r3");

    try
    {
      serial_.setPort(com_port);
      serial_.setBaudrate(460800);
      auto timeout = serial::Timeout::simpleTimeout(2);
      serial_.setTimeout(timeout);
      serial_.open();
    }
    catch (const std::exception & e)
    {
      tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
      exit(1);
    }
//           再开一个人手去跑这个函数 读串口数据
    thread_ = std::thread(&Gimbal::read_thread, this);

    queue_.pop();
    tools::logger()->info("[Gimbal] First q received.");
  }

  Gimbal::~Gimbal()
  {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
    serial_.close();
  }

  GimbalMode Gimbal::mode() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
  }

  GimbalState Gimbal::state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
  Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
  {
    while (true)
    {
      // 1. 阻塞等待并弹出队列中最老的一帧数据
      auto [q_a, t_a] = queue_.pop();

      // 2. 防死锁核心：如果弹出后队列空了，绝对不能再去调用 front()！
      // 否则 front() 会永久阻塞等待下一个数据导致画面卡死。
      // 此时直接返回当前唯一可用的数据即可。
      // if (queue_.empty()) {
      //   return q_a;
      // }

      // 3. 此时队列非空，可以安全地偷看（不弹出）下一个数据，绝不会阻塞
      auto [q_b, t_b] = queue_.front();

      // 4. 如果请求时间比插值终点还要晚，说明 q_a 已经没有保留价值了
      // 丢弃 q_a，在下一轮循环中让 q_b 成为新的起点
      // if (t > t_b) {
      //   continue;
      // }

      // 5. 正常的时间戳线性插值
      auto t_ab = tools::delta_time(t_a, t_b);
      auto t_ac = tools::delta_time(t_a, t);
      auto k = t_ac / t_ab;
      Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

      if (t < t_a) return q_c;

      // 此时 t 一定在 (t_a, t_b] 区间内
      if (!(t_a < t && t <= t_b)) continue;

      return q_c;
    }
  }
//                     发送函数             

  void Gimbal::sb_send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc, float target_x, float target_y,
    uint8_t target_name)
  {
    sb_tx_data_.mode = control ? (fire ? 2 : 1) : 0;
    sb_tx_data_.yaw = yaw;
    sb_tx_data_.yaw_vel = yaw_vel;
    sb_tx_data_.yaw_acc = yaw_acc;
    sb_tx_data_.pitch = pitch;
    sb_tx_data_.pitch_vel = pitch_vel;
    sb_tx_data_.pitch_acc = pitch_acc;

    // target_x / target_y / target_name 已从协议中移除(见 gimbal.hpp 的说明):
    // 电控 29 字节协议里没有这三个字段的位置。形参保留是为了不改动调用点
    // (rb_auto_aim_debug.cpp:106), 这里显式忽略, 避免 -Wunused-parameter。
    (void)target_x;
    (void)target_y;
    (void)target_name;

    // CRC16 覆盖 head 到 pitch_acc 共 26 字节, 不含 crc16 自身与 end。
    // 与 tools::get_crc16 的约定一致: "len不包括crc16"。
    //
    // 电控当前把 check_crc16 注释掉了, 所以填错也能跑 —— 正因如此才更要填对:
    // 否则等哪天电控放开校验, 又要回头查一遍这条链路。
    sb_tx_data_.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t *>(&sb_tx_data_), 26);

    // ---- 发送侧链路诊断 ----
    //
    // 为什么需要它: 云台"不跟随"时, 光看视觉端日志分不清是哪一环断了 ——
    // 识别没出目标? planner 没给 control? 还是包发出去了电控不认?
    // 这三种情况在原来的日志里长得一模一样(都是"程序在正常跑")。
    //
    // 电控的采纳条件在 Core/Src/stm32f4xx_it.c:423-444, 全部满足才会写 Target_Angle:
    //     Mini_PC_rx_buf[0]==0x66 && Mini_PC_rx_buf[28]==0x11   帧格式
    //     YK_Mode == PLAYER_MODE || YK_Mode == SHOOT_MODE       遥控档位
    //     request.zimiao_status                                 自瞄使能(右键或 yaogan.v < -600)
    //     !request.buff_status                                  非打符模式
    //     SuperPower.mode == 1 || == 2                          ★视觉端必须发 1 或 2
    // 前四条在电控侧, 这里看不到; 最后一条就是本包的 mode 字段, 由 plan.control 决定。
    //
    // mode 的来源链: 识别到装甲板 -> tracker 确认 -> planner 返回 control=true -> mode=1
    // 任何一环没成, mode 就是 0, 电控收下包也不会动云台 —— 这正是"抖一下就没了"的成因:
    // 偶尔某帧识别成功发了个 1, 云台刚要动, 下一帧又变回 0。
    {
      // TX_MODE_WATCH
      static uint8_t last_mode = 255;
      static auto last_report = std::chrono::steady_clock::now();
      static int cnt_total = 0, cnt_ctrl = 0;

      cnt_total++;
      if (sb_tx_data_.mode != 0) cnt_ctrl++;

      // mode 跳变时立刻打一条, 这是最有价值的信号
      if (sb_tx_data_.mode != last_mode)
      {
        const auto * p = reinterpret_cast<const uint8_t *>(&sb_tx_data_);
        tools::logger()->info(
          "[Gimbal] mode {} -> {} ({})  yaw={:.2f}deg pitch={:.2f}deg  首字节={:02X} 末字节={:02X}", last_mode == 255 ? 0 : last_mode,
          sb_tx_data_.mode,
          sb_tx_data_.mode == 0 ? "不控制:视觉没给出目标" : (sb_tx_data_.mode == 1 ? "控云台不开火" : "控云台并开火"),
          sb_tx_data_.yaw * 57.2958f, sb_tx_data_.pitch * 57.2958f, p[0], p[sizeof(sb_tx_data_) - 1]);
        last_mode = sb_tx_data_.mode;
      }

      // 每 2 秒报一次占比, 用来判断"识别是否稳定"
      auto now = std::chrono::steady_clock::now();
      if (now - last_report > std::chrono::seconds(2))
      {
        last_report = now;
        double pct = cnt_total ? 100.0 * cnt_ctrl / cnt_total : 0.0;
        if (cnt_ctrl == 0)
          tools::logger()->warn("[Gimbal] 近2秒发出 {} 帧, mode 全为 0 -> 视觉没锁定目标, 云台不会动", cnt_total);
        else
          tools::logger()->info("[Gimbal] 近2秒发出 {} 帧, 其中 mode!=0 占 {:.0f}% ({} 帧)", cnt_total, pct, cnt_ctrl);
        cnt_total = cnt_ctrl = 0;
      }
    }

    try
    {
      serial_.write(reinterpret_cast<const uint8_t *>(&sb_tx_data_), sizeof(sb_tx_data_));
    }
    catch (const std::exception & e)
    {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  }
//                     接收电控字节包
  bool Gimbal::read(uint8_t * buffer, size_t size)
  {
    try
    {
      return serial_.read(buffer, size) == size;
    }
    catch (const std::exception & e)
    {
      tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
      return false;
    }
  }
//                    独立线程，一直接收电控发来的数据
  void Gimbal::read_thread()
  {
    tools::logger()->info("[Gimbal] read_thread started.");
    int error_count = 0;

    while (!quit_)
    {
      if (error_count > 50000)
      {
        error_count = 0;
        tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
        reconnect();
        continue;
      }

      // 1. 一次性读取完整的一帧数据（基于 GimbalToVision 结构体的大小）
      if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_)))
      {
        error_count++;
        continue;
      }

      // 2. 检查帧头是否正确
      if (rx_data_.head[0] != 0x5a || rx_data_.head[1] != 0x53)
      {
        // 如果帧头不对，说明数据由于丢包等原因发生了错位（失步）
        // 此时必须立刻清空底层的接收缓冲区，把残留的错位数据全部丢弃，以便下一次能读到全新的完整帧
        serial_.flushInput();
        error_count++;
        // 可选：添加一条 debug 日志观察失步频率
        // tools::logger()->debug("[Gimbal] 帧头错位，已清空缓冲区");
        continue;
      }

      // 3. 记录成功接收到有效帧的时间戳
      auto t = std::chrono::steady_clock::now();

      // 4. 检查 CRC 校验和
      if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_)))
      {
        // tools::logger()->debug("[Gimbal] CRC16 check failed.");
        error_count++;
        continue;
      }

      // --- 以下为原本的数据处理逻辑，保持不变 ---
      error_count = 0;
      Eigen::Quaterniond q_(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
      auto ypr = tools::eulers(q_, 2, 1, 0);

      float yaw = ypr[abs(gimbal_yaw2vision) - 1];
      float pitch = ypr[abs(gimbal_pitch2vision) - 1];
      float roll = ypr[abs(gimbal_roll2vision) - 1];

      yaw = gimbal_yaw2vision > 0 ? yaw : -yaw;
      pitch = gimbal_pitch2vision > 0 ? pitch : -pitch;
      roll = gimbal_roll2vision > 0 ? roll : -roll;

      Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *    // 绕Z轴旋转yaw
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *  // 绕Y轴旋转pitch
                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());    // 绕X轴旋转roll

      queue_.push({q, t});

      std::lock_guard<std::mutex> lock(mutex_);
      auto ypr_now = tools::eulers(q, 2, 1, 0);
      state_.yaw = ypr_now[0] * 57.3;      //解算出的度数
      state_.pitch = ypr_now[1] * 57.3;

      // 收到模，颜色，弹速
      state_.mode = rx_data_.mode;
      state_.enemy_color = !rx_data_.color;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;
      // rx_data_.mode = 2;
      //

      switch (rx_data_.mode)
      {
        case 0:
          mode_ = GimbalMode::IDLE;
          break;
        case 1:
          mode_ = GimbalMode::AUTO_AIM;
          break;
        case 2:
          mode_ = GimbalMode::SMALL_BUFF;
          break;
        case 3:
          mode_ = GimbalMode::BIG_BUFF;
          break;
        case 4:
          mode_ = GimbalMode::LONG_FOCAL_LENGTH;
          break;
        default:
          mode_ = GimbalMode::IDLE;
          tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
          break;
      }
    }

    tools::logger()->info("[Gimbal] read_thread stopped.");
  }
// 重连
  void Gimbal::reconnect()
  {
    int max_retry_count = 10;
    for (int i = 0; i < max_retry_count && !quit_; ++i)
    {
      tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
      try
      {
        serial_.close();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      catch (...)
      {
      }

      try
      {
        serial_.open();  // 尝试重新打开
        queue_.clear();
        tools::logger()->info("[Gimbal] Reconnected serial successfully.");
        break;
      }
      catch (const std::exception & e)
      {
        tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

}  // namespace io