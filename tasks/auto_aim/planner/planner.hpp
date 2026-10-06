#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
  constexpr double DT = 0.01;
  constexpr int HALF_HORIZON = 50;
  constexpr int HORIZON = HALF_HORIZON * 2;

  using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

  struct Plan
  {
    bool control;
    bool fire;
    float target_yaw;
    float target_pitch;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
  };

  class Planner
  {
    public:
    enum ShootStrategy
    {                     //开火策略
      Dynamics,           //动力学
      rbSuppressiveFire,  //旧火控,火力压制
      rbHero,             //英雄
      SB                  //哨兵
    };
    Eigen::Vector4d debug_xyza;
    double aim_target_yaw;
    Planner(const std::string & config_path);

    // 回放用: 设了 clock_override 就用它当"现在", 否则用系统时钟。车上从不设置 -> 行为不变。
    std::optional<std::chrono::steady_clock::time_point> clock_override;
    std::chrono::steady_clock::time_point now() const
    {
      return clock_override ? *clock_override : std::chrono::steady_clock::now();
    }

    Plan plan(Target target, double bullet_speed);
    inline Plan plan(std::optional<Target> target, double bullet_speed, double gimbal_yaw = 0, ShootStrategy strategy = Dynamics)
    {
      if (!target.has_value()) return {false};

      double delay_time = std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

      if (std::abs(target->ekf_x()[7]) > decision_speed_) tools::logger()->warn("std::abs(target->ekf_x()[7]) > {}", decision_speed_);

      auto future = now() + std::chrono::microseconds(int(delay_time * 1e6));

      target->predict(future);

      switch (strategy)
      {
        case Dynamics:
          return plan(*target, bullet_speed);
          break;
        case rbSuppressiveFire:
          return rbplan(*target, bullet_speed, gimbal_yaw);
          break;
        case rbHero:
          return rbHeroplan(*target, bullet_speed, gimbal_yaw);
          break;
        case SB:
          return sbplan(*target, bullet_speed, gimbal_yaw);
        default:
          // tools::logger()->warn("planner model error!");
          break;
      }
    }
    Plan rbplan(Target target, double bullet_speed, double gimbal_yaw);

    // ★2026-10-03 快慢打法判定统一入口。标准不变: |转速| >= 90 deg/s(1.57 rad/s) 走高速反陀螺。
    //   但判定用的不是 EKF 瞬时转速, 而是它的低通值(时间常数 spin_select_tau_, yaml spin_select_tau, 秒)。
    //   【原因】日志 2026-10-03_08-58-31: 靶车真实 1.2~1.4 rad/s, EKF 每次换板冲到 2.5~3.0 持续约 0.3s,
    //   这 0.3s 走了高速打法、低速开火逻辑全部失效 -> 慢速没瞄到中心就开火。
    //   tau=0 时等价于原行为(直接用瞬时值)。只影响"走哪套打法", 不改 EKF、不改瞄准预测。
    bool is_high_spin(const Target & target);
    // 每次 rbplan 入口调用一次, 更新低通值。
    void update_spin_select(const Target & target);
    Plan sbplan(Target target, double bullet_speed, double gimbal_yaw);
    bool rbShoot(Target target, double gimbal_yaw, bool tower_fixed_pitch = false);
    Plan rbHeroplan(Target target, double bullet_speed, double gimbal_yaw);

    private:
    bool is_far = false;
    double yaw_offset_;
    double pitch_offset_;
    double far_pitch_offset_;
    double fire_thresh_;
    double target_dist_error_, target_h_error_;
    double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
    double small_armor_tolerance, big_armor_tolerance;
    double tower_and_base_armor_tolerance_;
    double gimbal_control_delay;
    double tower_pitch_prediction_time_;
    double leaving_angle_;  // 反陀螺: 当前板转过正对位置多少度后切到下一块(rad), 读 yaml leaving_angle

    // 激光模拟弹丸模式(yaml laser_mode, 缺省 false)。
    // 激光飞行时间为 0、走直线: 不预测 fly_time、不做重力补偿,
    // 开火判定用 +laser_latency_ 代替 -gimbal_control_delay。
    bool laser_mode_ = false;
    int laser_pitch_mode_ = 0;  // 0=直线, 1=真实弹道, 2=直线+固定pitch_offset
    double laser_latency_ = 0.0;  // s, 图像曝光/传输 + 串口 + 电控中断的总延迟
    double laser_aim_lead_ = 0.0;  // s, 激光模式低速追板时瞄准点前移时间, 抵消云台跟随滞后

    // 开火迟滞 + 进窗确认(yaml fire_exit_scale / fire_confirm_time, 均可缺省)
    double fire_exit_scale_ = 1.0;    // 开火后窗口放宽倍数, 1.0 = 无迟滞(原行为)
    double fire_confirm_time_ = 0.0;  // s, 低速时进窗后需保持多久才开火, 0 = 不确认(原行为)
    double fire_max_duration_ = 0.0;   // s, 低速一次开火最长持续时间, 0 = 不限制
    double fire_switch_settle_time_ = 0.0;  // s, 低速换板后等待云台到新板中心
    bool fire_latched_ = false;
    bool fire_cand_ = false;
    std::chrono::steady_clock::time_point fire_cand_start_t_{};
    std::chrono::steady_clock::time_point fire_last_call_t_{};
    double fire_last_armor_yaw_ = 0.0;
    std::chrono::steady_clock::time_point fire_block_until_t_{};
    std::chrono::steady_clock::time_point fire_started_t_{};
    bool fire_spent_ = false;          // 低速: 本块板已开过一次火, 换板/丢目标前不再开火
    double fire_center_ratio_ = 1.0;   // 低速起火窗口相对开火窗口的比例, 1.0 = 不收窄
    double shoot_latency_ = 0.0;              // s, 电控收到开火 -> 弹丸出膛 的机械延迟
    bool shoot_latency_by_dist_ = false;      // true = shoot_latency 按距离两点线性插值
    double shoot_latency_near_ = 0.0, shoot_latency_far_ = 0.0;            // s
    double shoot_latency_near_dist_ = 2.0, shoot_latency_far_dist_ = 4.0;  // m
    double high_spin_armor_tolerance_ = 0.0;  // m, 高速反陀螺小板开火窗口宽度, 0 = 沿用 small_armor_tolerance
    double high_spin_armor_tolerance_slow_ = 0.0;  // m, 高速分支慢端(<= vyaw_slow)窗口, 0 = 不插值
    double high_spin_window_vyaw_slow_ = 3.0;      // rad/s, 这个转速及以下用宽窗口
    double high_spin_window_vyaw_fast_ = 7.0;      // rad/s, 这个转速及以上用 high_spin_armor_tolerance
    int rbaim_high_spin_best_id_ = -1;        // rbaim() 高速分支本次选中的瞄准板(每次调用都会覆盖)
    int fire_lock_armor_id_ = -1;             // 本周期云台真正在等的那块板, rbShoot 高速开火判定锁定它
    double low_speed_coming_angle_ = 60.0 / 57.3;   // 低速: 板还差多少度转正就开始追它
    double low_speed_leaving_angle_ = 20.0 / 57.3;  // 低速: 板转过正对多少度就放弃、换下一块

    // 快慢打法判定(见 is_high_spin 注释)
    double spin_select_tau_ = 0.0;      // s, 判定用转速的低通时间常数, 0 = 原行为
    double spin_high_settle_time_ = 0.0; // s, 切入高速打法后禁火等待云台换到中心
    double spin_vyaw_settle_time_ = 0.0; // s, 高速vyaw稳定后才允许开火
    double spin_vyaw_stable_band_ = 0.0;  // rad/s, vyaw在此范围内才算稳定
    std::chrono::steady_clock::time_point spin_high_since_{};
    std::chrono::steady_clock::time_point spin_vyaw_stable_since_{};
    double spin_vyaw_at_stable_start_ = 0.0;
    bool spin_high_prev_ = false;
    double spin_select_vyaw_ = 0.0;     // 低通后的 |vyaw|, rad/s
    bool spin_select_init_ = false;
    bool spin_select_high_ = false;     // 带滞回的当前打法状态
    double spin_enter_high_ = 90.0 / 57.3;
    double spin_exit_high_ = 90.0 / 57.3;
    std::chrono::steady_clock::time_point spin_select_t_{};
    static constexpr double kSpinThreshold = 90. / 57.3;  // 快慢分界, 与原代码一致

    // 低速换板时限制视觉 yaw 指令单周期跳变，避免新旧装甲板切换造成甩过再回正。
    double low_speed_yaw_step_limit_ = 1.0 / 57.3;
    bool yaw_command_initialized_ = false;
    double last_yaw_command_ = 0.0;
    std::chrono::steady_clock::time_point last_yaw_command_t_{};

    TinySolver * yaw_solver_;
    TinySolver * pitch_solver_;

    int last_selected_idx = -1;
    Eigen::Vector3d last_selected_xyz = Eigen::Vector3d::Zero();

    void setup_yaw_solver(const std::string & config_path);
    void setup_pitch_solver(const std::string & config_path);

    Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
    Eigen::Matrix<double, 2, 1> rbaim(const Target & target, double bullet_speed);
    Eigen::Matrix<double, 2, 1> heroaim(const Target & target, double bullet_speed, double gimbal_yaw);
    Trajectory get_trajectory(Target target, double yaw0, double bullet_speed);
    Trajectory rbget_trajectory(Target target, double yaw0, double bullet_speed);

    std::chrono::steady_clock::time_point outpost_z_stable_start_time_;
    bool outpost_is_make = true;
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
