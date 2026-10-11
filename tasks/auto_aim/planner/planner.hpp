#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <deque>
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
    Eigen::Vector4d debug_xyza;
    double aim_target_yaw;
    Planner(const std::string & config_path);

    // 回放用: 设了 clock_override 就用它当"现在", 否则用系统时钟。车上从不设置 -> 行为不变。
    std::optional<std::chrono::steady_clock::time_point> clock_override;
    std::chrono::steady_clock::time_point now() const
    {
      return clock_override ? *clock_override : std::chrono::steady_clock::now();
    }

    // 上场入口: 先把目标预测到"现在", 再走 rbplan()。
    //   predict(now()) 不能删: target 是视觉线程那一帧的状态, 规划线程每 5ms 都要推到当前时刻。
    inline Plan plan(std::optional<Target> target, double bullet_speed, double gimbal_yaw = 0)
    {
      if (!target.has_value()) return {false};
      target->predict(now());
      return rbplan(*target, bullet_speed, gimbal_yaw);
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
    bool rbShoot(Target target, double gimbal_yaw);

    private:
    double yaw_offset_;
    double pitch_offset_;
    double target_dist_error_, target_h_error_;
    double small_armor_tolerance, big_armor_tolerance;
    double tower_and_base_armor_tolerance_;
    double tower_pitch_prediction_time_;
    double leaving_angle_;  // 反陀螺: 当前板转过正对位置多少度后切到下一块(rad), 读 yaml leaving_angle

    // 开火模式(yaml fire_mode: bullet / laser, 缺省 bullet)。
    // 激光飞行时间为 0、走直线: 不预测 fly_time、不做重力补偿,
    bool laser_ = false;               // true = 激光模式
    double fire_link_latency_ = 0.0;   // s, 拍照 -> 电控收到指令的链路延迟, 两种模式共用
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
    // ★2026-10-07 只在换板时限幅(yaml low_speed_yaw_limit_switch_only, 缺省关 = A 版行为)
    bool low_speed_yaw_limit_switch_only_ = false;
    double low_speed_yaw_switch_jump_ = 0.5 / 57.3;  // 换板信号下单周期跳变超过它才限
    double low_speed_yaw_spin_vyaw_ = 0.6;           // rad/s, |vyaw| 超过它算在转, 走 A 版
    bool low_speed_yaw_step_follow_center_ = false;  // 限幅加上车中心方位角变化(平移时不被钉住)
    bool low_speed_yaw_vel_follow_cmd_ = false;      // 限幅时前馈 = 指令自己的角速度(而不是 0)
    double low_speed_yaw_follow_min_speed_ = 0.5;    // m/s, 车速低于它不放宽(小陀螺完全 A 版)
    double low_speed_yaw_follow_gain_ = 1.5;         // 放宽量 = 车中心方位角速度 x 周期 x gain(留余量给提前量变化)
    double low_speed_yaw_spin_hold_ = 0.3;           // s, 转速掉下去后保持 A 版这么久再放开
    double low_speed_yaw_vel_limit_ = 60.0 / 57.3;   // 放开路径的 yaw 速度前馈上限, 0 = 不限
    bool high_spin_center_vel_ff_ = false;           // 高速反陀螺+平移: yaw 速度前馈 = 车中心方位角速度(原来 0)
    double high_spin_center_ff_min_speed_ = 0.3;     // m/s, 车速低于它不发(原地高速小陀螺完全原行为)
    double high_spin_left_shift_ = 0.0;              // m, 高速+往左平移时开火窗口往右挪多少(正 = 落点右移), 0 = 关
    double high_spin_left_shift_min_speed_ = 0.5;    // m/s, 往左的横向车速低于它不挪(原地小陀螺不变)
    std::chrono::steady_clock::time_point last_spin_seen_t_{};
    // ★2026-10-07 平移开火窗口 / 急刹不开火 (yaml, 0 = 关)
    double trans_armor_tolerance_ = 0.0;  // m, 平移时开火窗口宽度(板面左右), 0 = 沿用 small_armor_tolerance
    double trans_fire_speed_ = 0.3;       // m/s, 车速超过它算平移
    double fire_brake_decel_ = 0.0;       // m/s^2, 50ms 内减速超过它算急刹, 0 = 关
    double fire_brake_hold_ = 0.15;       // s, 急刹后多久不开火
    std::chrono::steady_clock::time_point fire_brake_until_t_{};
    std::deque<std::pair<std::chrono::steady_clock::time_point, double>> fire_speed_hist_;
    // ★2026-10-07 减速不开火 / 平移开火判断用云台提前角 (yaml, 0 = 关)
    double fire_decel_ratio_ = 0.0;    // 车速在窗口内掉了这个比例就不开火, 0 = 关
    double fire_decel_window_ = 0.1;   // s
    std::deque<std::pair<std::chrono::steady_clock::time_point, double>> fire_decel_hist_;
    double fire_gimbal_lead_ = 0.0;    // s, 平移时开火判断用云台这么久之后的角度, 0 = 关
    double fire_cmd_gimbal_tol_ = 0.0; // rad, 平移时按指令判窗口 + 云台离指令小于它才开火, 0 = 关
    double fire_cmd_yaw_ = 0.0;        // rbplan 本周期的 yaw 指令(已减 yaw_offset), 给 rbShoot 用
    double fire_trans_aim_tol_ = 0.0;  // rad, 平移时 |指令-板预测| 门限, 0 = 关(用原窗口)
    double fire_trans_hold_scale_ = 1.25;  // 开火中门限放宽倍数
    bool fire_cmd_yaw_valid_ = false;
    // ★2026-10-07 平移加速度补偿 (yaml trans_accel_gain, 0 = 关)
    double trans_accel_gain_ = 0.0;
    double trans_accel_window_ = 0.1;      // s, 速度差分窗口
    double trans_accel_max_ = 4.0;         // m/s^2, 加速度上限
    double trans_accel_extra_time_ = 0.0;  // s, 在弹丸飞行时间之外多补的时间(云台跟随)
    double trans_accel_deadzone_ = 1.0;    // m/s^2, 沿运动方向的加速度超过它才补(滤掉车速估计噪声)
    std::deque<std::pair<std::chrono::steady_clock::time_point, Eigen::Vector2d>> trans_accel_hist_;
    std::chrono::steady_clock::time_point trans_accel_last_t_{};

    // ★2026-10-07 平移时直接拟合最近观测修正预测 (yaml obs_fit_enable, 缺省关)
    bool obs_fit_enable_ = false;
    double obs_fit_window_ = 0.2;     // s
    int obs_fit_order_ = 2;           // 1 = 匀速, 2 = 匀加速
    double obs_fit_max_shift_ = 0.3;  // m, 修正量超过它就不修(拟合坏了)
    double obs_fit_max_dv_ = 1.5;     // m/s, 速度修正上限
    double obs_fit_smooth_tau_ = 0.0; // s, 修正量低通时间常数, 0 = 不平滑
    double obs_fit_blend_ = 1.0;      // 修正量用多少(0~1)
    double obs_fit_fast_thr_ = 0.0;   // m, 修正量变化超过它就不平滑直接跟, 0 = 一直平滑
    bool obs_fit_vel_only_ = false;   // true: 位置用 EKF, 只换拟合的速度/加速度(噪声小)
    Eigen::Vector2d obs_fit_shift_f_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d obs_fit_dv_f_ = Eigen::Vector2d::Zero();
    std::chrono::steady_clock::time_point obs_fit_last_t_{};
    int obs_fit_last_id_ = -1;

    public:
    Eigen::Vector2d debug_trans_accel_ = Eigen::Vector2d::Zero();  // 本周期用的加速度(回放/日志用)
    bool debug_obs_fit_used_ = false;
    bool debug_obs_fit_raw_ok_ = false;
    int debug_fire_gate_ = 0;           // rbShoot 每道门: 1 不在窗口 2 云台离指令远 4 换板等待 8 急刹 16 已用完
    double debug_fire_win_deg_ = 0.0;   // 开火窗口半宽(度)
    double debug_fire_delta_deg_ = 0.0; // 判开火用的角度差(度)
    double debug_fire_dgim_deg_ = 0.0;  // 指令离云台(度)
    bool debug_fire_trans_ = false;     // 本周期走的是平移开火判断
    Eigen::Vector2d debug_obs_fit_raw_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d debug_obs_fit_applied_ = Eigen::Vector2d::Zero();
    double fire_raw_speed_ = 0.0;       // 观测拟合修正前 EKF 车速, 给急刹判断用
    bool fire_raw_speed_valid_ = false;

    private:
    bool fire_gimbal_init_ = false;
    double fire_last_gimbal_yaw_ = 0.0;
    std::chrono::steady_clock::time_point fire_last_gimbal_t_{};
    bool yaw_switch_state_init_ = false;
    bool yaw_catching_up_ = false;     // 换板后正在按 step_limit 追原始瞄准角
    bool last_switch_signal_ = false;  // 上一周期有换板信号(允许错开 1 周期)
    double last_raw_yaw_ = 0.0;        // 上一周期限幅前的原始瞄准角
    double last_aim_armor_yaw_ = 0.0;  // 上一周期 rbaim 瞄准板朝向
    int last_target_id_ = -1;
    int last_update_count_ = 0;

    TinySolver * yaw_solver_;
    TinySolver * pitch_solver_;

    void setup_yaw_solver(const std::string & config_path);
    void setup_pitch_solver(const std::string & config_path);

    Eigen::Matrix<double, 2, 1> rbaim(const Target & target, double bullet_speed);
    // 前哨站没稳定前直接瞄中间那块板(rbplan 里用), 不走 MPC
    Eigen::Matrix<double, 2, 1> heroaim(const Target & target, double bullet_speed, double gimbal_yaw);
    Trajectory rbget_trajectory(Target target, double yaw0, double bullet_speed);

    std::chrono::steady_clock::time_point outpost_z_stable_start_time_;
    bool outpost_is_make = true;
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
