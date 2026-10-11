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
    bool fire_latched_ = false;
    bool fire_cand_ = false;
    std::chrono::steady_clock::time_point fire_cand_start_t_{};
    std::chrono::steady_clock::time_point fire_last_call_t_{};
    double fire_last_armor_yaw_ = 0.0;
    // 出膛延迟(电控收到开火 -> 弹丸出膛), 按距离两点线性插值
    double shoot_latency_near_ = 0.0, shoot_latency_far_ = 0.0;            // s
    double shoot_latency_near_dist_ = 2.0, shoot_latency_far_dist_ = 4.0;  // m
    double high_spin_armor_tolerance_ = 0.0;       // m, 高速反陀螺小板开火窗口宽度(快端, >= vyaw_fast)
    double high_spin_armor_tolerance_slow_ = 0.0;  // m, 高速分支慢端(<= vyaw_slow)窗口
    double high_spin_window_vyaw_slow_ = 3.0;      // rad/s, 这个转速及以下用宽窗口
    double high_spin_window_vyaw_fast_ = 7.0;      // rad/s, 这个转速及以上用 high_spin_armor_tolerance
    int rbaim_high_spin_best_id_ = -1;        // rbaim() 高速分支本次选中的瞄准板(每次调用都会覆盖)
    int fire_lock_armor_id_ = -1;             // 本周期云台真正在等的那块板, rbShoot 高速开火判定锁定它
    double low_speed_coming_angle_ = 60.0 / 57.3;   // 低速: 板还差多少度转正就开始追它
    double low_speed_leaving_angle_ = 20.0 / 57.3;  // 低速: 板转过正对多少度就放弃、换下一块

    // ===================== 运动状态判断 =====================
    // 程序里有 5 处要判断"车在不在平移 / 在不在转", 阈值和取值故意不完全一样(都是回放调出来的):
    //
    //  用在哪                    | 在转的判据                         | 平移的判据
    //  --------------------------+------------------------------------+-----------------------------------------
    //  走哪套打法(高速/低速)     | spin_select_high_ (低通+滞回 2.0/1.3)| -
    //  观测拟合 / 开火平移判断   | high_spin 或 |vyaw|>=spinning_vyaw_ | 拟合修正前车速 >= fire_trans_speed_ (0.3)
    //                            |   (瞬时, 无保持)                   |
    //  低速 yaw 限幅 A 版/放开   | |vyaw|>=spinning_vyaw_ 且保持      | 拟合修正后车速 >= yaw_follow_trans_speed_ (0.5)
    //                            |   spinning_hold_ (0.3s)            |   (放宽限幅 + 发速度前馈)
    //  高速反陀螺速度前馈        | (已在高速档)                       | 拟合修正后车速 >= high_spin_ff_trans_speed_ (0.3)
    //  高速+往左平移窗口右挪     | (已在高速档)                       | 往左横向车速 >= high_spin_left_shift_min_speed_ (0.5)
    //
    // 为什么开火用 0.3、云台放开用 0.5:
    //   云台放开限幅/发前馈太早, 慢速小陀螺(车速 0.3~0.5 常见)会抖(10-06/10-08 实车)。
    //   开火门限 0.3 来自平移开火回放, 慢速平移也要能开火。
    // 为什么云台判"在转"要保持 0.3s 而开火不保持:
    //   平移时约 14% 周期 EKF 转速被带偏到 0.6 以上, 云台要防抖所以保持; 开火判断是逐周期的, 原版就不保持。
    // 已知不一致: 车速 0.3~0.5 或转速刚掉到 0.6 以下的 0.3s 内,
    //   开火按"平移"判, 云台还按"在转"(A 版限幅, 慢)。2026-10-11 回放(17 份日志)试过统一, 都变差, 保留两条线:
    //   1) 开火也用带 0.3s 保持的"在转": 纯平移开火 86.9% -> 81.5%, 慢陀螺 33.0% -> 25.1%。
    //      丢掉的开火里, 慢陀螺那部分开火时云台离指令平均只有 0.17 度(本来就打得准), 不是"没追上就开火"。
    //   2) 开火平移门限 0.3 -> 0.5(观测拟合仍 0.3): 纯平移开火 86.9% -> 73.5%。
    //      车速 0.3~0.5 时开火的周期, 云台离指令平均 0.39 度, 超过 1 度的只有 4.7%(v>=0.5 时是 26.7%)。
    //   结论: 这两处不一致在回放里没造成"云台没追上就开火", 统一反而少开火。上车如果看到慢速平移打偏再回来查。
    double spinning_vyaw_ = 0.6;                // rad/s, |vyaw| 超过它算"在转"(yaml low_speed_yaw_spin_vyaw)
    double fire_trans_speed_ = 0.3;             // m/s, 开火/观测拟合的平移门限(yaml trans_fire_speed)
    double yaw_follow_trans_speed_ = 0.5;       // m/s, 低速 yaw 放宽/前馈的平移门限(yaml low_speed_yaw_follow_min_speed)
    double high_spin_ff_trans_speed_ = 0.3;     // m/s, 高速反陀螺发速度前馈的车速门限(yaml high_spin_center_ff_min_speed)
    struct MotionFrame
    {
      bool high_spin = false;     // 本周期走高速打法
      double speed_raw = 0.0;     // 观测拟合修正前的 EKF 车中心速度(m/s)
      bool spinning_now = false;  // high_spin 或 |vyaw| >= spinning_vyaw_ (瞬时, 无保持)
    };
    MotionFrame motion_;          // rbplan 每周期算一次, rbShoot 读

    // 快慢打法判定(见 is_high_spin 注释)
    double spin_select_tau_ = 0.0;      // s, 判定用转速的低通时间常数
    double spin_select_vyaw_ = 0.0;     // 低通后的 |vyaw|, rad/s
    bool spin_select_init_ = false;
    bool spin_select_high_ = false;     // 带滞回的当前打法状态
    double spin_enter_high_ = 90.0 / 57.3;
    double spin_exit_high_ = 90.0 / 57.3;
    std::chrono::steady_clock::time_point spin_select_t_{};

    // 低速换板时限制视觉 yaw 指令单周期跳变，避免新旧装甲板切换造成甩过再回正。
    double low_speed_yaw_step_limit_ = 1.0 / 57.3;
    bool yaw_command_initialized_ = false;
    double last_yaw_command_ = 0.0;
    std::chrono::steady_clock::time_point last_yaw_command_t_{};
    // ★2026-10-07 只在换板时限幅(在转时仍每周期限幅 = A 版)
    double low_speed_yaw_switch_jump_ = 0.5 / 57.3;  // 换板信号下单周期跳变超过它才限
    double low_speed_yaw_follow_gain_ = 1.5;         // 放宽量 = 车中心方位角速度 x 周期 x gain(留余量给提前量变化)
    double low_speed_yaw_spin_hold_ = 0.3;           // s, 转速掉下去后保持 A 版这么久再放开
    double low_speed_yaw_vel_limit_ = 60.0 / 57.3;   // rad/s, yaw 速度前馈上限
    double high_spin_left_shift_ = 0.0;              // m, 高速+往左平移时开火窗口往右挪多少(正 = 落点右移), 0 = 关
    double high_spin_left_shift_min_speed_ = 0.5;    // m/s, 往左的横向车速低于它不挪(原地小陀螺不变)
    std::chrono::steady_clock::time_point last_spin_seen_t_{};
    // ★2026-10-07 急刹不开火
    double fire_brake_decel_ = 0.0;       // m/s^2, 50ms 内减速超过它算急刹
    double fire_brake_hold_ = 0.15;       // s, 急刹后多久不开火
    std::chrono::steady_clock::time_point fire_brake_until_t_{};
    std::deque<std::pair<std::chrono::steady_clock::time_point, double>> fire_speed_hist_;
    double fire_cmd_gimbal_tol_ = 0.0; // rad, 平移时云台离指令小于它才开火
    double fire_cmd_yaw_ = 0.0;        // rbplan 本周期的 yaw 指令(已减 yaw_offset), 给 rbShoot 用
    double fire_trans_aim_tol_ = 0.0;  // rad, 平移时 |指令-板预测| 门限
    double fire_trans_hold_scale_ = 1.25;  // 开火中门限放宽倍数
    bool fire_cmd_yaw_valid_ = false;
    // ★2026-10-07 平移时直接拟合最近观测修正预测
    double obs_fit_window_ = 0.2;     // s
    int obs_fit_order_ = 2;           // 1 = 匀速, 2 = 匀加速
    double obs_fit_max_shift_ = 0.3;  // m, 修正量超过它就不修(拟合坏了)
    double obs_fit_max_dv_ = 1.5;     // m/s, 速度修正上限
    Eigen::Vector2d obs_fit_shift_f_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d obs_fit_dv_f_ = Eigen::Vector2d::Zero();
    std::chrono::steady_clock::time_point obs_fit_last_t_{};
    int obs_fit_last_id_ = -1;

    public:
    bool debug_obs_fit_used_ = false;
    bool debug_obs_fit_raw_ok_ = false;
    int debug_fire_gate_ = 0;           // rbShoot 每道门: 1 不在窗口 2 云台离指令远 8 急刹
    double debug_fire_win_deg_ = 0.0;   // 开火窗口半宽(度)
    double debug_fire_delta_deg_ = 0.0; // 判开火用的角度差(度)
    double debug_fire_dgim_deg_ = 0.0;  // 指令离云台(度)
    bool debug_fire_trans_ = false;     // 本周期走的是平移开火判断
    Eigen::Vector2d debug_obs_fit_raw_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d debug_obs_fit_applied_ = Eigen::Vector2d::Zero();

    private:
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
