#include <algorithm>
#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
  Planner::Planner(const std::string & config_path)
  {
    auto yaml = tools::load(config_path);
    yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
    pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
    target_dist_error_ = tools::read<double>(yaml, "target_dist_error");
    target_h_error_ = tools::read<double>(yaml, "target_h_error");
    small_armor_tolerance = tools::read<double>(yaml, "small_armor_tolerance");
    big_armor_tolerance = tools::read<double>(yaml, "big_armor_tolerance");
    tower_and_base_armor_tolerance_ = tools::read<double>(yaml, "tower_and_base_armor_tolerance_");
    tower_pitch_prediction_time_ = tools::read<double>(yaml, "tower_pitch_prediction_time");
    leaving_angle_ = tools::read<double>(yaml, "leaving_angle") / 57.3;

    // 可选键: 其它 yaml(demo/detect_test) 没有这两项, 不能用 tools::read(缺键直接 exit)。
    // 开火模式: bullet(实弹, 缺省) / laser(激光)。其它值直接退出, 防止写错静默走错模式。
    {
      const std::string mode = yaml["fire_mode"] ? yaml["fire_mode"].as<std::string>() : std::string("bullet");
      if (mode != "bullet" && mode != "laser")
      {
        tools::logger()->error("[Planner] fire_mode 只能是 bullet 或 laser, 现在是 \"{}\"", mode);
        std::exit(1);
      }
      laser_ = mode == "laser";
    }
    fire_link_latency_ = yaml["fire_link_latency"] ? yaml["fire_link_latency"].as<double>() : 0.0;
    laser_aim_lead_ = yaml["laser_aim_lead"] ? yaml["laser_aim_lead"].as<double>() : 0.0;
    fire_exit_scale_ = yaml["fire_exit_scale"] ? yaml["fire_exit_scale"].as<double>() : 1.0;
    fire_confirm_time_ = yaml["fire_confirm_time"] ? yaml["fire_confirm_time"].as<double>() : 0.0;
    // ★2026-10-05 出膛延迟按距离两点线性插值(超出范围外推, 不低于 0)。
    shoot_latency_near_ = tools::read<double>(yaml, "shoot_latency_near");
    shoot_latency_far_ = tools::read<double>(yaml, "shoot_latency_far");
    shoot_latency_near_dist_ = tools::read<double>(yaml, "shoot_latency_near_dist");
    shoot_latency_far_dist_ = tools::read<double>(yaml, "shoot_latency_far_dist");
    if (std::abs(shoot_latency_far_dist_ - shoot_latency_near_dist_) <= 0.1)
    {
      tools::logger()->error("[Planner] shoot_latency_near_dist 和 shoot_latency_far_dist 至少要差 0.1m");
      std::exit(1);
    }
    tools::logger()->warn("[Planner] shoot_latency 按距离插值: {:.1f}m={:.1f}ms, {:.1f}m={:.1f}ms",
                          shoot_latency_near_dist_, shoot_latency_near_ * 1e3, shoot_latency_far_dist_, shoot_latency_far_ * 1e3);
    high_spin_armor_tolerance_ = tools::read<double>(yaml, "high_spin_armor_tolerance");
    high_spin_armor_tolerance_slow_ = tools::read<double>(yaml, "high_spin_armor_tolerance_slow");
    high_spin_window_vyaw_slow_ = tools::read<double>(yaml, "high_spin_window_vyaw_slow");
    high_spin_window_vyaw_fast_ = tools::read<double>(yaml, "high_spin_window_vyaw_fast");
    if (high_spin_armor_tolerance_ <= 0.0 || high_spin_armor_tolerance_slow_ <= 0.0 ||
        high_spin_window_vyaw_fast_ <= high_spin_window_vyaw_slow_)
    {
      tools::logger()->error("[Planner] high_spin_armor_tolerance(_slow) 要 > 0, 且 high_spin_window_vyaw_fast > _slow");
      std::exit(1);
    }
    tools::logger()->warn("[Planner] 高速窗口按转速插值: <= {:.1f}rad/s 用 {:.0f}mm, >= {:.1f}rad/s 用 {:.0f}mm",
                          high_spin_window_vyaw_slow_, high_spin_armor_tolerance_slow_ * 1e3,
                          high_spin_window_vyaw_fast_, high_spin_armor_tolerance_ * 1e3);
    // 缺省值 = 原来写死的 60 / leaving_angle, 不配置时行为不变。
    low_speed_coming_angle_ = (yaml["low_speed_coming_angle"] ? yaml["low_speed_coming_angle"].as<double>() : 60.0) / 57.3;
    low_speed_leaving_angle_ = yaml["low_speed_leaving_angle"] ? yaml["low_speed_leaving_angle"].as<double>() / 57.3 : leaving_angle_;
    fire_switch_settle_time_ = yaml["fire_switch_settle_time"] ? yaml["fire_switch_settle_time"].as<double>() : 0.0;
    spin_select_tau_ = tools::read<double>(yaml, "spin_select_tau");
    spin_enter_high_ = tools::read<double>(yaml, "spin_enter_high");
    spin_exit_high_ = tools::read<double>(yaml, "spin_exit_high");
    if (spin_select_tau_ <= 0.0)
    {
      tools::logger()->error("[Planner] spin_select_tau 要 > 0");
      std::exit(1);
    }
    if (spin_exit_high_ > spin_enter_high_) spin_exit_high_ = spin_enter_high_;
    tools::logger()->warn("[Planner] 快慢判定: 低通 {:.2f}s, 进高速 {:.2f} / 退高速 {:.2f} rad/s", spin_select_tau_,
                          spin_enter_high_, spin_exit_high_);
    if (yaml["low_speed_yaw_step_limit_deg"])
      low_speed_yaw_step_limit_ = yaml["low_speed_yaw_step_limit_deg"].as<double>() / 57.3;
    if (yaml["low_speed_yaw_switch_jump_deg"])
      low_speed_yaw_switch_jump_ = yaml["low_speed_yaw_switch_jump_deg"].as<double>() / 57.3;
    if (yaml["low_speed_yaw_follow_min_speed"]) yaw_follow_trans_speed_ = yaml["low_speed_yaw_follow_min_speed"].as<double>();
    if (yaml["low_speed_yaw_follow_gain"]) low_speed_yaw_follow_gain_ = yaml["low_speed_yaw_follow_gain"].as<double>();
    if (yaml["low_speed_yaw_spin_vyaw"])
      spinning_vyaw_ = yaml["low_speed_yaw_spin_vyaw"].as<double>();
    if (yaml["low_speed_yaw_spin_hold"])
      low_speed_yaw_spin_hold_ = yaml["low_speed_yaw_spin_hold"].as<double>();
    low_speed_yaw_vel_limit_ = tools::read<double>(yaml, "low_speed_yaw_vel_limit_deg") / 57.3;
    if (low_speed_yaw_vel_limit_ <= 0.0)
    {
      tools::logger()->error("[Planner] low_speed_yaw_vel_limit_deg 要 > 0");
      std::exit(1);
    }
    if (yaml["high_spin_center_ff_min_speed"])
      high_spin_ff_trans_speed_ = yaml["high_spin_center_ff_min_speed"].as<double>();
    tools::logger()->warn("[Planner] 高速反陀螺+平移: yaw 速度前馈 = 车中心方位角速度(车速 >= {:.2f}m/s, 上限 {:.0f} deg/s)",
                          high_spin_ff_trans_speed_, low_speed_yaw_vel_limit_ * 57.3);
    high_spin_left_shift_ = yaml["high_spin_left_shift"] ? yaml["high_spin_left_shift"].as<double>() : 0.0;
    if (yaml["high_spin_left_shift_min_speed"])
      high_spin_left_shift_min_speed_ = yaml["high_spin_left_shift_min_speed"].as<double>();
    if (high_spin_left_shift_ != 0.0)
      tools::logger()->warn("[Planner] 高速小陀螺+往左平移: 开火窗口往右挪 {:.1f}mm(往左横向车速 >= {:.2f}m/s)",
                            high_spin_left_shift_ * 1000.0, high_spin_left_shift_min_speed_);
    fire_trans_speed_ = yaml["trans_fire_speed"] ? yaml["trans_fire_speed"].as<double>() : 0.3;
    fire_brake_decel_ = tools::read<double>(yaml, "fire_brake_decel");
    fire_brake_hold_ = yaml["fire_brake_hold"] ? yaml["fire_brake_hold"].as<double>() : 0.15;
    fire_cmd_gimbal_tol_ = tools::read<double>(yaml, "fire_cmd_gimbal_tol_deg") / 57.3;
    fire_trans_aim_tol_ = tools::read<double>(yaml, "fire_trans_aim_tol_deg") / 57.3;
    if (yaml["fire_trans_hold_scale"]) fire_trans_hold_scale_ = std::max(1.0, yaml["fire_trans_hold_scale"].as<double>());
    if (fire_brake_decel_ <= 0.0 || fire_cmd_gimbal_tol_ <= 0.0 || fire_trans_aim_tol_ <= 0.0)
    {
      tools::logger()->error("[Planner] fire_brake_decel / fire_cmd_gimbal_tol_deg / fire_trans_aim_tol_deg 都要 > 0");
      std::exit(1);
    }
    tools::logger()->warn("[Planner] 平移开火第四版: 指令离板 <= {:.2f} deg 且云台离指令 <= {:.2f} deg 就开火(开火中放宽 x{:.2f})",
                          fire_trans_aim_tol_ * 57.3, fire_cmd_gimbal_tol_ * 57.3, fire_trans_hold_scale_);
    if (yaml["obs_fit_window"]) obs_fit_window_ = yaml["obs_fit_window"].as<double>();
    if (yaml["obs_fit_order"]) obs_fit_order_ = std::clamp(yaml["obs_fit_order"].as<int>(), 1, 2);
    if (yaml["obs_fit_max_shift"]) obs_fit_max_shift_ = yaml["obs_fit_max_shift"].as<double>();
    if (yaml["obs_fit_max_dv"]) obs_fit_max_dv_ = yaml["obs_fit_max_dv"].as<double>();
    tools::logger()->warn("[Planner] 平移观测拟合: {} 阶, 窗口 {:.0f}ms, 修正上限 {:.2f}m / {:.1f}m/s",
                          obs_fit_order_, obs_fit_window_ * 1e3, obs_fit_max_shift_, obs_fit_max_dv_);
    tools::logger()->warn("[Planner] 平移开火: 车速>={:.2f}m/s; 急刹(减速>{:.1f}m/s2)后 {:.0f}ms 不开火",
                          fire_trans_speed_, fire_brake_decel_, fire_brake_hold_ * 1e3);
    tools::logger()->warn("[Planner] 低速 yaw 限幅只在换板时生效: 换板跳变阈值 {:.2f} deg, |vyaw|>={:.2f}rad/s 走 A 版(保持 {:.2f}s), 速度前馈上限 {:.0f} deg/s",
                          low_speed_yaw_switch_jump_ * 57.3, spinning_vyaw_, low_speed_yaw_spin_hold_, low_speed_yaw_vel_limit_ * 57.3);
    tools::logger()->warn("[Planner] 开火模式: {}, 链路延迟 fire_link_latency={:.1f}ms{}", laser_ ? "laser(激光)" : "bullet(实弹)",
                          fire_link_latency_ * 1e3, laser_ ? ", 低速瞄准前推 laser_aim_lead" : "");

    setup_yaw_solver(config_path);
    setup_pitch_solver(config_path);
  }

  bool Planner::rbShoot(Target target, double gimbal_yaw)
  {
    bool suggest_fire = 1;
    const auto armor_xyza = target.armor_xyza_list();
    Eigen::Vector4d target_armor_xyza;
    auto min_dist = 1e10;
    int selected_id = -1;
    for (int i = 0; i < static_cast<int>(armor_xyza.size()); ++i)
    {
      const auto & xyza = armor_xyza[i];
      auto dist = xyza.head<2>().norm();
      if (dist < min_dist)
      {
        min_dist = dist;
        target_armor_xyza = xyza;
        selected_id = i;
      }
    }

    // 低速追板时，开火判断必须和 rbaim() 使用同一块 last_id 装甲板，
    // 不能在这里再次按最近距离选另一块板，否则会出现“瞄旧板、按新板开火”。
    const auto ekf_x = target.ekf_x();
    const bool high_spin = is_high_spin(target);

    // ★2026-10-04 高速反陀螺: 开火判定锁定 rbaim() 选中的那块板(fire_lock_armor_id_), 不再按最近距离选。
    //   【原因】这里的 target 已经往前预测了 fire_link_latency + shoot_latency(+fly_time 已在 rbplan 入口预测),
    //   9.6rad/s 时总提前量 ~100ms 以上, 板要转 ~55 度, 超过相邻两板间隔的一半(45 度)。
    //   按"最近距离"选板时, 一半的时候选到的是下一块板, 日志 13-49-57 高速段下令时 face
    //   一半在 -40 度、一半在 +42 度, 后者就是按错板开火, 弹丸落在板转走的一侧(偏右)。
    //   中速 8.1rad/s 时提前量还不到 45 度, 所以中速没事。
    //   锁定后, 判定的始终是云台正在等的那块板: 它预测到弹丸到达时刻有没有转进窗口。
    if (high_spin && fire_lock_armor_id_ >= 0 &&
        fire_lock_armor_id_ < static_cast<int>(armor_xyza.size()))
    {
      selected_id = fire_lock_armor_id_;
      target_armor_xyza = armor_xyza[selected_id];
    }
    if (!high_spin && target.last_id >= 0 &&
        target.last_id < static_cast<int>(armor_xyza.size()))
    {
      const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
      const double spin_sign = ekf_x(7) >= 0 ? 1.0 : -1.0;
      const double d = tools::limit_rad(armor_xyza[target.last_id][3] - center_yaw) * spin_sign;
      // 和 rbaim() 用同一个范围, 保证"瞄的板"和"按哪块板判开火"一致。
      if (d <= low_speed_leaving_angle_ && d >= -low_speed_coming_angle_)
      {
        selected_id = target.last_id;
        target_armor_xyza = armor_xyza[selected_id];
      }
    }

    double target_yaw = target_armor_xyza(3);

    aim_target_yaw = atan2(target_armor_xyza(1), target_armor_xyza(0));

    // ★2026-09-28 开火迟滞: 换板 / 丢目标一段时间后, 锁存状态作废, 重新走"进入窗口 + 确认"。
    const auto fire_now_t = now();
    const bool armor_switched =
      std::abs(tools::limit_rad(target_yaw - fire_last_armor_yaw_)) > 30. / 57.3;
    const bool target_lost = tools::delta_time(fire_now_t, fire_last_call_t_) > 0.1;
    if (target_lost || armor_switched)
    {
      fire_latched_ = false;
      fire_cand_ = false;
    }
    const bool low_speed = !high_spin;
    if (armor_switched && low_speed)
      fire_block_until_t_ = fire_now_t + std::chrono::microseconds(static_cast<int64_t>(fire_switch_settle_time_ * 1e6));
    fire_last_call_t_ = fire_now_t;
    fire_last_armor_yaw_ = target_yaw;

    double shoot_range = target.armor_type == ArmorType::big ? big_armor_tolerance : small_armor_tolerance;

    // ★2026-10-04 高速反陀螺开火窗口单独设置(yaml high_spin_armor_tolerance)。
    //   仍然是"装甲板面上左右多宽"换成角度判断, 不按时间。
    //   原来 0.03m(±1.5cm): 8.5rad/s 时板扫过只有 ~11ms, 电控 25Hz(40ms 一发) 只够一发。
    //   小板宽 135mm, 弹落点离板中心 ±(135/2 - 余量) 以内都算打中。窗口取 0.10m(±5cm):
    //   两发首尾落点离中心最多 ±5cm, 留 1.75cm 给散布, 不出板;
    //   8.5rad/s / r=0.29m 时窗口 ~40ms, 刚好两发; 转得更快只剩一发, 转得慢可能三发, 都不会打出板。
    if (high_spin && target.armor_type != ArmorType::big)
    {
      // ★2026-10-05 高速分支窗口按转速插值(yaml high_spin_armor_tolerance_slow + high_spin_window_vyaw_slow/fast)。
      //   原来高速分支(|vyaw|>=2)一律用 high_spin_armor_tolerance(调好的 0.025 是给 7rad/s+ 一块板一发用的)。
      //   日志 14-23-33: 2.9rad/s 时 ±1.25cm 窗口板扫过只有 ~30ms, 电控 25Hz(40ms) 连发不起来, 0~1 发。
      //   转速 <= vyaw_slow 用宽窗口(连发), >= vyaw_fast 用原窄窗口(一发), 中间线性过渡。
      //   宽窗口上限仍受装甲板宽度约束(小板宽 135mm), 不会打出板。
      const double v = std::abs(target.ekf_x()(7));
      double k = (v - high_spin_window_vyaw_slow_) / (high_spin_window_vyaw_fast_ - high_spin_window_vyaw_slow_);
      k = std::clamp(k, 0.0, 1.0);
      shoot_range = high_spin_armor_tolerance_slow_ + k * (high_spin_armor_tolerance_ - high_spin_armor_tolerance_slow_);
    }

    // ★2026-10-07 急刹不开火(yaml fire_brake_decel)。
    //   平移 = 车速 >= trans_fire_speed 且 |vyaw| < low_speed_yaw_spin_vyaw(和 yaw 放开的判据一致)。
    //   平移时开火窗口在下面的"平移开火第四版"里单独判断, 不用 shoot_range。
    //   【急刹】车速 50ms 内掉得比 fire_brake_decel(m/s^2) 快, 认为在急刹, fire_brake_hold 秒内不开火:
    //   急刹时 EKF 的速度还没跟上, 预测点往前冲, 这时打出去的弹基本落在车前面。
    //   回放统计: 减速 > 3m/s^2 在平移段占 2~11% 周期, 静止/慢速小陀螺 < 3%。
    // 观测拟合会改写 target 的速度(每周期在跳), 急刹/平移判断用修正前 EKF 自己的车速(rbplan 里算好的)。
    // vyaw 这里读本函数 target 自己的(已预测到开火时刻): 前哨站 predict 时转速会被限幅, 和入口值可能不同。
    const double center_speed = motion_.speed_raw;
    const bool translating = !high_spin && center_speed >= fire_trans_speed_ &&
                             std::abs(ekf_x[7]) < spinning_vyaw_;

    bool braking = false;
    {
      if (target_lost) fire_speed_hist_.clear();
      fire_speed_hist_.push_back({fire_now_t, center_speed});
      while (fire_speed_hist_.size() > 1 && tools::delta_time(fire_now_t, fire_speed_hist_.front().first) > 0.05)
        fire_speed_hist_.pop_front();
      const auto & old = fire_speed_hist_.front();
      const double dt = tools::delta_time(fire_now_t, old.first);
      if (dt > 0.03 && old.second > 0.3 && (old.second - center_speed) / dt > fire_brake_decel_)
        fire_brake_until_t_ = fire_now_t + std::chrono::microseconds(static_cast<int64_t>(fire_brake_hold_ * 1e6));
      braking = !high_spin && fire_now_t < fire_brake_until_t_;
    }

    double fire_gimbal_yaw = gimbal_yaw;
    // ★2026-10-07 平移时按"指令"判窗口, 再要求云台离指令不远(yaml fire_cmd_gimbal_tol_deg)。
    //   实车 18-13-10: 起步/加速时云台落后指令 ~0.9 度(匀速 0.5), 上面按云台角速度外推补不够, 开火 6%。
    //   改成: 指令(本周期发给电控的 yaw)在窗口里 = 瞄的地方是对的; |云台 - 指令| < tol = 云台快追上了。
    //   两条同时满足就开火。只在平移用, 小陀螺不变。
    bool cmd_gimbal_far = false;
    if (translating && fire_cmd_yaw_valid_)
    {
      fire_gimbal_yaw = fire_cmd_yaw_;
      cmd_gimbal_far = std::abs(tools::limit_rad(fire_cmd_yaw_ - gimbal_yaw)) > fire_cmd_gimbal_tol_;
    }

    if (target.name == ArmorName::base || target.name == ArmorName::outpost) shoot_range = tower_and_base_armor_tolerance_;

    // 已经在开火: 窗口放宽到 fire_exit_scale_ 倍才判"出窗"(施密特触发器)。
    // 否则云台稳态误差 0.1~0.3 度正好压在 ±0.34 度的窗口边上, fire 会被噪声来回翻。
    if (fire_latched_) shoot_range *= fire_exit_scale_;

    // 打击范围计算
    double ax = target_armor_xyza(0) - 0.5f * shoot_range * sin(target_yaw);
    double ay = target_armor_xyza(1) + 0.5f * shoot_range * cos(target_yaw);
    double bx = target_armor_xyza(0) + 0.5f * shoot_range * sin(target_yaw);
    double by = target_armor_xyza(1) - 0.5f * shoot_range * cos(target_yaw);
    double angle_a = atan2(ay, ax);
    double angle_b = atan2(by, bx);
    double angle_c = atan2(target_armor_xyza(1), target_armor_xyza(0));
    double allow_fire_ang_max = std::max(angle_a, angle_b) - angle_c;
    double allow_fire_ang_min = std::min(angle_a, angle_b) - angle_c;
    allow_fire_ang_max = tools::limit_rad(allow_fire_ang_max);
    allow_fire_ang_min = tools::limit_rad(allow_fire_ang_min);
    // ★2026-10-08 高速小陀螺 + 靶车往左平移: 开火窗口整体往右挪(yaml high_spin_left_shift, 米, 0 = 关)。
    //   实车: 这种工况弹丸落在板左侧 1~2cm, 部分打出板。窗口宽度不变, 只挪位置:
    //   要等板再往左多转一点(板在云台左边)才开火, 落点相对板往右移。
    //   只在 high_spin 且往左横向车速 >= high_spin_left_shift_min_speed 时生效;
    //   原地小陀螺 / 往右平移 / 慢速 / 不转 完全不变。v_lat > 0 = 往左(yaw 增大 = 左)。
    if (high_spin && high_spin_left_shift_ != 0.0)
    {
      const double los_s = std::atan2(ekf_x[2], ekf_x[0]);
      const double v_lat_s = -ekf_x[1] * std::sin(los_s) + ekf_x[3] * std::cos(los_s);
      if (v_lat_s >= high_spin_left_shift_min_speed_)
      {
        const double dist_s = std::max(0.5, std::hypot(target_armor_xyza(0), target_armor_xyza(1)));
        const double sh = high_spin_left_shift_ / dist_s;  // rad, 正 = 落点往右
        allow_fire_ang_max += sh;
        allow_fire_ang_min += sh;
      }
    }

    // yaw_ang_ref
    double control_delta_angle = tools::limit_rad(atan2(target_armor_xyza(1), target_armor_xyza(0)) - fire_gimbal_yaw);
    suggest_fire = (control_delta_angle < allow_fire_ang_max && control_delta_angle > allow_fire_ang_min);
    // 诊断: 记下每一道门的结果(只写成员变量, 不改行为)
    debug_fire_gate_ = 0;
    debug_fire_win_deg_ = (allow_fire_ang_max - allow_fire_ang_min) * 0.5 * 57.3;
    debug_fire_delta_deg_ = control_delta_angle * 57.3;
    debug_fire_dgim_deg_ = tools::limit_rad(fire_cmd_yaw_ - gimbal_yaw) * 57.3;
    debug_fire_trans_ = false;
    if (!suggest_fire) debug_fire_gate_ |= 1;               // 1 指令/枪口不在窗口
    // ★2026-10-07 平移开火窗口改判(yaml fire_trans_aim_tol_deg)。
    //   【原因】实车 14-39-24 匀速平移: 枪口在板上 83%, 实际开火只有 25%; 不开火的周期里 84% 卡在上面的窗口判断。
    //   窗口是拿"开火时刻(t + 链路延迟 + 出膛延迟)的板预测"去比"本周期指令(按弹丸飞行/激光提前量瞄的)",
    //   两个不同时刻的预测平均差 1.3 度, 窗口只有 ±1.2 度, 六成周期被误判"不在窗口"。
    //   【做法】平移时改成: 指令离板预测 <= tol(只防瞄错板/瞄飞) + 云台离指令 <= fire_cmd_gimbal_tol(上面已判)。
    //   开火中两个门限都放宽 fire_trans_hold_scale 倍, 防止在边上来回翻。小陀螺/静止不走这里。
    if (translating && fire_cmd_yaw_valid_)
    {
      const double k = fire_latched_ ? fire_trans_hold_scale_ : 1.0;
      const double d_aim = tools::limit_rad(fire_cmd_yaw_ - std::atan2(target_armor_xyza(1), target_armor_xyza(0)));
      const double d_gim = tools::limit_rad(fire_cmd_yaw_ - gimbal_yaw);
      suggest_fire = std::abs(d_aim) <= fire_trans_aim_tol_ * k;
      cmd_gimbal_far = std::abs(d_gim) > fire_cmd_gimbal_tol_ * k;
      debug_fire_gate_ = suggest_fire ? 0 : 1;
      debug_fire_win_deg_ = fire_trans_aim_tol_ * k * 57.3;   // 诊断: 这条路径下窗口 = tol
      debug_fire_delta_deg_ = d_aim * 57.3;                   // 诊断: 指令离板预测
      debug_fire_dgim_deg_ = d_gim * 57.3;                    // 诊断: 指令离云台
      debug_fire_trans_ = true;
    }
    if (cmd_gimbal_far) debug_fire_gate_ |= 2;              // 2 云台离指令 > tol
    if (fire_now_t < fire_block_until_t_) debug_fire_gate_ |= 4;  // 4 换板等待
    if (braking) debug_fire_gate_ |= 8;                     // 8 急刹
    if (cmd_gimbal_far) suggest_fire = false;

    if (!outpost_is_make) suggest_fire = 0;

    // 进窗确认: 低速(逐块追板)时, 要求连续在窗口内 fire_confirm_time_ 才开火,
    // 滤掉换板时云台"甩过"装甲板的那一瞬间(即"哒 ... 哒哒哒"里单独那一发)。
    // 高速反陀螺时云台基本不动、板自己扫过窗口, 扫过时间本来就只有十几 ms, 不做确认。
    if (fire_now_t < fire_block_until_t_)
      suggest_fire = false;
    if (braking) suggest_fire = false;

    if (!suggest_fire)
    {
      fire_latched_ = false;
      fire_cand_ = false;
    }
    else if (!fire_latched_)
    {
      if (!fire_cand_)
      {
        fire_cand_ = true;
        fire_cand_start_t_ = fire_now_t;
      }
      if (!low_speed || tools::delta_time(fire_now_t, fire_cand_start_t_) >= fire_confirm_time_)
      {
        fire_latched_ = true;
      }
    }

    return fire_latched_;
  }

  void Planner::update_spin_select(const Target & target)
  {
    const double v = std::abs(target.ekf_x()(7));
    const auto now = this->now();
    if (!spin_select_init_)
    {
      spin_select_vyaw_ = v;
      spin_select_init_ = true;
    }
    else
    {
      // 非对称一阶低通: 转速往上走时慢慢跟(滤掉换板尖峰), 往下走时立刻跟(真减速/停转马上退出高速)。
      //   换板尖峰只会向上冲, 所以这样既能挡住尖峰, 又不会让"停下来了还在用高速打法"。
      //   回放(慢速日志 08-58 / 12-28, tau=0.6): 慢速误判高速 26.1%/22.9% -> 0.3%/0.0%;
      //   快速段判高速 100% 不变, 停转后仍判高速 0%。对称低通 tau=0.6 时停转后误判高速 19%。
      // dt 按真实调用间隔算, 丢目标后(间隔很长)直接重置, 避免沿用旧目标的转速。
      const double dt = tools::delta_time(now, spin_select_t_);
      if (dt > 0.3)
      {
        spin_select_vyaw_ = v;
        spin_select_high_ = v >= spin_enter_high_;
      }
      else if (v < spin_select_vyaw_) spin_select_vyaw_ = v;
      else
      {
        const double a = dt / (spin_select_tau_ + dt);
        spin_select_vyaw_ += a * (v - spin_select_vyaw_);
      }
    }
    spin_select_t_ = now;
    if (!spin_select_high_ && spin_select_vyaw_ >= spin_enter_high_) spin_select_high_ = true;
    else if (spin_select_high_ && spin_select_vyaw_ < spin_exit_high_) spin_select_high_ = false;
  }

  // rbplan 入口已调过 update_spin_select, 这里直接返回本周期的判定(带低通 + 滞回)。
  bool Planner::is_high_spin(const Target &) { return spin_select_high_; }

  Plan Planner::rbplan(Target target, double bullet_speed, double gimbal_yaw)
  {
    update_spin_select(target);
    // 0. Check bullet speed
    if (bullet_speed < 10 || bullet_speed > 25)
    {
      bullet_speed = 22;
    }

    // 1. Predict fly_time
    Eigen::Vector3d xyz;
    auto min_dist = 1e10;
    for (auto & xyza : target.armor_xyza_list())
    {
      auto dist = xyza.head<2>().norm();
      if (dist < min_dist)
      {
        min_dist = dist;
        xyz = xyza.head<3>();
      }
    }
    min_dist += target_dist_error_;
    double target_h = xyz.z();
    target_h += target_h_error_;
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_h);

    // ★2026-09-28 弹丸飞行时间在这里单独预测。
    //   2.5m / 24.4m/s 时 fly_time ≈ 0.104s, 小陀螺 vyaw=5rad/s 时就是约 30 度的提前量 ——
    //   子弹需要这个提前量, 激光(光速)不需要, 否则激光按转速成比例地打在"来板方向"的车身上。
    if (!laser_) target.predict(bullet_traj.fly_time);
    // ★2026-09-28 激光模式低速(逐块追板)时, 云台要一直转着跟板, 从发指令到云台真正转到位有滞后,
    //   激光就落在板运动方向的后面。这里把瞄准点往前预测 laser_aim_lead_ 抵消这段滞后。
    //   高速反陀螺时云台 yaw 基本不动、等板扫过来, 没有跟随滞后, 不加。
    else if (!is_high_spin(target))
      target.predict(laser_aim_lead_);

    // ★2026-10-07 平移时用"直接拟合最近观测"修正预测。
    //   【原因】EKF 是匀速模型, 起步/加速时车速估计慢半拍, 预测落后。离线评估(9 份平移日志, 预测 0.1s 后):
    //     起步加速 EKF 偏差 -0.76 度 / 误差 0.85 度(40% 超过 1 度); 二阶拟合 -0.03 / 0.23(1.4%)
    //     匀速     EKF -0.37 / 0.48; 二阶拟合 +0.03 / 0.20      减速 EKF +0.13 / 0.80; 二阶拟合 +0.18 / 0.33
    //   【做法】target.obs_hist = 同一块板最近的原始观测位置(换板清空)。对 x、y 各做 obs_fit_order 阶最小二乘
    //   (窗口 obs_fit_window 秒), 外推到"本次瞄准对应的时刻"(target 的预测时刻 + 已预测的飞行时间/激光提前量),
    //   得到板的位置和速度, 和 EKF 预测的同一块板比, 差值整体加到车中心位置和速度上。
    //   只在平移(没在转、车速 >= trans_fire_speed)用; 修正量限 obs_fit_max_shift 米, 观测不够就不修。
    //   小陀螺/静止完全不走这里。
    debug_obs_fit_used_ = false;
    {
      const auto xf = target.ekf_x();
      // 本周期运动状态, 只算这一次(见 planner.hpp MotionFrame 注释)。
      motion_.high_spin = is_high_spin(target);
      motion_.speed_raw = std::hypot(xf[1], xf[3]);
      motion_.spinning_now = motion_.high_spin || std::abs(xf[7]) >= spinning_vyaw_;
      const bool spinning = motion_.spinning_now;
      const auto & oh = target.obs_hist;
      const auto al = target.armor_xyza_list();
      const int id = (target.last_id >= 0 && target.last_id < static_cast<int>(al.size())) ? target.last_id : -1;
      // 第二版: 本周期算出的"原始修正量"(不适用时为 0)先低通, 再加到 target 上, 不让瞄准点每帧乱跳。
      //   第一版回放: 修正量每周期跳, 指令每周期变化 0.09 -> 0.13 度, 云台跟不上, 匀速开火 70% -> 30%。
      Eigen::Vector2d raw_shift = Eigen::Vector2d::Zero(), raw_dv = Eigen::Vector2d::Zero();
      bool raw_ok = false;
      if (!spinning && id >= 0 && motion_.speed_raw >= fire_trans_speed_ && oh.size() >= 6)
      {
        const auto t_last = oh.back().t;
        size_t lo = oh.size() - 1;
        while (lo > 0 && tools::delta_time(t_last, oh[lo - 1].t) <= obs_fit_window_) lo--;
        const int n = static_cast<int>(oh.size() - lo);
        const double span = tools::delta_time(t_last, oh[lo].t);
        const int order = obs_fit_order_;
        if (n >= order + 4 && span >= 0.6 * obs_fit_window_)
        {
          Eigen::MatrixXd A(n, order + 1);
          Eigen::MatrixXd b(n, 2);
          for (int i = 0; i < n; i++)
          {
            const double dt = tools::delta_time(oh[lo + i].t, t_last);
            for (int p = 0; p <= order; p++) A(i, p) = std::pow(dt, p);
            b(i, 0) = oh[lo + i].x;
            b(i, 1) = oh[lo + i].y;
          }
          const Eigen::MatrixXd c = A.colPivHouseholderQr().solve(b);
          const double T_lead = !laser_ ? bullet_traj.fly_time : laser_aim_lead_;
          const double d = tools::delta_time(target.last_obs_t(), t_last) + T_lead;  // 从最后一帧观测到瞄准时刻
          Eigen::Vector2d vel = Eigen::Vector2d::Zero();  // 拟合出的瞄准时刻速度
          for (int p = 1; p <= order; p++) vel += c.row(p).transpose() * (p * std::pow(d, p - 1));
          // 只修速度(第三版): 位置还用 EKF 的, 只把"拟合出的速度/加速度"换进来:
          //   shift = (v_fit(现在) - v_ekf) * T + 0.5 * a_fit * T^2
          //   第二版诊断: 直接用拟合位置时, 原始修正量每周期跳 22mm(二阶外推放大了观测噪声), 云台跟不上;
          //   位置截距是噪声大头, 速度/加速度要平稳得多。
          const double d_now = tools::delta_time(target.last_obs_t(), t_last);
          Eigen::Vector2d v_now = Eigen::Vector2d::Zero(), acc = Eigen::Vector2d::Zero();
          for (int p = 1; p <= order; p++) v_now += c.row(p).transpose() * (p * std::pow(d_now, p - 1));
          if (order >= 2) acc = 2.0 * c.row(2).transpose();
          const Eigen::Vector2d shift = (v_now - Eigen::Vector2d(xf[1], xf[3])) * T_lead + 0.5 * acc * T_lead * T_lead;
          if (shift.norm() <= obs_fit_max_shift_)
          {
            Eigen::Vector2d dv = vel - Eigen::Vector2d(xf[1], xf[3]);
            if (dv.norm() > obs_fit_max_dv_) dv *= obs_fit_max_dv_ / dv.norm();
            raw_shift = shift;
            raw_dv = dv;
            raw_ok = true;
          }
        }
      }
      // 低通: 丢目标/换板(obs_hist 被清空)时直接清零; 不适用时朝 0 衰减, 进出平移不会一下跳
      const auto now_f = now();
      const double dtf = tools::delta_time(now_f, obs_fit_last_t_);
      obs_fit_last_t_ = now_f;
      if (spinning || dtf > 0.2 || oh.size() < 2 || id != obs_fit_last_id_)
      {
        // 在转 / 丢目标 / 换板: 立刻清零, 小陀螺完全不受影响
        obs_fit_shift_f_.setZero();
        obs_fit_dv_f_.setZero();
      }
      else
      {
        // 不平滑, 直接用本周期修正量(回放: 平滑会让减速段变准差 1.19 -> 1.7 度)。
        // 写成 f += (raw - f) 而不是 f = raw, 和原来 a = 1.0 时的浮点结果逐位一致。
        obs_fit_shift_f_ += (raw_shift - obs_fit_shift_f_);
        obs_fit_dv_f_ += (raw_dv - obs_fit_dv_f_);
      }
      obs_fit_last_id_ = id;
      debug_obs_fit_raw_ok_ = raw_ok;
      debug_obs_fit_raw_ = raw_shift;
      debug_obs_fit_applied_ = obs_fit_shift_f_;
      if (obs_fit_shift_f_.norm() > 1e-6 || obs_fit_dv_f_.norm() > 1e-6)
      {
        target.shift_center(obs_fit_shift_f_.x(), obs_fit_shift_f_.y(), obs_fit_dv_f_.x(), obs_fit_dv_f_.y());
        debug_obs_fit_used_ = true;
      }
    }

    // 2. Get trajectory
    double yaw0;
    Trajectory traj;
    Eigen::Vector2d yaw_pitch;
    try
    {
      yaw_pitch = rbaim(target, bullet_speed);
      yaw0 = yaw_pitch(0);
      traj = rbget_trajectory(target, yaw0, bullet_speed);
    }
    catch (const std::exception & e)
    {
      tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
      return {false};
    }

    // 3. Solve yaw
    Eigen::VectorXd x0(2);
    x0 << traj(0, 0), traj(1, 0);
    tiny_set_x0(yaw_solver_, x0);

    yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
    tiny_solve(yaw_solver_);

    // 4. Solve pitch
    x0 << traj(2, 0), traj(3, 0);
    tiny_set_x0(pitch_solver_, x0);

    pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
    tiny_solve(pitch_solver_);

    Plan plan;
    plan.control = true;
    // plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);

    plan.target_pitch = traj(2, HALF_HORIZON);

    plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
    plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

    plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
    plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
    plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

    // ★2026-09-27: 修正 debug_xyza（画面红框）显示的是错误时刻的瞄准点。
    //
    // 【原来的问题】debug_xyza 在 rbaim() 内被赋值, 而上面的 rbget_trajectory()
    //   会把 target 从 -0.51s 一路 predict 到 +0.5s, 每步都调一次 rbaim() ——
    //   一次 rbplan() 里 rbaim() 被调 102 次, debug_xyza 被覆盖 102 次,
    //   最终残留的是【目标被预测到 +0.5 秒之后】的位置。
    //   但 MPC 实际采用的是 HALF_HORIZON（轨迹中点, 即"当前应瞄时刻"）,
    //   所以红框画的点和真正发给电控的指令根本不对应。
    //
    // 【实测后果】小陀螺靶 vyaw 约 -3.5 rad/s, 0.5s 转过约 -100 度,
    //   正好跨过一块板(四板间隔 90 度), 红框就跳到另一块板上。
    //   本靶标是"两高两低"两种高度, 于是 aim_z 相对 obs_z 直接变号:
    //   obs_z 约 +0.15 而 aim_z 约 -0.30。日志里 43.8% 的帧 |aim-obs| > 50cm,
    //   平均朝向差 17.8 度 —— 看起来就是"红框乱飞"。
    //
    // 【修法】MPC 求解完成后, 用函数入口那份已 predict(fly_time) 的 target 副本
    //   再调一次 rbaim(), 让 debug_xyza 落在真正被采用的时刻上。
    //   rbaim() 的返回值丢弃, 只要它对 debug_xyza 的副作用。
    //
    // ⚠️ 纯可视化修正: plan.* 在上面已全部取完, 不改变任何发给电控的量。
    {
      Target target_now = target;
      rbaim(target_now, bullet_speed);
      // ★2026-10-04 这一次 rbaim 用的是"当前应瞄时刻"(已 predict fly_time)的目标,
      //   它选中的高速瞄准板就是本周期云台真正在等的那块板, 锁下来给 rbShoot 用。
      fire_lock_armor_id_ = rbaim_high_spin_best_id_;
    }

    // ★2026-09-27 反陀螺模式绕过 MPC, 直接发瞄准点, 速度/加速度前馈置 0。
    //
    // 【为什么】反陀螺时 pitch 参考是两层高度之间的方波(每块板切一次, 约 3.4 度),
    //   MPC 参数 Q=9e6 / R=1 近似 bang-bang, 每个台阶都会输出最大加速度和速度脉冲,
    //   这些前馈直接进电控 SMC(速度项增益很大), 对方波参考毫无意义。
    //   实测开自瞄后视觉发出的 pitch 很干净(-4.8 / -8.1 交替), 云台却摆到 +14.5 度、
    //   又撞到水平下限位, 远超指令范围 —— 先排除前馈这个嫌疑。
    // 【做法】位置用 rbaim() 的直接结果(已在上面 predict(fly_time)),
    //   vel = acc = 0, 让电控自己的位置环去走台阶。
    // 低速(|vyaw| < 90deg/s)仍走 MPC, 不受影响。
    const bool high_spin_plan = is_high_spin(target);
    if (high_spin_plan)
    {
      plan.yaw = yaw_pitch(0);
      plan.yaw_vel = 0;
      plan.yaw_acc = 0;
      // ★2026-10-08 高速反陀螺 + 平移: yaw 速度前馈 = 车中心方位角速度。
      //   【现象】实车 17-54-52 边转边平移, 往左平移开火 22~42%, 往右平移 1~8%, 两边转速估计一样(~2.9), 都在高速档。
      //   【原因】这里前馈固定 0, 电控只能靠位置误差追平移, 云台落后指令 ~80ms(车速 1.2+ 时 ~2 度);
      //   靶车的板在视野里从右往左扫, 锁定的板过正中再往左 leaving_angle(20 度)就换下一块:
      //   往左平移云台落在右边, 板扫过来正好经过云台; 往右平移云台落在左边, 板走不到就换板了,
      //   回放: 往右时换板前一刻板离云台还差 1.5 度。转向反过来时, 打不出的就是往左。
      //   【做法】把车中心的方位角速度告诉电控。它只和整车平移有关, 和 MPC/换板无关,
      //   实车逐帧跳变最大 2.6 deg/s, 不会像 MPC 前馈那样换板时冲 100+ deg/s。
      //   车速 < high_spin_ff_trans_speed_ 时不发: 原地高速小陀螺(车速 ~0.15)不受影响。
      {
        const auto xh = target.ekf_x();
        const double r2h = xh(0) * xh(0) + xh(2) * xh(2);
        if (r2h > 0.04 && std::hypot(xh(1), xh(3)) >= high_spin_ff_trans_speed_)
        {
          double w = (xh(0) * xh(3) - xh(2) * xh(1)) / r2h;  // rad/s, 车中心方位角速度
          if (std::abs(w) > low_speed_yaw_vel_limit_) w = std::copysign(low_speed_yaw_vel_limit_, w);
          plan.yaw_vel = w;
        }
      }
      plan.pitch = yaw_pitch(1);
      plan.pitch_vel = 0;
      plan.pitch_acc = 0;
    }

    // ★2026-10-03 低速 yaw 指令单周期限幅(yaml low_speed_yaw_step_limit_deg)。
    //   原来写在 plan() 里, 但车上跑的是 rbplan(), 所以一直没生效; 现在挪到这里。
    //   【作用】低速追板换板时, rbaim 选到新板, 视觉 yaw 指令会一下跳几度(日志: -3.06 -> +4.17,
    //   云台实际几乎没动), 电控追这个跳变就"甩过头再回来"。限幅后指令按每周期最多 N 度过渡。
    //   限幅生效时清掉 MPC 的 yaw 速度/加速度前馈: 位置被限住而前馈照推, 云台一样会冲过头。
    //   只在低速路径: 高速反陀螺云台基本不动, 指令本来就不跳, 加限幅反而拖慢瞄准。
    //   低速/高速判定用 is_high_spin(), 和本函数其它分支保持一致。
    //   【单位】是"每次 rbplan 调用"最多改多少度。plan_thread 约 5ms 调一次,
    //   0.15 度/次 ≈ 30 度/s, 1.0 度/次 ≈ 200 度/s。
    //   丢目标后 rbplan 不再被调用, 重新捕获时距上次调用超过 0.2s, 下面会直接重新初始化,
    //   不会从丢失前的旧角度慢慢挪过去。
    if (!high_spin_plan)
    {
      const auto now_t = now();
      // 超过 0.2s 没调用(丢目标/没控制)就重新初始化, 直接用当前指令, 不做限幅。
      if (!yaw_command_initialized_ || tools::delta_time(now_t, last_yaw_command_t_) > 0.2)
      {
        last_yaw_command_ = plan.yaw;
        yaw_command_initialized_ = true;
        yaw_switch_state_init_ = false;
      }
      // ★2026-10-07 第二版: 目标在转(|vyaw| >= low_speed_yaw_spin_vyaw, 保持 low_speed_yaw_spin_hold 秒)就完全走 A 版。
      //   【原因】第一版慢速小陀螺也放开了, 实车 17-01-40: 每次换板 MPC 速度前馈冲到 100~176 deg/s,
      //   云台来回抖, 慢速段开火 0%。A 版每周期限幅时顺带把前馈清零, 所以不抖。
      //   换板频率区分不了(平移时 EKF 也频繁换板, 31~58% 周期), 只能用转速, 平移时约 14% 周期被误判成在转,
      //   误判时退回 A 版(稍慢, 但不抖)。转速掉到阈值以下要持续 low_speed_yaw_spin_hold 秒才放开, 防止来回切。
      const double vyaw_abs = std::abs(target.ekf_x()(7));
      if (vyaw_abs >= spinning_vyaw_) last_spin_seen_t_ = now_t;
      const bool spin_mode = tools::delta_time(now_t, last_spin_seen_t_) < low_speed_yaw_spin_hold_;
      if (spin_mode) yaw_switch_state_init_ = false;  // 下次放开时从当前指令开始慢慢追, 不跳
      // ★2026-10-07 限幅加上"车中心方位角本周期的变化"。
      //   【原因】实车 14-59-23: 快速平移(1~1.6m/s)时 EKF 转速被带偏到 0.3~0.9, 进了上面的 A 版限幅,
      //   指令被钉在 0.05 度/周期 = 8.9 deg/s, 板却在以 40~60 deg/s 移动, 1 秒内落后十几度, 板出画面丢目标。
      //   28 次丢目标里 10 次是板从画面边上出去的, 出去前指令都卡在 8.9 deg/s。
      //   【做法】每周期允许的变化 = step_limit + |车中心方位角本周期的变化|:
      //   平移带来的方位变化直接放行, 换板跳变(车中心不动、板跳)照样只走 step_limit。
      //   原地小陀螺车中心基本不动, 和 A 版几乎一样。
      // 第二版: 不用车中心位置的逐周期差分(位置估计每周期在跳, 慢速小陀螺时把限幅放开了, 前馈峰值 52 -> 106 deg/s),
      //   改用 EKF 车中心速度的横向分量算方位角速度 x 周期时长, 而且只在车速 >= low_speed_yaw_follow_min_speed 时生效。
      //   原地/慢速小陀螺车速 < 0.5m/s, 完全是 A 版。
      const auto xc = target.ekf_x();
      double center_step = 0.0;
      const double c_r2 = xc(0) * xc(0) + xc(2) * xc(2);
      const double c_speed = std::hypot(xc(1), xc(3));
      //   注: 这里故意不加 !spin_mode。center_step 只放宽位置限幅, 不发速度前馈, 不会引起抖;
      //   而平移时约 14% 的周期 EKF 转速被带偏到 0.6 以上误判成 spin_mode,
      //   加上之后回放 13-50-38 平移开火 70.4% -> 50.6%, 14-59-23 84.4% -> 72.8%。
      if (c_r2 > 0.04 && c_speed >= yaw_follow_trans_speed_)
      {
        const double az_rate = (xc(0) * xc(3) - xc(2) * xc(1)) / c_r2;  // rad/s, 车中心方位角变化率
        const double dt_cyc = std::clamp(tools::delta_time(now_t, last_yaw_command_t_), 0.0, 0.02);
        // 只放宽"跟车中心同方向"的那一边: 反方向的抖动/换板回跳照样只走 step_limit。
        //   回放 14-37-44(边平移边慢转): 两边都放宽时指令来回锯齿, 前馈冲到 106 deg/s。
        const double want = tools::limit_rad(plan.yaw - last_yaw_command_);
        if (want * az_rate > 0.0) center_step = std::abs(az_rate) * dt_cyc * low_speed_yaw_follow_gain_;
      }
      // ★2026-10-08 限幅生效时, 前馈改成"指令自己的角速度"而不是 0。
      //   【原因】电控 SMC 里 d_error = 实际角速度 - 目标角速度, 前馈给 0 等于告诉电控"目标是静止的",
      //   电控只能靠位置误差积累驱动, 必然稳态落后。实车 15-51-18: 视觉发的 yaw 前馈 63.8% 的周期是 0,
      //   平均只有 2.5 deg/s, 而平移时板实际在以 20~30 deg/s 移动。
      //   【做法】被限幅那一周期, 指令实际走了 step 度, 用时 dt, 所以指令自己的角速度就是 step/dt。
      //   发这个值既和指令一致(不会像 MPC 原始前馈那样位置被限住还猛推), 又不会让电控以为目标静止。
      //   只在"车真的在平移"(车速 >= low_speed_yaw_follow_min_speed)时生效: 原地/慢速小陀螺车速 < 0.5m/s,
      //   前馈照旧清零, 和 A 版一样不抖。
      //   ★2026-10-08 同上, 必须同时 !spin_mode。原来只看车速, 边转边平移时前馈照发:
      //   实车 17-00-08 慢速段车速 >= 0.5 的周期占 59%, 这些周期 100% 发了前馈(平均 25 deg/s),
      //   改之前(16-19-05)只有 0.6; 云台逐周期动 0.34 -> 2.12 度, 顺向开火 0.4%。
      //   10-06 已验证: 慢速小陀螺收到速度前馈就来回抖, 所以 A 版限幅时要顺带清零。
      const bool vel_ff_ok = !spin_mode && c_speed >= yaw_follow_trans_speed_;
      const double vel_dt = std::clamp(tools::delta_time(now_t, last_yaw_command_t_), 0.003, 0.02);
      auto cmd_vel_ff = [&](double step_rad, double sign) {
        double w = step_rad / vel_dt;
        if (w > low_speed_yaw_vel_limit_) w = low_speed_yaw_vel_limit_;
        return std::copysign(w, sign);
      };
      if (spin_mode)
      {
        // A 版原行为: 每周期都限幅
        const double yaw_delta = tools::limit_rad(plan.yaw - last_yaw_command_);
        if (std::abs(yaw_delta) > low_speed_yaw_step_limit_ + center_step)
        {
          const double step = low_speed_yaw_step_limit_ + center_step;
          plan.yaw = tools::limit_rad(last_yaw_command_ + std::copysign(step, yaw_delta));
          plan.yaw_vel = vel_ff_ok ? cmd_vel_ff(step, yaw_delta) : 0.0;
          plan.yaw_acc = 0.0;
        }
        // 放宽生效的周期没被限住时, MPC 前馈会直接放出去; 和放开路径一样限到 low_speed_yaw_vel_limit。
        //   回放 14-37-44: 不限时慢速段前馈峰值 52 -> 91 deg/s。
        if (center_step > 0.0 && std::abs(plan.yaw_vel) > low_speed_yaw_vel_limit_)
          plan.yaw_vel = std::copysign(low_speed_yaw_vel_limit_, plan.yaw_vel);
      }
      else
      {
        // ★2026-10-07 只在换板时限幅。
        //   【原因】A 版每周期都限 0.05 度, 纯平移时 78~91% 的周期被限住, 指令落后 10~17 度, 打不中。
        //   按"单周期变化 > 0.5 度才限"也不行: 急刹/急起时 EKF 速度修正会让瞄准角单周期跳 1~9 度,
        //   和换板跳变一样大, 会被误限(回放: 急刹段 13~28% 周期落后 > 1 度)。
        //   【做法】不看幅度大小, 看"是不是真的换了板"。本周期或上一周期出现下面任一信号算换板:
        //     1. EKF 匹配的板 last_id 变了
        //     2. rbaim 瞄的板变了(瞄准板朝向单周期变 > 30 度, 和 rbShoot 的 armor_switched 同一判据)
        //     3. EKF 重建(update_count_ 变小)
        //   换板且跳变 > low_speed_yaw_switch_jump_deg: 从旧指令开始按 low_speed_yaw_step_limit 追, 并清前馈。
        //   追赶期间每周期最多走"原始瞄准角本周期的变化 + step_limit", 既跟住平移又慢慢追平, 差值不累加。
        //   没换板: 指令 = 原始瞄准角, 不限幅, 前馈不动。
        const double raw_yaw = plan.yaw;
        const double aim_armor_yaw = debug_xyza(3);  // 上面 rbaim(target_now) 刚写入, 本周期瞄准的板朝向
        if (!yaw_switch_state_init_)
        {
          last_raw_yaw_ = raw_yaw;
          last_aim_armor_yaw_ = aim_armor_yaw;
          last_target_id_ = target.last_id;
          last_update_count_ = target.update_count_;
          last_switch_signal_ = false;
          // 刚从 A 版(在转)切过来时, 指令可能还落后原始瞄准角, 从当前指令慢慢追, 不一步跳过去
          yaw_catching_up_ = std::abs(tools::limit_rad(raw_yaw - last_yaw_command_)) > low_speed_yaw_step_limit_;
          yaw_switch_state_init_ = true;
        }
        const bool sig_id = target.last_id != last_target_id_;
        const bool sig_aim = std::abs(tools::limit_rad(aim_armor_yaw - last_aim_armor_yaw_)) > 30. / 57.3;
        const bool sig_rebuild = target.update_count_ < last_update_count_;
        const bool sig_now = sig_id || sig_aim || sig_rebuild;
        const double raw_delta = tools::limit_rad(raw_yaw - last_raw_yaw_);
        const bool switch_jump = (sig_now || last_switch_signal_) && std::abs(raw_delta) > low_speed_yaw_switch_jump_;
        if (switch_jump) yaw_catching_up_ = true;

        if (yaw_catching_up_)
        {
          // 换板跳变那一周期只走 step_limit; 之后跟上原始瞄准角的变化再多追 step_limit
          const double step = switch_jump ? low_speed_yaw_step_limit_ : std::abs(raw_delta) + low_speed_yaw_step_limit_;
          const double err = tools::limit_rad(raw_yaw - last_yaw_command_);
          if (std::abs(err) > step)
          {
            plan.yaw = tools::limit_rad(last_yaw_command_ + std::copysign(step, err));
            plan.yaw_vel = vel_ff_ok ? cmd_vel_ff(step, err) : 0.0;
            plan.yaw_acc = 0.0;
          }
          else
          {
            yaw_catching_up_ = false;  // 已追平, 本周期直接用原始瞄准角
          }
        }
        // 保险: yaw 速度前馈限幅。平移时实车前馈 25~55 deg/s, 换板尖峰 100~176 deg/s。
        //   万一转速判断漏掉, 也不会被一下猛推。只作用在放开的这条路径, A 版路径不变。
        if (std::abs(plan.yaw_vel) > low_speed_yaw_vel_limit_)
          plan.yaw_vel = std::copysign(low_speed_yaw_vel_limit_, plan.yaw_vel);

        last_raw_yaw_ = raw_yaw;
        last_aim_armor_yaw_ = aim_armor_yaw;
        last_target_id_ = target.last_id;
        last_update_count_ = target.update_count_;
        last_switch_signal_ = sig_now;
      }
      last_yaw_command_ = plan.yaw;
      last_yaw_command_t_ = now_t;
    }
    else
    {
      yaw_command_initialized_ = false;
      yaw_switch_state_init_ = false;
    }

    // 前哨站迭代限制
    if (target.name == ArmorName::outpost)
    {
      double vz = target.ekf_x()(5);  // 获取当前前哨站中心Z轴坐标
      auto now = this->now();

      // 如果Z轴变化幅度大于指定阈值（例如0.05米），重置基准和计时器，并禁止开火
      if (vz > 0.09)
      {
        outpost_z_stable_start_time_ = now;
        outpost_is_make = false;
      }
      else
      {
        // 跟踪更新次数不够 500 次(前哨站还没收敛), 不开火
        if (target.update_count_ < 500)
        {
          outpost_is_make = false;
        }
        else
        {
          outpost_is_make = true;
        }
      }

      if (!outpost_is_make)
      {
        Eigen::Vector2d yaw_pitch_nan = heroaim(target, 100000, gimbal_yaw);
        plan.yaw = yaw_pitch_nan(0);
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;

        plan.pitch = yaw_pitch_nan(1);
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
      }
    }

    // 开火判定再往前预测: 链路延迟 fire_link_latency(拍照 -> 电控收到) + 出膛延迟 shoot_latency。两种模式一样。
    // ★2026-10-04 shoot_latency_: 电控收到 mode==2 -> 拨盘推弹 -> 弹丸过摩擦轮(出膛) 的机械延迟。
    //   电控 Ozone dbg_shot_lat_avg 实测 62.9ms(min 59 / max 65, 7 颗)。
    //   原来只补了链路延迟(拍照->电控收到), 没补这一段, 8.5rad/s 时板多转 ~30 度, 弹丸偏晚。
    //   开火判定往前多预测这一段, 下令时刻 = 弹丸出膛时板正好在窗口里。
    // ★2026-10-05 shoot_latency 按距离线性插值(yaml shoot_latency_near/far + *_dist)。
    //   弹速固定 24.4 不改, 但小陀螺实测 4m 要 45ms、2m 要 55ms 才打中心 ——
    //   时间误差随距离变(可能来自阻力系数/测距/远处转速估计), 这里直接按实测两点插值补偿。
    //   min_dist 是上面算 fly_time 用的同一个水平距离。
    const double k_lat = (min_dist - shoot_latency_near_dist_) / (shoot_latency_far_dist_ - shoot_latency_near_dist_);
    const double shoot_latency_now = std::max(0.0, shoot_latency_near_ + k_lat * (shoot_latency_far_ - shoot_latency_near_));
    target.predict(fire_link_latency_ + shoot_latency_now);
    fire_cmd_yaw_ = tools::limit_rad(plan.yaw - yaw_offset_);  // 本周期发给电控的 yaw, 和 gimbal_yaw 同一坐标
    fire_cmd_yaw_valid_ = true;
    plan.fire = rbShoot(target, (gimbal_yaw) / 57.3 - yaw_offset_);
    fire_cmd_yaw_valid_ = false;

    plan.target_yaw = (aim_target_yaw + yaw_offset_) * 57.3;

    return plan;
  }

  void Planner::setup_yaw_solver(const std::string & config_path)
  {
    auto yaml = tools::load(config_path);
    auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
    auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
    auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

    Eigen::MatrixXd A{{1, DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
    Eigen::Matrix<double, 1, 1> R(R_yaw.data());
    tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
    tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

    yaw_solver_->settings->max_iter = 10;
  }

  void Planner::setup_pitch_solver(const std::string & config_path)
  {
    auto yaml = tools::load(config_path);
    auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
    auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
    auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

    Eigen::MatrixXd A{{1, DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
    Eigen::Matrix<double, 1, 1> R(R_pitch.data());
    tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
    tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

    pitch_solver_->settings->max_iter = 10;
  }

  Eigen::Matrix<double, 2, 1> Planner::rbaim(const Target & target, double bullet_speed)
  {
    Eigen::Vector3d xyz;
    double yaw;
    auto min_dist = 1e10;

    Eigen::VectorXd ekf_x = target.ekf_x();
    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    auto armor_num = armor_xyza_list.size();
    // 每块板朝向相对整车中心方位的夹角; 0 = 板正对我方
    std::vector<std::pair<int, double>> armorId_delta_list;
    armorId_delta_list.reserve(armor_num);

    // 整车旋转中心的球坐标yaw
    auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

    for (int i = 0; i < armor_num; i++)
    {
      auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
      armorId_delta_list.emplace_back(std::make_pair(i, delta_angle));
    }

    for (auto & xyza : target.armor_xyza_list())
    {
      auto dist = xyza.head<2>().norm();
      if (dist < min_dist)
      {
        min_dist = dist;
        xyz = xyza.head<3>();
        yaw = xyza[3];
      }
    }

    // 快慢判定统一走 is_high_spin(): rbaim 在一次 rbplan 里被轨迹生成调用约 100 次,
    // 用同一个低通判定保证整条轨迹走同一种打法, 不会在轨迹中间切换。
    const bool high_spin = is_high_spin(target);
    if (!high_spin && target.last_id >= 0 &&
        target.last_id < static_cast<int>(armor_xyza_list.size()))
    {  // 判断当前看到的装甲板在预测时间之后是否还在视野内
      const double spin_sign = ekf_x(7) >= 0 ? 1.0 : -1.0;
      const double d = armorId_delta_list[target.last_id].second * spin_sign;
      // ★2026-10-03 低速只在 [-low_speed_coming_angle, +low_speed_leaving_angle] 范围内追同一块板,
      //   超出就换下一块。原来是 [-60, +20] 度, 一块板追 80 度, 太大。
      if (d <= low_speed_leaving_angle_ && d >= -low_speed_coming_angle_)
      {
        min_dist = armor_xyza_list[target.last_id].head<2>().norm();
        xyz = armor_xyza_list[target.last_id].head<3>();
        yaw = armor_xyza_list[target.last_id](3);
      }
    }

    // ★2026-09-27 反陀螺(第二版): |vyaw| >= 90deg/s 时 yaw 固定, pitch 按"下一块转到正面的板"选高度。
    //
    // 【为什么不能逐块追板】实测 6020 靶 vyaw 约 -5.5 rad/s(315 deg/s), 四块板每 0.29s 换一块。
    //   原逻辑落回"水平距离最近的板", yaw 每 0.29s 甩约 7 度、pitch 在两层高度间跳,
    //   MPC 又近似 bang-bang(Q/R=9e6) -> 云台狂震。
    //
    // 【为什么第一版(瞄中心 + 高度取 x[4])也不行】
    //   x[4] 只是 id 0/2 那一层的高度。EKF 关联时两层板的 id 奇偶会对调,
    //   x[4] 就在两层之间被来回拉并过冲, 实测发出的 pitch 在 0 度(限幅) 和 -10 度之间每 0.45s 跳一次。
    //   而且只瞄一层, 另一层的板永远打不到。
    //
    // 【本版做法】
    //   yaw  : 瞄"车中心 - r * 指向我方单位向量", 即板正对我方时经过的点, 云台 yaw 基本不动。
    //   pitch: 取【下一块要转到正对位置的那块板】自己的 z(armor_xyza_list[i][2]),
    //          高板来就瞄高、低板来就瞄低 —— 两层都打。用每块板自己的 z,
    //          所以即使 EKF 把两层 id 对调, 瞄的高度仍跟随真实那块板。
    //   换板 : d = (板朝向 - 中心方位) * sign(vyaw), d<0 正在转来, d=0 正对, d>0 已转过。
    //          只在 d <= leaving_angle 的板里取 d 最大的那块(即还没转离、最靠前的那块)。
    //          当前板转过正对 leaving_angle(yaml 20 度)就切到下一块, 下一块此时在 20-90=-70 度,
    //          到进入可打范围(约 -30 度)前 pitch 有约 40/315 = 0.13s 调整到位。
    //   注意: rbplan 在调 rbaim 之前已 predict(fly_time), 所以这里的板朝向是【子弹到达时刻】的。
    //
    if (!high_spin) rbaim_high_spin_best_id_ = -1;
    if (high_spin)
    {
      const double cx = ekf_x[0];
      const double cy = ekf_x[2];
      const double center_azim = std::atan2(cy, cx);
      const double spin_sign = ekf_x(7) > 0 ? 1.0 : -1.0;

      int best = -1;
      double best_d = -1e10;
      for (int i = 0; i < static_cast<int>(armor_num); i++)
      {
        double d = tools::limit_rad(armor_xyza_list[i][3] - center_azim) * spin_sign;
        if (d > leaving_angle_) continue;  // 已转离正对位置超过 leaving_angle, 不再瞄
        if (d > best_d)
        {
          best_d = d;
          best = i;
        }
      }

      rbaim_high_spin_best_id_ = best;   // 记下瞄的是哪块板, rbShoot 高速开火判定用同一块
      if (best >= 0)
      {
        const auto & p = armor_xyza_list[best];
        const double r_i = std::hypot(p[0] - cx, p[1] - cy);  // 该板自身半径(两层半径可能不同)
        xyz = Eigen::Vector3d(cx - r_i * std::cos(center_azim), cy - r_i * std::sin(center_azim), p[2]);
        min_dist = xyz.head<2>().norm();
        yaw = center_azim;  // 仅用于画面显示: 正对我方的板面朝向
      }
    }

    debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

    auto azim = std::atan2(xyz.y(), xyz.x());
    auto now_pitch_offset = pitch_offset_;

    // 激光走直线: pitch 就是视线角, 不加重力下坠补偿(2.5m 时约 1.2 度)。
    if (laser_) return {tools::limit_rad(azim + yaw_offset_), std::atan2(xyz.z(), min_dist) + 0.0};

    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

    return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + now_pitch_offset};
  }

  Eigen::Matrix<double, 2, 1> Planner::heroaim(const Target & target, double bullet_speed, double gimbal_yaw)
  {
    Eigen::Vector3d xyz;
    double yaw;
    auto min_dist = 1e10;

    Eigen::VectorXd ekf_x = target.ekf_x();
    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    auto armor_num = armor_xyza_list.size();
    // 每块板朝向相对整车中心方位的夹角; 0 = 板正对我方
    std::vector<std::pair<int, double>> armorId_delta_list;
    armorId_delta_list.reserve(armor_num);

    // 整车旋转中心的球坐标yaw
    auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

    for (int i = 0; i < armor_num; i++)
    {
      auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
      armorId_delta_list.emplace_back(std::make_pair(i, delta_angle));
    }

    for (auto & xyza : target.armor_xyza_list())
    {
      auto dist = xyza.head<2>().norm();
      if (dist < min_dist)
      {
        min_dist = dist;
        xyz = xyza.head<3>();
        yaw = xyza[3];
      }
    }

    double abs_vyaw = abs(ekf_x(7));
    if (abs_vyaw < 90. / 57.3 && armorId_delta_list[target.last_id].second < 60. / 57.3)
    {  // 判断当前看到的装甲板在预测时间之后是否还在视野内
      min_dist = armor_xyza_list[target.last_id].head<2>().norm();
      xyz = armor_xyza_list[target.last_id].head<3>();
      yaw = armor_xyza_list[target.last_id](3);
    }

    auto r = target.ekf_x()(8);
    auto v_yaw = target.ekf_x()(7);

    // 旋转中心的坐标
    auto center_x = target.ekf_x()(0);
    auto center_y = target.ekf_x()(2);

    auto direction_yaw = atan2(center_y, center_x);
    auto aim_point_x = center_x - r * std::cos(direction_yaw);
    auto aim_point_y = center_y - r * std::sin(direction_yaw);
    auto aim_point_z = xyz.z();

    if (abs(v_yaw) < 1)
    {
      aim_point_x = xyz.x();
      aim_point_y = xyz.y();
    }
    auto min_dist1 = min_dist;
    if (target.name == ArmorName::outpost)
    {
      Target target_pitch = target;
      min_dist1 = 1e10;
      Eigen::Vector3d xyz1;
      double yaw1;
      double max_h_armor = 10e-6, min_h_armor = 10e+6;
      size_t max_armor_id, min_armor_id;
      target_pitch.predict(tower_pitch_prediction_time_);
      // for (auto & xyza : target.armor_xyza_list()) {
      //   auto dist = xyza.head<2>().norm();
      //   if (dist < min_dist1) {
      //     min_dist1 = dist;
      //     xyz1 = xyza.head<3>();
      //     yaw1 = xyza[3];
      //   }
      //   if(max_h_armor < xyza(2)) {
      //     max_h_armor = xyza(2);
      //   }
      // }
      for (int i = 0; i < 3; i++)
      {
        auto xyza = target.armor_xyza_list()[i];
        auto dist = xyza.head<2>().norm();
        if (dist < min_dist1)
        {
          min_dist1 = dist;
          xyz1 = xyza.head<3>();
          yaw1 = xyza[3];
        }
        if (max_h_armor < xyza(2))
        {
          max_h_armor = xyza(2);
          max_armor_id = i;
        }
        if (min_h_armor > xyza(2))
        {
          min_h_armor = xyza(2);
          min_armor_id = i;
        }
      }
      size_t middle_armor_id = 0;
      for (int i = 0; i < 3; i++)
      {
        if (min_armor_id != i && max_armor_id != i) middle_armor_id = i;
      }
      // if(abs(max_h_armor - target.ekf_x()(4)) < 0.05) aim_point_z =
      // else aim_point_z = target.ekf_x()(4);
      aim_point_z = target.armor_xyza_list()[middle_armor_id](2);
    }

    //补偿距离和补偿高度
    double comp_dist = 0;
    double comp_h = 0;
    min_dist = sqrt(aim_point_x * aim_point_x + aim_point_y * aim_point_y);

    debug_xyza = Eigen::Vector4d(aim_point_x, aim_point_y, aim_point_z, yaw);

    auto azim = std::atan2(aim_point_y, aim_point_x);
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist1 - comp_dist, aim_point_z - comp_h);
    if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

    auto now_pitch_offset = pitch_offset_;

    return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + now_pitch_offset};
  }

  Trajectory Planner::rbget_trajectory(Target target, double yaw0, double bullet_speed)
  {
    Trajectory traj;

    target.predict(-DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = rbaim(target, bullet_speed);

    target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
    auto yaw_pitch = rbaim(target, bullet_speed);

    for (int i = 0; i < HORIZON; i++)
    {
      target.predict(DT);
      auto yaw_pitch_next = rbaim(target, bullet_speed);

      auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
      auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

      traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

      yaw_pitch_last = yaw_pitch;
      yaw_pitch = yaw_pitch_next;
    }

    return traj;
  }

}  // namespace auto_aim
