#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

  // 定义运动状态枚举
  enum class MotionState
  {
    TRANSLATION,          // 纯平移
    IN_PLACE_ROTATION,    // 原地旋转
    TRANSLATION_ROTATION  // 平移 + 旋转
  };

  class Target
  {
    public:
    ArmorName name;
    ArmorType armor_type;
    ArmorPriority priority;
    bool jumped;
    int last_id;  // debug only
    bool predict_extrap_ = false;  // ★2026-10-05 true = 这是 planner 的外推副本, predict 时套用平移速度死区
    Eigen::Vector3d xyz_in_world;

    Target() = default;
    Target(const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, Eigen::VectorXd P0_dig);
    Target(double x, double vyaw, double radius, double h);

    void predict(std::chrono::steady_clock::time_point t);
    void predict(double dt);
    void update(const Armor & armor);

    Eigen::VectorXd ekf_x() const;
    const tools::ExtendedKalmanFilter & ekf() const;
    std::vector<Eigen::Vector4d> armor_xyza_list() const;

    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

    bool diverged() const;

    bool convergened();

    bool isinit = false;

    bool checkinit();

    inline Eigen::VectorXd getEKFXest() { return ekf_.x; }

    inline std::chrono::steady_clock::time_point getTimePoint() { return t_; }

    //前哨站
    std::pair<bool, double> tower_armor_hs[3] = {std::pair<bool, double>(false, 0), std::pair<bool, double>(false, 0), std::pair<bool, double>(false, 0)};
    // double tower_armor_hs[3] = {0,0,0};
    double tower_armor_h;
    double tower_armor_hs_datas[3] = {0, 0, 0};
    double last_tower_armor_h[3] = {0, 0, 0};
    int tower_armor_hs_datas_ptr[3] = {0, 0, 0};

    //长短焦
    bool cam_is_short = true;
    bool last_cam_is_short = true;
    std::chrono::steady_clock::time_point cam_is_switch_time_point;  //相机切换时间点；

    int update_count_;

    private:
    int armor_num_;
    int switch_count_;

    bool is_switch_, is_converged_;

    // 单一 EKF 实例
    tools::ExtendedKalmanFilter ekf_;

    std::chrono::steady_clock::time_point t_;

    // 记录当前的运动状态
    MotionState motion_state_ = MotionState::TRANSLATION;

    void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

    public:
    // ★2026-10-03 空窗后转速修正限幅 (分支 fix/ekf-switch-vyaw, yaml ekf_gap_vyaw_limit, 缺省关)
    //   详细原因见 target.cpp update_ypda() 末尾的注释。
    struct GapVyawConfig
    {
      bool enable = false;
      double gap_s = 0.06;      // 距上次观测超过这么久, 才算"空窗后第一帧"
      double max_dvyaw = 0.3;   // 这一帧 vyaw 最多改多少(rad/s)
      double vyaw_q = 0;        // >0 时覆盖转速过程噪声 v2(原值 400); 0 = 原行为
      // ★板朝向观测噪声 r2_angle 的整体放大倍数, 所有帧生效, 1.0 = 原行为。
      //   静止实测: 角点水平偏 1 像素, PnP 朝向就差约 10 度; 正对静止时 PnP 原始解 -3.5 度,
      //   optimize_yaw 后变 -10.9 度。朝向观测本身噪声远大于原 R 设定, 应少信它、多信方位角。
      double angle_r_scale = 1.0;
      double angle_r_max_vyaw = 1e9;  // rad/s, 只在 |vyaw| 低于它时放大, 缺省不限
      int angle_r_warmup = 0;         // 新建目标前多少次更新不放大(让滤波器先锁住转速)
      // 残差门控: 朝向残差 |观测-预测| 超过 angle_r_gate(rad) 说明目标真的在加速转,
      // 不是 1 像素的噪声(噪声不会累积, 真实转速差会逐帧累积), 此后 angle_r_bypass_frames
      // 帧不放大, 让 EKF 正常追上新转速。缺省 1e9 = 不门控。
      double angle_r_gate = 1e9;
      int angle_r_bypass_frames = 30;
      // ★2026-10-05 平移速度死区(m/s): 往前外推时, 车中心水平合速度低于它就不外推平移(只外推旋转)。
      //   只影响"往未来预测"(dt>0), 不改 EKF 自身的状态/协方差。0 = 关闭(原行为)。
      double predict_v_deadband = 0.0;
      double angle_r_min_vyaw = 0.0;  // rad/s, |vyaw| 低于它(目标基本没在转)时不放大
    };
    static GapVyawConfig gap_vyaw_cfg;

    private:
    std::chrono::steady_clock::time_point last_update_t_{};  // 上一次观测更新的时刻
    int angle_r_bypass_left_ = 0;  // 残差门控触发后, 还剩多少帧不放大朝向噪声
    double angle_res_ema_ = 0.0;   // 朝向残差的指数平均(rad), 换板清零
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP