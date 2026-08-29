#include "target.hpp"

#include <cmath>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 物理常量定义
constexpr double TOWER_ARMOR_DH = 0.10;   // 前哨站两个装甲板之间的标准高低差(m)
constexpr double TOWER_ARMOR_DTB = 0.16;  // 前哨装甲大跳变阈值(m)
constexpr double TOWER_ARMOR_XTB = 0.05;  // 前哨装甲小跳变阈值(m)

namespace auto_aim
{

  Target::Target(const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, Eigen::VectorXd P0_dig)
  : name(armor.name),
    armor_type(armor.type),
    jumped(false),
    last_id(0),
    update_count_(0),
    armor_num_(armor_num),
    t_(t),
    is_switch_(false),
    is_converged_(false),
    switch_count_(0),
    motion_state_(MotionState::TRANSLATION)  // 默认初始状态为平移模型
  {
    auto r = radius;
    priority = armor.priority;
    const Eigen::VectorXd & xyz = armor.xyz_in_world;
    const Eigen::VectorXd & ypr = armor.ypr_in_world;

    // 根据当前装甲板位置和半径，反推旋转中心的坐标
    auto center_x = xyz[0] + r * std::cos(ypr[0]);
    auto center_y = xyz[1] + r * std::sin(ypr[0]);
    auto center_z = xyz[2];

    if (name == ArmorName::outpost)
    {
      tower_armor_hs[0].first = true;       // 标记 0 号位已成功初始化
      tower_armor_hs[0].second = center_z;  // 记录真实高度
    }

    cam_is_switch_time_point = std::chrono::steady_clock::time_point{};

    // ==========================================
    // EKF 11维状态向量定义:
    // [0]x, [1]vx, [2]y, [3]vy, [4]z, [5]vz,
    // [6]yaw(偏航角), [7]vyaw(自转角速度),
    // [8]r(基础半径), [9]r_(半径补偿量), [10]z_(高度补偿量)
    // ==========================================
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);

    // 如果是前哨站，将 z_ (x[10]) 的初始值设为物理理论值
    double initial_dz = (name == ArmorName::outpost) ? TOWER_ARMOR_DH : 0.0;

    x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, initial_dz;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11) * 10.0;
    P0.block(0, 0, 11, 11) = P0_dig.asDiagonal();

    // 自定义状态加法，确保角度(Yaw)在 -PI 到 PI 之间
    auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
      Eigen::VectorXd c = a + b;
      c[6] = tools::limit_rad(c[6]);
      return c;
    };

    ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  }

  // 供手动初始化使用的构造函数
  Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4), motion_state_(MotionState::TRANSLATION)
  {
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
    x0 << x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h;

    Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11) * 10.0;
    P0.block(0, 0, 11, 11) = P0_dig.asDiagonal();

    auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
      Eigen::VectorXd c = a + b;
      c[6] = tools::limit_rad(c[6]);
      return c;
    };

    ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  }

  void Target::predict(std::chrono::steady_clock::time_point t)
  {
    auto dt = tools::delta_time(t, t_);
    predict(dt);
    t_ = t;
  }

  void Target::predict(double dt)
  {
    double vyaw = std::abs(ekf_.x[7]);
    double v_linear = std::hypot(ekf_.x[1], ekf_.x[3]);  // 计算XY方向合成线速度

    // 状态机滞回阈值配置
    const double OMEGA_HIGH = 3;   // 进入旋转的角速度阈值 (rad/s)
    const double OMEGA_LOW = 1.5;  // 退出旋转的角速度阈值 (rad/s)
    const double V_HIGH = 0.6;     // 进入平移旋转的线速度阈值 (m/s)
    const double V_LOW = 0.3;      // 退出平移旋转的线速度阈值 (m/s)

    // ================= 运动状态转移逻辑 =================
    // 核心目的是根据车辆当前的平移和自转速度，动态调整过程噪声Q，使得滤波器既能跟得紧，又不会乱漂
    switch (motion_state_)
    {
      case MotionState::TRANSLATION:
        if (vyaw > OMEGA_HIGH)
        {
          if (v_linear > V_HIGH)
            motion_state_ = MotionState::TRANSLATION_ROTATION;
          else
            motion_state_ = MotionState::IN_PLACE_ROTATION;
        }
        break;

      case MotionState::IN_PLACE_ROTATION:
        if (vyaw < OMEGA_LOW)
        {
          motion_state_ = MotionState::TRANSLATION;
        }
        else if (v_linear > V_HIGH)
        {
          motion_state_ = MotionState::TRANSLATION_ROTATION;
        }
        break;

      case MotionState::TRANSLATION_ROTATION:
        if (vyaw < OMEGA_LOW)
        {
          motion_state_ = MotionState::TRANSLATION;
        }
        else if (v_linear < V_LOW)
        {
          motion_state_ = MotionState::IN_PLACE_ROTATION;
        }
        break;
    }

    // 11维基础转移矩阵 F (x = F*x)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;
    F(6, 7) = dt;

    double v1, v2;
    if (name == ArmorName::outpost)
    {
      this->ekf_x()(10) = TOWER_ARMOR_DH;
      // 前哨站位置固定，收敛后极大限制平移噪声
      if (this->convergened())
      {
        v1 = 0.1;  // 锁死 X, Y, Z 中心
      }
      else
      {
        v1 = 20;  // 允许前期寻找中心
      }
      v2 = 0.1;  // 允许自转速度存在微小波动
    }
    else
    {
      // ★注意：三个分支的 v1/v2 取值**完全相同**（都是 100/400）。
      // 也就是说上面那个运动状态机（TRANSLATION / IN_PLACE_ROTATION / TRANSLATION_ROTATION）
      // 目前对 Q 没有产生任何实际影响——它照常迁移状态，但分配出来的噪声一模一样，
      // 三行尾注（"灵活平移"/"抑制平移漂移"/"全部放开"）描述的是设计意图，不是当前行为。
      // 这是留给后人调参的骨架：真想让状态机起作用，就把这三组值按注释的意图调开。
      // （motion_state_ 并非全无用处，update_ypda 里旋转态会放大 r2_angle，那处是活的。）
      // 根据状态机分配不同的噪声
      switch (motion_state_)
      {
        case MotionState::TRANSLATION:
          v1 = 100;
          v2 = 400;  // 灵活平移
          break;
        case MotionState::IN_PLACE_ROTATION:
          v1 = 100;
          v2 = 400;  // 抑制平移漂移，紧跟自转
          break;
        case MotionState::TRANSLATION_ROTATION:
          v1 = 100;
          v2 = 400;  // 高机动状态，全部放开
          break;
      }
    }

    // 构造过程噪声矩阵 Q
    auto a_ = dt * dt * dt * dt / 4;
    auto b_ = dt * dt * dt / 2;
    auto c_ = dt * dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    Q(0, 0) = a_ * v1;
    Q(0, 1) = b_ * v1;
    Q(1, 0) = b_ * v1;
    Q(1, 1) = c_ * v1;  // X
    Q(2, 2) = a_ * v1;
    Q(2, 3) = b_ * v1;
    Q(3, 2) = b_ * v1;
    Q(3, 3) = c_ * v1;  // Y
    Q(4, 4) = a_ * v1;
    Q(4, 5) = b_ * v1;
    Q(5, 4) = b_ * v1;
    Q(5, 5) = c_ * v1;  // Z
    Q(6, 6) = a_ * v2;
    Q(6, 7) = b_ * v2;
    Q(7, 6) = b_ * v2;
    Q(7, 7) = c_ * v2;  // Yaw

    auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      Eigen::VectorXd x_prior = F * x;
      x_prior[6] = tools::limit_rad(x_prior[6]);
      return x_prior;
    };

    // 前哨站收敛后限制最大转速防飞
    if (this->convergened() && this->name == ArmorName::outpost)
    {
      if (std::abs(this->ekf_.x[7]) > 2) this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;
    }

    ekf_.predict(F, Q, f);
  }

  void Target::update(const Armor & armor)
  {
    int id = 0;

    if (this->name == ArmorName::outpost)
    {
      // 【策略 A：前哨站专用】
      // 纯几何匹配(距离+复合角度)，绕开因高度阶梯跳变导致 EKF 协方差波动的干扰
      auto min_angle_error = 1e10;
      const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

      this->ekf_x()(10) = TOWER_ARMOR_DH;

      std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
      for (int i = 0; i < armor_num_; i++)
      {
        xyza_i_list.push_back({xyza_list[i], i});
      }

      // 按距离(ypd[2])由近及远排序
      std::sort(xyza_i_list.begin(), xyza_i_list.end(), [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
        Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
        Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
        return ypd1[2] < ypd2[2];
      });

      // 只取最近的3个装甲板验证角度匹配度
      for (int i = 0; i < 3; i++)
      {
        const auto & xyza = xyza_i_list[i].first;
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));

        auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) + std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

        if (std::abs(angle_error) < std::abs(min_angle_error))
        {
          id = xyza_i_list[i].second;
          min_angle_error = angle_error;
        }
      }
    }
    else
    {
      // 【策略 B：其他兵种通用】
      // 马氏距离匹配 + 迟滞防抖，有效应对平移带来的透视形变
      auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
      auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);

      auto r2_azimuth = 4e-3;
      auto r2_pitch = 4e-3;
      auto r2_angle = log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2;
      auto r2_d = log(std::abs(delta_angle) + 1) + 1;

      // 处理镜头长短焦切换时的噪声激增
      if (last_cam_is_short != cam_is_short)
      {
        cam_is_switch_time_point = std::chrono::steady_clock::now();
        last_cam_is_short = cam_is_short;
      }
      if (last_cam_is_short)
      {
        // tools::logger()->info("[Target] last_cam_is_short");
      }
      auto now = std::chrono::steady_clock::now();
      double cam_is_switch_lter_dt = tools::delta_time(now, cam_is_switch_time_point);
      if (cam_is_switch_lter_dt < 0.7 && update_count_ > 50)
      {
        r2_azimuth = 4e+4;
        r2_angle *= 300;
        r2_d *= 300;
      }

      Eigen::VectorXd R_dig{{r2_azimuth, r2_pitch, r2_d, r2_angle}};
      Eigen::MatrixXd R = R_dig.asDiagonal();

      const Eigen::VectorXd & ypd = armor.ypd_in_world;
      const Eigen::VectorXd & ypr = armor.ypr_in_world;
      Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

      int best_id = 0;
      double min_mahalanobis_dist = 1e10;
      std::vector<double> md_list(armor_num_, 1e10);

      for (int i = 0; i < armor_num_; i++)
      {
        Eigen::VectorXd xyz_pred = h_armor_xyz(ekf_.x, i);
        Eigen::VectorXd ypd_pred = tools::xyz2ypd(xyz_pred);
        auto angle_pred = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
        Eigen::VectorXd z_pred{{ypd_pred[0], ypd_pred[1], ypd_pred[2], angle_pred}};

        Eigen::VectorXd y = z - z_pred;
        y[0] = tools::limit_rad(y[0]);
        y[1] = tools::limit_rad(y[1]);
        y[3] = tools::limit_rad(y[3]);

        // 【马氏距离是什么】"考虑了不确定性的距离"。
        // 普通欧氏距离把每个方向一视同仁；马氏距离先用协方差 S 做归一化，
        // 于是"在本来就不确定的方向上差一点"惩罚小，"在很确定的方向上差一点"惩罚大。
        // 这正是我们要的：距离测不准，所以距离差一点可以容忍；角度测得准，角度差一点就很可疑。
        // S = H*P*H' + R 是"预测的不确定性 + 观测的不确定性"，即残差本该有多大的方差。
        Eigen::MatrixXd H = h_jacobian(ekf_.x, i);
        Eigen::MatrixXd S = H * ekf_.P * H.transpose() + R;

        double mahalanobis_dist = y.transpose() * S.inverse() * y;
        md_list[i] = mahalanobis_dist;

        if (mahalanobis_dist < min_mahalanobis_dist)
        {
          min_mahalanobis_dist = mahalanobis_dist;
          best_id = i;
        }
      }

      // 迟滞防抖动（Hysteresis）机制：倾向于保持上一次匹配的ID
      //
      // 【为什么需要迟滞】小陀螺时两块板的马氏距离可能非常接近，如果每帧都取最小值，
      // 匹配 ID 会在两块板之间来回跳（0->1->0->1），每次跳都被当成"换板"事件，
      // EKF 的 yaw 也跟着来回拽，跟踪就废了。
      // 做法：只要上一次那块板"还算合理"（马氏距离 < 卡方门限），就要求新候选必须
      // 明显更好（好过它至少 HYSTERESIS_MARGIN=5.0）才允许换，否则维持原判。
      // 这和电控里给按键/阈值加迟滞回环是同一个套路。
      //
      // CHI_SQ_THRESHOLD = 9.488 是自由度 4、置信度 95% 的卡方分位点
      //（观测是 4 维，所以自由度取 4）。含义：如果观测真的属于这块板，
      // 那么马氏距离超过 9.488 的概率只有 5%，超了就认为"这不是同一块板"。
      // ★这个值写死在代码里，不在 yaml。
      // 顺带一提：tools/extended_kalman_filter.cpp 里做 NIS 检验时本该用同一个 9.488，
      // 但原先写的是 0.711（自由度 4 的 5% 分位，方向刚好相反），已改回 9.488——见那里的注释。
      id = best_id;
      double CHI_SQ_THRESHOLD = 9.488;
      double HYSTERESIS_MARGIN = 5.0;

      if (md_list[last_id] < CHI_SQ_THRESHOLD)
      {
        if (min_mahalanobis_dist > md_list[last_id] - HYSTERESIS_MARGIN)
        {
          id = last_id;
        }
      }
    }

    if (id != 0) jumped = true;

    // 检测换板事件
    if (id != last_id)
    {
      is_switch_ = true;
      switch_count_++;

      // 换板时，将上一块装甲板的历史累加数据计算为平均高度锚点
      if (name == ArmorName::outpost)
      {
        if (tower_armor_hs_datas_ptr[last_id] > 0)
        {
          tower_armor_hs[last_id].first = true;  // 标记该装甲板已有有效的历史数据
          tower_armor_hs[last_id].second = tower_armor_hs_datas[last_id] / tower_armor_hs_datas_ptr[last_id];
        }
      }
    }
    else
    {
      is_switch_ = false;
    }

    // 累加当前块装甲板的高度特征
    if (name == ArmorName::outpost)
    {
      double a = 0.1;  // 互补滤波系数
      tower_armor_h = a * armor.xyz_in_world[2] + (1 - a) * last_tower_armor_h[id];

      tower_armor_hs_datas[id] += tower_armor_h;
      last_tower_armor_h[id] = tower_armor_h;
      tower_armor_hs_datas_ptr[id]++;

      // 历史高度数据保护机制，防止长时间追踪导致累加溢出
      if (tower_armor_hs_datas[id] > 10000)
      {
        tower_armor_hs_datas[id] = (tower_armor_hs_datas[id] / tower_armor_hs_datas_ptr[id]) * 600;
        tower_armor_hs_datas_ptr[id] = 600;
      }
    }

    last_id = id;
    update_count_++;
    xyz_in_world = armor.xyz_in_world;

    // 调用 EKF 更新观测值
    update_ypda(armor, id);
  }

  // 真正把观测喂进 EKF 的地方。观测量 z 是 4 维：(yaw方位角, pitch俯仰角, distance距离, 板朝向角)。
  //
  // 【为什么用球坐标而不是 xyz】相机的误差特性天生不均匀：测角度很准（像素位置精确），
  // 测距离很糙（靠装甲板在图上的大小反推，稍微识别偏一点距离就差不少）。
  // 用 (yaw,pitch,distance) 表示，就能在下面的 R 里分别给它们不同的信任度；
  // 如果用 xyz，这三个方向的误差会耦合在一起，没法这样区分对待。
  //
  // 【R 是什么】观测噪声矩阵，即"这次测量有多不可信"。和 Q 正好是一对：
  // R 小 = 信观测（跟得紧、抖）；R 大 = 信预测（平滑、滞后）。
  // 这里的 R 不是常数，而是按当前情况**启发式**算出来的（下面几行），这是本文件的一个特色：
  //   r2_azimuth / r2_pitch  角度观测，固定给 4e-3（很小 = 很信任，因为测角准）
  //   r2_angle               板朝向角的噪声，随距离增大而增大（越远，yaw 越不可信）
  //   r2_d                   距离噪声，随"板正面偏离视线的角度"增大而增大
  //                          —— 因为板越侧，PnP 解出的距离越不准，这符合物理直觉
  //   旋转态额外把 r2_angle 放大 3 倍：小陀螺时板朝向变化最快，最不该被信任
  void Target::update_ypda(const Armor & armor, int id)
  {
    auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
    auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);

    auto r2_azimuth = 4e-3;
    auto r2_pitch = 4e-3;
    auto r2_angle = log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2;
    auto r2_d = log(std::abs(delta_angle) + 1) + 1;

    if (motion_state_ == MotionState::IN_PLACE_ROTATION)
    {
      r2_angle *= 3.0;  // 旋转时进一步增加对角度的不信任
    }

    // // 前哨站换板瞬间，放宽距离和高度噪声信任度防跳变
    // if (name == ArmorName::outpost && is_switch_) {
    //     r2_pitch *= 100.0;
    //     r2_d     *= 100.0;
    // }

    if (last_cam_is_short != cam_is_short)
    {
      cam_is_switch_time_point = std::chrono::steady_clock::now();
      last_cam_is_short = cam_is_short;
    }
    auto now = std::chrono::steady_clock::now();
    // 换镜头（长焦/短焦切换）后的 0.7 秒内，把噪声抬到极大（4e+4 相当于"这段观测基本别信"）。
    // 原因：切焦距瞬间视野和成像比例突变，那几帧的解算结果不可靠，若照常喂进 EKF 会把
    // 已经收敛的状态一把带飞。抬高 R 让滤波器这段时间主要靠预测撑过去。
    // update_count_ > 50 这个条件是说：只有已经跟了一阵子（状态可信）才需要这样保护，
    // 刚起手时本来就没什么可保的。
    double cam_is_switch_lter_dt = tools::delta_time(now, cam_is_switch_time_point);
    if (cam_is_switch_lter_dt < 0.7 && update_count_ > 50)
    {
      r2_azimuth = 4e+4;
      r2_angle *= 300;
      r2_d *= 300;
    }

    // R 是对角矩阵：认为四个观测量的误差互不相关（工程上的常见简化）
    Eigen::VectorXd R_dig{{r2_azimuth, r2_pitch, r2_d, r2_angle}};
    Eigen::MatrixXd R = R_dig.asDiagonal();

    // 观测函数 h(x)：给定状态，预测"相机应该看到什么"。
    // 即：车心状态 --h_armor_xyz--> 第 id 块板的 xyz --xyz2ypd--> (yaw,pitch,distance)，
    // 再补上第四维板朝向角。EKF 拿它和实际观测 z 相减得到残差，据此修正状态。
    // 这是 EKF 里的 "E"（Extended）之所在：h 是非线性的，所以还需要下面的雅可比 H 做线性化。
    // 预测观测函数 h(x)
    auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
      Eigen::VectorXd xyz = h_armor_xyz(x, id);
      Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
      auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
      return {ypd[0], ypd[1], ypd[2], angle};
    };

    // 角度相减必须绕回 [-π, π]：179° 和 -179° 实际只差 2°，直接相减会得到 358°，
    // 滤波器会以为目标瞬间转了大半圈而炸掉。第 0/1/3 维是角度，第 2 维是距离所以不处理。
    // 自定义减法（处理角度越界）
    auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
      Eigen::VectorXd c = a - b;
      c[0] = tools::limit_rad(c[0]);
      c[1] = tools::limit_rad(c[1]);
      c[3] = tools::limit_rad(c[3]);
      return c;
    };

    const Eigen::VectorXd & ypd = armor.ypd_in_world;
    const Eigen::VectorXd & ypr = armor.ypr_in_world;
    Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

    ekf_.update(z, H, R, h, z_subtract);
  }

  // 获取 EKF 状态向量
  Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

  // 获取滤波器常引用
  const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

  // 返回所有装甲板的预测四维状态 (X, Y, Z, Angle) 列表
  std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
  {
    std::vector<Eigen::Vector4d> _armor_xyza_list;
    for (int i = 0; i < armor_num_; i++)
    {
      auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
      Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
      _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return _armor_xyza_list;
  }

  // 检查滤波器半径是否发散
  bool Target::diverged() const
  {
    auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
    auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;
    if (r_ok && l_ok) return false;
    return true;
  }

  // 判断当前目标是否收敛
  bool Target::convergened()
  {
    if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged())
    {
      is_converged_ = true;
    }
    if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged())
    {
      is_converged_ = true;
    }
    return is_converged_;
  }

  // 核心函数：根据 EKF 状态和 ID 推算该装甲板在世界坐标系下的理论位置 XYZ
  Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
  {
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto armor_x = x[0] - r * std::cos(angle);
    auto armor_y = x[2] - r * std::sin(angle);

    double armor_z;
    if (name == ArmorName::outpost)
    {
      double dz = tower_armor_hs[id].second - tower_armor_hs[0].second;
      int dz_px = dz > 0 ? 1 : -1;
      int dz_mu;

      // 使用定义的常量区分大跳变和小跳变阶梯
      if (std::abs(dz) > TOWER_ARMOR_DTB)
      {
        dz_mu = 2;  // 相隔两个阶梯 (大跳变)
      }
      else if (std::abs(dz) > TOWER_ARMOR_XTB)
      {
        dz_mu = 1;  // 相隔一个阶梯 (小跳变)
      }
      else
      {
        dz_mu = 0;  // 同一阶梯
      }

      // 结合滤波器的高度参数 x[10]
      armor_z = x[4] + x[10] * dz_px * dz_mu;
    }
    else
    {
      armor_z = (use_l_h) ? x[4] + x[10] : x[4];
    }
    return {armor_x, armor_y, armor_z};
  }

  // 计算当前预测观测函数的雅可比矩阵
  Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
  {
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);
    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);
    auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
    auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

    double dz_dh;
    if (this->name == ArmorName::outpost)
    {
      double dz = tower_armor_hs[id].second - tower_armor_hs[0].second;
      int dz_px = dz > 0 ? 1 : -1;
      int dz_mu;

      // 使用定义的常量区分跳变阶梯
      if (std::abs(dz) > TOWER_ARMOR_DTB)
      {
        dz_mu = 2;
      }
      else if (std::abs(dz) > TOWER_ARMOR_XTB)
      {
        dz_mu = 1;
      }
      else
      {
        dz_mu = 0;
      }
      dz_dh = dz_mu * dz_px;
    }
    else
    {
      dz_dh = (use_l_h) ? 1.0 : 0.0;
    }

    // 11 维位置偏导雅可比矩阵
    Eigen::MatrixXd H_armor_xyza = Eigen::MatrixXd::Zero(4, 11);
    H_armor_xyza(0, 0) = 1;
    H_armor_xyza(0, 6) = dx_da;
    H_armor_xyza(0, 8) = dx_dr;
    H_armor_xyza(0, 9) = dx_dl;
    H_armor_xyza(1, 2) = 1;
    H_armor_xyza(1, 6) = dy_da;
    H_armor_xyza(1, 8) = dy_dr;
    H_armor_xyza(1, 9) = dy_dl;
    H_armor_xyza(2, 4) = 1;
    H_armor_xyza(2, 10) = dz_dh;
    H_armor_xyza(3, 6) = 1;

    // 将 XYZ 偏导转换到 YPD 球坐标系下
    Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);

    Eigen::MatrixXd H_armor_ypda{
      {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
      {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
      {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
      {0, 0, 0, 1}};

    // 链式求导法 H_Final = H_ypda * H_xyza
    return H_armor_ypda * H_armor_xyza;
  }

  // 检查是否完成初始化
  bool Target::checkinit() { return isinit; }

}  // namespace auto_aim