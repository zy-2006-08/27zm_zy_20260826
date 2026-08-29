#include "trajectory.hpp"

#include <cmath>
#include <iostream>

namespace tools
{
  // ============================================================================
  // 弹道解算：已知目标在哪，反算枪口该抬多高、子弹要飞多久。
  //
  // 输入统一是：v0 弹速【m/s】、d 水平距离【m】、h 高度差【m】（目标比枪口高为正）。
  // 输出：pitch 发射角【弧度】（抬头为正）、fly_time 飞行时间【秒】。
  //
  // fly_time 为什么重要：子弹飞过去要时间，这期间敌方车还在动，所以必须打提前量——
  // 用 EKF 预测 fly_time 之后目标在哪，再朝那里开火。飞行时间算错，提前量就错。
  // 调用处见 tasks/auto_aim/aimer.cpp，那里会迭代几轮（位置->飞行时间->新位置->...）。
  // ============================================================================

  // 重力加速度。不是课本的 9.8，是按本地纬度/海拔修正过的值。
  constexpr double g = 9.7833;

  // 【V1：只考虑重力，忽略空气阻力】
  // 思路：把斜抛的水平、竖直两个方程联立、消掉时间，得到关于 tan(pitch) 的一元二次方程，
  // 直接套求根公式。delta < 0 表示无实数解，物理含义是"以这个弹速根本打不到那么远"，
  // 此时置 unsolvable=true，调用方应放弃这一帧，别拿垃圾角度去转云台。
  // 两个根对应低抛和高抛（吊射），下面取飞行时间短的那个即低抛：
  // 飞得越久，预测误差和风的影响越大，所以能平射就不吊射。
  //
  // ★V1 当前并不生效，实际用的是下面的 V2（见 trajectory.hpp 的 using Trajectory）。
  //同济原弹道模型
  TrajectoryV1::TrajectoryV1(const double v0, const double d, const double h)
  {
    auto a = g * d * d / (2 * v0 * v0);
    auto b = -d;
    auto c = a + h;
    auto delta = b * b - 4 * a * c;

    if (delta < 0)
    {
      unsolvable = true;
      return;
    }
    // std::cout<<"delta :"<<delta<<std::endl;

    unsolvable = false;
    auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
    auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
    auto pitch_1 = std::atan(tan_pitch_1);
    auto pitch_2 = std::atan(tan_pitch_2);
    auto t_1 = d / (v0 * std::cos(pitch_1));
    auto t_2 = d / (v0 * std::cos(pitch_2));

    pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
    fly_time = (t_1 < t_2) ? t_1 : t_2;
  }

  // 【V2：重力 + 空气阻力】★这才是实际生效的模型。
  //
  // 空气阻力按 F = k*v^2 建模（高速下阻力与速度平方成正比）。真正进公式的是 k/m 这个组合：
  // 同样的阻力，弹丸越重减速越慢。
  //   大弹丸（42mm，英雄）k=0.000429838, m=0.043 kg
  //   小弹丸（17mm，步兵）k=0.000067165, m=0.0032 kg
  // 这四个数是实测拟合的，属于"不在 yaml、改了要重新编译"那一类。
  //
  // 求解策略（与 V1 不同，分两步）：
  //   第一步 只看水平方向。有阻力时水平速度衰减，由此列出关于飞行时间的二次方程，先定 fly_time。
  //   第二步 飞行时间已知，代入竖直方程反解发射角。
  // 这样避免把阻力和重力耦合成一个超越方程；代价是竖直方向仍只算了重力（近似）。
  //V2空气阻力弹道模型
  constexpr double kBig = 0.000429838;
  constexpr double mBig = 0.043;
  constexpr double kSmall = 0.000067165;
  constexpr double mSmall = 0.0032;

  TrajectoryV2::TrajectoryV2(const double v0, const double d, const double h)
  {
    double k, m;
    // ★按弹速反推弹丸大小：v0 > 18 m/s 判为**小**弹丸（注意这里落在 false 分支，别读反）。
    // 依据是规则限速不同——17mm 小弹丸允许 30 m/s 上下，42mm 大弹丸只有 16 m/s 左右，
    // 所以弹速本身就能区分兵种，不需要额外配置项。
    // 隐患：若弹速回传异常（例如 C 板还没测到、回传 0），会误判成大弹丸而用错 k/m。
    bool isBigBullet = v0 > 18 ? false : true;
    fly_time = 0.;
    if (isBigBullet)
    {
      k = kBig;
      m = mBig;
    }
    else
    {
      k = kSmall;
      m = mSmall;
    }

    double kv2m = k * v0 * v0 / m;
    double discriminant = v0 * v0 - 2 * kv2m * d;

    // 判别式 < 0：水平方向的时间方程无实数解，物理含义是"子弹在到达该距离前已被阻力减速殆尽"，
    // 也就是打不到。unsolvable 是给调用方的信号：这一帧的解不可用，别拿去转云台或开火。
    if (discriminant < 0)
    {
      unsolvable = true;  // 无正数解
      return;
    }

    double t1 = (v0 + std::sqrt(discriminant)) / kv2m;
    double t2 = (v0 - std::sqrt(discriminant)) / kv2m;

    // 选择正的时间解
    if (t1 > 0 && t2 > 0)
    {
      fly_time = (t1 < t2) ? t1 : t2;
      unsolvable = false;
    }
    else if (t1 > 0)
    {
      fly_time = t1;
      unsolvable = false;
    }
    else if (t2 > 0)
    {
      fly_time = t2;
      unsolvable = false;
    }
    else
    {
      unsolvable = true;  // 无正数解
      return;
    }

    // 第二步：飞行时间已定，解竖直方向。
    // 竖直位移 v0*sin(pitch)*t - 0.5*g*t^2 要等于高度差 h，对 sin(pitch) 解出来就是下面这行：
    // 抬枪既要够到 h，还要额外补上这段时间里重力造成的下坠 0.5*g*t^2。
    // 这就是"枪口得往上抬一点"的来源。
    // 计算目标角度（考虑重力影响的sinθ值）
    double target_sin_angle = (h + 0.5 * g * fly_time * fly_time) / (v0 * fly_time);

    // sin 只能落在 [-1,1]。超出说明"再怎么抬枪也够不着"（太远或高度差太大），同样判为无解。
    // 检查sin值是否在有效范围内 [-1, 1]
    if (std::abs(target_sin_angle) > 1.0)
    {
      unsolvable = true;
      return;
    }

    // 3. 计算发射角度
    pitch = std::asin(target_sin_angle);

    // // 4. 计算重力补偿角度
    // // 重力补偿角度 = 实际发射角度 - 无重力时的发射角度
    // double no_gravity_angle = std::asin(h / d); // 注意：需要检查d不为0
    // if (d == 0) {
    //     no_gravity_angle = 0;
    // }

    // weight_compensation_angle = pitch - no_gravity_angle;

    unsolvable = false;
  }

}  // namespace tools