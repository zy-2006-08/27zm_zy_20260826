#include "trajectory.hpp"

#include <cmath>
#include <iostream>

namespace tools
{
constexpr double g = 9.7833;

//同济原弹道模型
TrajectoryV1::TrajectoryV1(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
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

//V2空气阻力弹道模型
constexpr double kBig = 0.000429838;
constexpr double mBig = 0.043;
constexpr double kSmall = 0.000067165;
constexpr double mSmall = 0.0032;

TrajectoryV2::TrajectoryV2(const double v0, const double d, const double h){
  double k, m;
  bool isBigBullet = v0 > 18 ? false : true;
   fly_time = 0.;
  if (isBigBullet) {
      k = kBig;
      m = mBig;
  } else {
      k = kSmall;
      m = mSmall;
  }
  
  double kv2m = k * v0 * v0 / m;
  double discriminant = v0 * v0 - 2 * kv2m * d;
  
  if (discriminant < 0) {
    unsolvable = true; // 无正数解
    return;
  }
  
  double t1 = (v0 + std::sqrt(discriminant)) / kv2m;
  double t2 = (v0 - std::sqrt(discriminant)) / kv2m;
  
  // 选择正的时间解
  if (t1 > 0 && t2 > 0) {
      fly_time =  (t1 < t2) ? t1 : t2;
      unsolvable = false; 
  } else if (t1 > 0) {
      fly_time =  t1;
      unsolvable = false; 
  } else if (t2 > 0) {
      fly_time =  t2;
      unsolvable = false; 
  }else{
    unsolvable = true; // 无正数解
    return ;
  }

  // 计算目标角度（考虑重力影响的sinθ值）
  double target_sin_angle = (h + 0.5 * g * fly_time * fly_time) / (v0 * fly_time);
  
  // 检查sin值是否在有效范围内 [-1, 1]
  if (std::abs(target_sin_angle) > 1.0) {
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