#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
  struct TrajectoryV1
  {
    bool unsolvable;
    double fly_time;
    double pitch;  // 抬头为正

    // 不考虑空气阻力
    // v0 子弹初速度大小，单位：m/s
    // d 目标水平距离，单位：m
    // h 目标竖直高度，单位：m

    TrajectoryV1() = default;
    TrajectoryV1(const double v0, const double d, const double h);
  };

  struct TrajectoryV2 : TrajectoryV1
  {
    TrajectoryV2(const double v0, const double d, const double h);
  };

  // ★全仓库都通过 tools::Trajectory 这个别名使用弹道，所以**实际生效的是 V2（含空气阻力）**。
  // V1 只考虑重力，留作参照，没有任何地方直接构造它。
  // 想对比两个模型的差异，改这一行即可全局切换（记得改回来）。
  using Trajectory = TrajectoryV2;

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP