// 离线回放: 日志里的原始观测 -> 真实 Target + Planner, 每个规划周期输出一行指令, 写到 stdout。
// 只用 A 版就有的接口, 所以 A 版和之后的分支都能编同一份, 两份输出直接逐行比较。
// 用法: replay_dump <log> <config.yaml> > out.txt
// 每行(17 列):
//   1 t  2 yaw(deg)  3 yaw_vel(deg/s)  4 pitch(deg)  5 fire  6 vyaw(rad/s)  7 v(m/s)  8 center_azim(deg)
//   9 yaw_acc(deg/s^2)  10 pitch_vel(deg/s)  11 target_yaw(deg)  12 fire_gate(位: 1窗口 2云台远 4换板等待 8急刹 16用完)
//   13 fire_win(deg)  14 fire_delta(deg)  15 dgim(指令-云台, deg)  16 走平移开火判断  17 观测拟合生效
// 以 '[' 开头的行是程序日志, 对比时过滤掉(tests/golden/run_golden.sh)。
// 云台模型: 云台角 = 30ms 前自己发出的 yaw 指令(实车日志: 云台稳定落后指令约 30ms)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <optional>
#include <regex>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/math_tools.hpp"

using namespace auto_aim;

struct Obs
{
  double t, x, y, z, yaw;
};

int main(int argc, char ** argv)
{
  if (argc < 3) { std::fprintf(stderr, "usage: %s log config\n", argv[0]); return 1; }
  std::ifstream f(argv[1]);
  std::regex ts(R"(^\[\d+-\d+-\d+ (\d+):(\d+):([\d.]+)\])");
  std::regex raw(R"(RAWPROBE n=\d+ i=0 d=[\d.]+ xyz=\(([-\d.]+),([-\d.]+),([-\d.]+)\) yaw=([-\d.]+))");
  std::vector<Obs> obs;
  std::string line;
  std::smatch m;
  while (std::getline(f, line))
  {
    if (line.find("RAWPROBE n=") == std::string::npos || line.find(" i=0 ") == std::string::npos) continue;
    if (!std::regex_search(line, m, ts)) continue;
    const double t = std::stod(m[1]) * 3600 + std::stod(m[2]) * 60 + std::stod(m[3]);
    if (!std::regex_search(line, m, raw)) continue;
    if (std::abs(std::stod(m[1])) < 0.3) continue;  // 无效解算
    obs.push_back({t, std::stod(m[1]), std::stod(m[2]), std::stod(m[3]), std::stod(m[4]) / 57.2958});
  }
  if (obs.size() < 100) { std::fprintf(stderr, "too few obs %zu\n", obs.size()); return 1; }

  Tracker cfg_loader(argv[2], nullptr);  // 只为读 yaml 里的 EKF 参数, 和车上一样
  Planner P(argv[2]);

  auto mk = [](const Obs & o) {
    std::vector<cv::Point2f> kp{{0, 0}, {10, 0}, {10, 5}, {0, 5}};
    Armor a(0, 1.0f, cv::Rect(0, 0, 10, 5), kp);
    a.name = ArmorName::three;
    a.type = ArmorType::small;
    a.xyz_in_world = Eigen::Vector3d(o.x, o.y, o.z);
    a.ypr_in_world = Eigen::Vector3d(o.yaw, 0, 0);
    a.ypd_in_world = tools::xyz2ypd(a.xyz_in_world);
    return a;
  };
  const auto base = std::chrono::steady_clock::now();
  auto tp = [&](double t) { return base + std::chrono::microseconds(static_cast<int64_t>((t - obs[0].t) * 1e6)); };
  Eigen::VectorXd P0{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};

  std::optional<Target> T = Target(mk(obs[0]), tp(obs[0].t), 0.2, 4, P0);
  std::vector<std::pair<double, double>> hist;  // (t, 发出的 yaw rad)
  auto gimbal = [&](double t) {
    for (size_t i = hist.size(); i-- > 0;)
      if (hist[i].first <= t - 0.03) return hist[i].second;
    return hist.empty() ? 0.0 : hist.front().second;
  };

  const double DT = 0.0056;  // 车上 plan_thread 周期
  double tpl = obs[0].t;
  for (size_t i = 1; i < obs.size(); i++)
  {
    while (tpl + DT < obs[i].t)
    {
      tpl += DT;
      if (tpl - obs[i - 1].t > 0.5) continue;
      P.clock_override = tp(tpl);
      Plan p = P.plan(T, 24.4, gimbal(tpl) * 57.3, Planner::rbSuppressiveFire);
      if (!p.control) continue;
      hist.push_back({tpl, p.yaw});
      const auto x = T->ekf_x();
      std::printf("%.4f %.4f %.3f %.4f %d %.3f %.3f %.3f", tpl - obs[0].t, p.yaw * 57.3, p.yaw_vel * 57.3,
                  p.pitch * 57.3, p.fire ? 1 : 0, x[7], std::hypot(x[1], x[3]), std::atan2(x[2], x[0]) * 57.3);
      // 诊断列(只输出, 不改逻辑): 9~17
      std::printf(" %.4f %.4f %.4f %d %.4f %.4f %.4f %d %d\n", p.yaw_acc * 57.3, p.pitch_vel * 57.3, p.target_yaw,
                  P.debug_fire_gate_, P.debug_fire_win_deg_, P.debug_fire_delta_deg_, P.debug_fire_dgim_deg_,
                  P.debug_fire_trans_ ? 1 : 0, P.debug_obs_fit_used_ ? 1 : 0);
    }
    if (obs[i].t - obs[i - 1].t > 0.5 || T->diverged())
    {
      T = Target(mk(obs[i]), tp(obs[i].t), 0.2, 4, P0);  // 丢跟踪/发散 -> 重建
      continue;
    }
    T->predict(tp(obs[i].t));
    T->update(mk(obs[i]));
  }
  return 0;
}
