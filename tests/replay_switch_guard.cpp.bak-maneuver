// 离线回放: 把日志里的 RAWPROBE 原始观测按时间喂给真实的 auto_aim::Target,
// 对比 ekf_switch_guard 开/关时 vyaw 在换板后的冲高。只读日志, 不碰硬件。
// 用法: replay_switch_guard <log> <0|1 guard> [frames] [scale]
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

using namespace auto_aim;

struct Obs
{
  double t;
  double x, y, z, yaw;  // yaw: rad
};

int main(int argc, char ** argv)
{
  if (argc < 3) { std::printf("usage: %s log enable [gap_s] [max_dvyaw]\n", argv[0]); return 1; }
  Target::gap_vyaw_cfg.enable = std::atoi(argv[2]) != 0;
  if (argc > 3) Target::gap_vyaw_cfg.gap_s = std::atof(argv[3]);
  if (argc > 4) Target::gap_vyaw_cfg.max_dvyaw = std::atof(argv[4]);
  if (argc > 5) Target::gap_vyaw_cfg.vyaw_q = std::atof(argv[5]);
  if (argc > 6) Target::gap_vyaw_cfg.angle_r_scale = std::atof(argv[6]);
  if (argc > 7) Target::gap_vyaw_cfg.angle_r_max_vyaw = std::atof(argv[7]);
  if (argc > 8) Target::gap_vyaw_cfg.angle_r_warmup = std::atoi(argv[8]);
  if (argc > 9) Target::gap_vyaw_cfg.angle_r_gate = std::atof(argv[9]) / 57.3;
  if (argc > 10) Target::gap_vyaw_cfg.angle_r_bypass_frames = std::atoi(argv[10]);
  if (argc > 11) Target::gap_vyaw_cfg.angle_r_min_vyaw = std::atof(argv[11]);

  std::ifstream f(argv[1]);
  std::regex ts(R"(^\[\d+-\d+-\d+ (\d+):(\d+):([\d.]+)\])");
  std::regex raw(R"(RAWPROBE n=1 i=0 d=[\d.]+ xyz=\(([-\d.]+),([-\d.]+),([-\d.]+)\) yaw=([-\d.]+))");
  std::vector<Obs> obs;
  std::string line;
  std::smatch m;
  while (std::getline(f, line))
  {
    if (line.find("RAWPROBE n=1 i=0") == std::string::npos) continue;
    if (!std::regex_search(line, m, ts)) continue;
    double t = std::stod(m[1]) * 3600 + std::stod(m[2]) * 60 + std::stod(m[3]);
    if (!std::regex_search(line, m, raw)) continue;
    obs.push_back({t, std::stod(m[1]), std::stod(m[2]), std::stod(m[3]), std::stod(m[4]) / 57.2958});
  }
  if (obs.size() < 100) { std::printf("too few obs %zu\n", obs.size()); return 1; }

  auto mk = [](const Obs & o) {
    // 只为构造对象给四个占位角点, 下面会覆盖 EKF 用到的全部字段
    std::vector<cv::Point2f> kp{{0, 0}, {10, 0}, {10, 5}, {0, 5}};
    Armor a(0, 1.0f, cv::Rect(0, 0, 10, 5), kp);
    a.name = ArmorName::three;
    a.type = ArmorType::small;
    a.xyz_in_world = Eigen::Vector3d(o.x, o.y, o.z);
    a.ypr_in_world = Eigen::Vector3d(o.yaw, 0, 0);
    a.ypd_in_world = tools::xyz2ypd(a.xyz_in_world);
    return a;
  };

  using clk = std::chrono::steady_clock;
  const auto base = clk::now();
  auto tp = [&](double t) { return base + std::chrono::microseconds(static_cast<int64_t>((t - obs[0].t) * 1e6)); };

  Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  Target tgt(mk(obs[0]), tp(obs[0].t), 0.2, 4, P0_dig);

  int last_id = tgt.last_id, switches = 0, spin_frames = 0, high_frames = 0;
  std::vector<std::pair<double, double>> vyaw_trace;  // (t, |vyaw|)
  std::vector<double> switch_times;
  std::vector<double> peaks, settled;
  double win_peak = -1, win_end = -1, prev_t = obs[0].t;
  for (size_t i = 1; i < obs.size(); i++)
  {
    // 时间断档(>0.5s 没观测)就重建, 模拟 tracker 丢跟踪
    if (obs[i].t - prev_t > 0.5)
    {
      tgt = Target(mk(obs[i]), tp(obs[i].t), 0.2, 4, P0_dig);
      last_id = tgt.last_id; prev_t = obs[i].t; win_peak = -1; continue;
    }
    tgt.predict(tp(obs[i].t));
    const double v_before = tgt.ekf_x()[7];
    const int id_before = tgt.last_id;
    tgt.update(mk(obs[i]));
    prev_t = obs[i].t;
    double v = std::abs(tgt.ekf_x()[7]);
    // 换板后前 6 帧: 打印每帧 vyaw 的修正量和四维残差, 找出是哪一维把 vyaw 推高
    static int dump_left = 0, dumped = 0;
    static const bool verbose = std::getenv("REPLAY_VERBOSE") != nullptr;
    if (verbose && tgt.last_id != id_before && dumped < 4) { dump_left = 6; dumped++; std::printf("--- switch %d->%d\n", id_before, tgt.last_id); }
    if (dump_left > 0)
    {
      const auto & d = tgt.ekf().data;
      const auto & P = tgt.ekf().P;
      std::printf("  dt=%4.0fms dvyaw=%+.3f vyaw=%.2f sd(vyaw)=%.2f sd(yaw)=%.3f | res az=%+.4f dist=%+.3f ang=%+.4f r=%.3f\n",
        (obs[i].t - obs[i - 1].t) * 1e3, tgt.ekf_x()[7] - v_before, tgt.ekf_x()[7], std::sqrt(P(7, 7)), std::sqrt(P(6, 6)),
        d.at("residual_yaw"), d.at("residual_distance"), d.at("residual_angle"), tgt.ekf_x()[8]);
      dump_left--;
    }
    // 每次换板前一帧: 打印距上次观测间隔, 看是不是"转背面没看到 -> 不确定度涨起来"
    if (verbose && tgt.last_id != id_before && dumped <= 4)
    {
      const auto & P = tgt.ekf().P;
      std::printf("  [switch frame] gap since prev obs=%.0fms\n", (obs[i].t - obs[i - 1].t) * 1e3);
      (void)P;
    }
    if (tgt.last_id != last_id)
    {
      switches++;
      if (win_peak >= 0) peaks.push_back(win_peak);
      win_peak = v; win_end = obs[i].t + 0.4;
      last_id = tgt.last_id;
    }
    else if (win_peak >= 0 && obs[i].t <= win_end) win_peak = std::max(win_peak, v);
    else if (win_peak >= 0 && obs[i].t > win_end) { peaks.push_back(win_peak); win_peak = -1; }
    if (win_peak < 0 && v > 0.5) settled.push_back(v);
    // 全程(只算靶车在转的帧): vyaw 高于 1.57 的帧占比 —— 这才是 planner 走错分支的真实比例
    if (v > 0.5) { spin_frames++; if (v >= 1.57) high_frames++; }
    vyaw_trace.push_back({obs[i].t, v});
    if (tgt.last_id != id_before) switch_times.push_back(obs[i].t);
  }

  auto med = [](std::vector<double> v) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };

  // ★与转速无关的指标: EKF 转速 vs "真实转速"。
  //   真实转速 = 90 度 / 相邻两次换板的间隔(4 块板)。只用换板时间, 不用 PnP 朝向/距离。
  //   对每个换板区间, 求区间内 EKF |vyaw| 的中位数和最大值, 与真实值的比值。
  //   好的滤波: 中位比值接近 1, 最大比值不远离 1。
  std::vector<double> ratio_med, ratio_max;
  size_t k = 0;
  for (size_t s = 1; s < switch_times.size(); s++)
  {
    const double a = switch_times[s - 1], b = switch_times[s];
    const double period = b - a;
    if (period < 0.05 || period > 3.0) continue;  // 过短是换板抖动, 过长是丢目标
    const double w_true = (CV_PI / 2) / period;
    std::vector<double> in;
    while (k < vyaw_trace.size() && vyaw_trace[k].first < a) k++;
    for (size_t j = k; j < vyaw_trace.size() && vyaw_trace[j].first < b; j++) in.push_back(vyaw_trace[j].second);
    if (in.size() < 5) continue;
    double mxv = 0; for (double x : in) mxv = std::max(mxv, x);
    ratio_med.push_back(med(in) / w_true);
    ratio_max.push_back(mxv / w_true);
  }
  auto pct = [](std::vector<double> v, double q) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[std::min(v.size() - 1, (size_t)(q * v.size()))]; };
  int over = 0;
  for (double p : peaks) over += p >= 1.57;
  double mx = 0;
  for (double p : peaks) mx = std::max(mx, p);
  std::printf("angR=x%-4.0f(<%.1f,wu%d) v2=%4.0f ", Target::gap_vyaw_cfg.angle_r_scale, std::min(99.0, Target::gap_vyaw_cfg.angle_r_max_vyaw),
    Target::gap_vyaw_cfg.angle_r_warmup,
    Target::gap_vyaw_cfg.vyaw_q > 0 ? Target::gap_vyaw_cfg.vyaw_q : 400.0);
  std::printf("limit=%d gap=%.0fms max_dv=%.2f | switches=%d | peak median=%.2f max=%.2f >=1.57: %d/%zu | steady median=%.2f | spin frames >=1.57: %.1f%%\n",
    Target::gap_vyaw_cfg.enable, Target::gap_vyaw_cfg.gap_s * 1e3, Target::gap_vyaw_cfg.max_dvyaw,
    switches, med(peaks), mx, over, peaks.size(), med(settled), spin_frames ? 100.0 * high_frames / spin_frames : 0.0);
  std::printf("    [vs true spin, %zu intervals] EKF/true: median-in-interval p50=%.2f p10=%.2f p90=%.2f | max-in-interval p50=%.2f p90=%.2f\n",
    ratio_med.size(), pct(ratio_med, 0.5), pct(ratio_med, 0.1), pct(ratio_med, 0.9), pct(ratio_max, 0.5), pct(ratio_max, 0.9));

  // ★走错分支统计: 用"真实转速"(换板间隔 + 4 块板)判断每帧本该走哪个分支,
  //   再用 EKF 转速 + 快慢分界(可带迟滞)模拟 planner 实际走哪个分支。
  //   慢错高 = 真实 <1.57 却走了高速打法(慢速打早的来源); 快错低 = 真实 >2.5 却走了低速追板。
  struct Hy { double enter, exit; };
  for (Hy h : {Hy{1.57, 1.57}, Hy{2.0, 1.3}, Hy{2.2, 1.2}, Hy{2.5, 1.4}})
  {
    std::vector<char> st(vyaw_trace.size(), 0);
    bool high = false;
    for (size_t j = 0; j < vyaw_trace.size(); j++)
    {
      const double v = vyaw_trace[j].second;
      if (!high && v >= h.enter) high = true;
      else if (high && v < h.exit) high = false;
      st[j] = high;
    }
    int slow_n = 0, slow_high = 0, fast_n = 0, fast_low = 0;
    size_t kk = 0;
    for (size_t s = 1; s < switch_times.size(); s++)
    {
      const double a = switch_times[s - 1], b = switch_times[s];
      const double period = b - a;
      if (period < 0.05 || period > 3.0) continue;
      const double w_true = (CV_PI / 2) / period;
      while (kk < vyaw_trace.size() && vyaw_trace[kk].first < a) kk++;
      for (size_t j = kk; j < vyaw_trace.size() && vyaw_trace[j].first < b; j++)
      {
        if (w_true < 1.57) { slow_n++; slow_high += st[j]; }
        else if (w_true > 2.5) { fast_n++; fast_low += !st[j]; }
      }
    }
    std::printf("    [branch enter>=%.2f exit<%.2f] slow->HIGH(wrong) %d/%d (%.1f%%) | fast->LOW(wrong) %d/%d (%.1f%%)\n",
      h.enter, h.exit, slow_high, slow_n, slow_n ? 100.0 * slow_high / slow_n : 0.0, fast_low, fast_n, fast_n ? 100.0 * fast_low / fast_n : 0.0);
  }
  return 0;
}
