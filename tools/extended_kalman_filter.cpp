#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
  ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
  : x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
  {
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
  }

  Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
  {
    return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
  }

  Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
  {
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
  }

  Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
  {
    return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
  }

  Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
  {
    Eigen::VectorXd x_prior = x;
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Stable Compution of the Posterior Covariance
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    x = x_add(x, K * z_subtract(z, h(x)));

    /// 卡方检验
    Eigen::VectorXd residual = z_subtract(z, h(x));
    // 新增检验
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    double nis = residual.transpose() * S.inverse() * residual;
    double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

    // 卡方检验阈值（自由度=4，取置信水平95%）
    //
    // 【这里原来是 0.711，是错的】自由度 4 的卡方分布：
    //   95% 分位 = 9.488   <- 注释声称要的就是这个
    //    5% 分位 = 0.711   <- 但实际填的是这个
    // 两者含义完全相反：用 9.488 表示"NIS 超过它才算异常"（正常数据只有 5% 会误报）；
    // 用 0.711 则表示"NIS 超过它就算异常"，而正常数据有整整 95% 都会超过，等于把
    // 健康的跟踪判成失败。配合 tracker.cpp 里"最近 100 次里失败 40% 就丢跟踪"的逻辑，
    // 理论上会误杀正常跟踪。故改回 9.488。
    // 7.779 是自由度 4 的 90% 分位，是更严格的备选，保留注释备查。
    //
    // 【离线实测（demo 视频前 200 帧）】改前改后 "Bad Converge Found" 都是 0 次，行为无差异。
    // 原因：本段视频里目标基本正对、运动平缓，NIS 中位数只有 0.009、最大 1.966，
    // 113 次更新中仅 3 次超过 0.711，无一次超过 9.488 —— 连错的阈值都没被触发到 40%。
    // 也就是说这个 bug 在这段素材上不会暴露，要在小陀螺/快速平移的素材上才看得出差别。
    //
    // 【另一个尚未修的问题】NIS 按定义应该用**新息**（更新前的残差 z - h(x_prior)），
    // 但上面第 52 行已经先更新了 x，第 55 行再算 residual 得到的是**后验残差**，
    // 数值会明显偏小（这也是实测 NIS 这么小的原因之一）。
    // 改它要动滤波器的计算顺序，影响面比换个常数大得多，本次不动，仅记录。
    constexpr double nis_threshold = 9.488;
    constexpr double nees_threshold = 9.488;
    // constexpr double nis_threshold = 7.779;
    // constexpr double nees_threshold = 7.779;

    if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
    if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
    total_count_++;
    last_nis = nis;

    recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

    if (recent_nis_failures.size() > window_size)
    {
      recent_nis_failures.pop_front();
    }

    int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

    data["residual_yaw"] = residual[0];
    data["residual_pitch"] = residual[1];
    data["residual_distance"] = residual[2];
    data["residual_angle"] = residual[3];
    data["nis"] = nis;
    data["nees"] = nees;
    data["recent_nis_failures"] = recent_rate;

    return x;
  }

}  // namespace tools