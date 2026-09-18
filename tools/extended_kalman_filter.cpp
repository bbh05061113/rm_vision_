#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
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

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
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
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
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
  Eigen::MatrixXd S = H * P * H.transpose() + R;//残差协方差矩阵
  double nis = residual.transpose() * S.inverse() * residual;//衡量观测和状态预测之间的匹配程度 NIS 过大，当前观测是坏点，丢弃观测，不做 EKF 更新
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);//滤波器本身估计误差和协方差矩阵 P 是否自洽

  // 卡方检验阈值（自由度=4，取置信水平95%）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();//删除队列最前面的元素，不返回值
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);//遍历容器，把容器里面所有元素累加求和
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();//窗口内观测野点失败率

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