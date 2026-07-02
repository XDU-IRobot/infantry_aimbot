#pragma once

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace ia {
namespace tools {

class ExtendedKalmanFilter {
 public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
      const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add =
          [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a + b; });

  Eigen::VectorXd Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

  Eigen::VectorXd Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
                          std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f);

  Eigen::VectorXd Update(
      const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
          [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; });

  Eigen::VectorXd Update(
      const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
          [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; });

  // 卡方检验数据
  std::map<std::string, double> data;
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis;

 private:
  Eigen::MatrixXd I_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add_;

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;
};

}  // namespace tools
}  // namespace ia
