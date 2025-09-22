#pragma once

#include "car_tracker.hpp"

namespace ia {
namespace estimation {
class EKFCarTracker : public CarTracker {
 public:
  EKFCarTracker();
  ~EKFCarTracker();

  result_void Initialize() override;
  result_void Reset() override;
  result_sp<Eigen::MatrixXd> estimate() const override;
  result_sp<Eigen::MatrixXd> predict(double dt) const override;
  result_void Update(std::shared_ptr<Eigen::MatrixXd>) override;

 private:
  std::unique_ptr<cv::KalmanFilter> g_kf_;
};
}  // namespace estimation
}  // namespace ia
