#include "estimation/ekf_car_tracker.hpp"

#include <opencv2/opencv.hpp>

namespace ia {
namespace estimation {
EKFCarTracker::EKFCarTracker() : CarTracker() {}

EKFCarTracker::~EKFCarTracker() {}
result_void EKFCarTracker::Initialize() { return outcome_v2::success(); }

result_void EKFCarTracker::Reset() { return outcome_v2::success(); }

result_sp<Eigen::MatrixXd> EKFCarTracker::estimate() const {
  auto state = std::make_shared<Eigen::MatrixXd>(4, 1);
  state->setZero();
  return outcome_v2::success(state);
}

result_sp<Eigen::MatrixXd> EKFCarTracker::predict(double dt) const {
  auto state = std::make_shared<Eigen::MatrixXd>(4, 1);
  state->setZero();
  return outcome_v2::success(state);
}

result_void EKFCarTracker::Update(std::shared_ptr<Eigen::MatrixXd>) { return outcome_v2::success(); }

}  // namespace estimation
}  // namespace ia