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
  result_sp<Eigen::VectorXd> estimate() const override;
  result_sp<Eigen::VectorXd> predict(double dt) const override;
  result_void Update(std::shared_ptr<Eigen::VectorXd>) override;

 private:
};
}  // namespace estimation
}  // namespace ia
