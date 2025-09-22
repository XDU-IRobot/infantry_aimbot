#pragma once

#include <memory>

#include <Eigen/Dense>

#include "typedefs.hpp"

namespace ia {
namespace estimation {

class CarTracker {
 public:
  CarTracker();
  virtual ~CarTracker() = 0;

  // 追踪器初始化，配置参数
  virtual result_void Initialize();

  // 重置追踪器
  virtual result_void Reset();

  virtual result_sp<Eigen::MatrixXd> estimate() const = 0;

  virtual result_sp<Eigen::MatrixXd> predict(double dt) const = 0;

  virtual result_void Update(std::shared_ptr<Eigen::MatrixXd>) = 0;

 private:
};
}  // namespace estimation
}  // namespace ia