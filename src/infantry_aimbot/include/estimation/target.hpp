#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "tools/ekf.hpp"
#include "typedefs.hpp"

namespace ia {
namespace estimation {

/// @brief EKF整车状态估计：11维状态向量追踪敌方车辆
///
/// 状态向量: [x, vx, y, vy, z, vz, yaw, omega, r, l, h]
///   x,y,z: 旋转中心世界坐标
///   vx,vy,vz: 旋转中心速度
///   yaw: 旋转角度
///   omega: 角速度
///   r: 装甲板半径（中心到装甲板的距离）
///   l: r2 - r1（对向装甲板半径差）
///   h: z2 - z1（对向装甲板高度差）
class Target {
 public:
  ArmorName name{ArmorName::kNotArmor};
  ArmorKind armor_kind{ArmorKind::kSmall};
  ArmorPriority priority{ArmorPriority::kFifth};
  bool jumped{false};
  int last_id{0};

  Target() = default;

  /// 从检测初始化
  Target(const Armor& armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
         const Eigen::VectorXd& P0_diag);

  void Predict(std::chrono::steady_clock::time_point t);
  void Predict(double dt);
  void Update(const Armor& armor);

  Eigen::VectorXd EkfX() const;
  const tools::ExtendedKalmanFilter& Ekf() const;
  std::vector<Eigen::Vector4d> ArmorXyzaList() const;
  bool Diverged() const;
  bool Convergened();

 private:
  int armor_num_{4};
  int switch_count_{0};
  int update_count_{0};
  bool is_converged_{false};
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void UpdateYpda(const Armor& armor, int id);
  Eigen::Vector3d HArmorXyz(const Eigen::VectorXd& x, int id) const;
  Eigen::MatrixXd HJacobian(const Eigen::VectorXd& x, int id) const;
};

}  // namespace estimation
}  // namespace ia
