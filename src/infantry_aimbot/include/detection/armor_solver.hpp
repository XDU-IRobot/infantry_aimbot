#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "params.hpp"
#include "typedefs.hpp"

namespace ia {
namespace detection {

/// @brief 坐标变换链：PnP → 相机系 → 云台系 → 世界系 → yaw优化
/// @warning 线程不安全，单线程使用
class ArmorSolver {
 public:
  static constexpr double kBigArmorWidth = 0.225;
  static constexpr double kBigArmorHeight = 0.05;
  static constexpr double kSmallArmorWidth = 0.133;
  static constexpr double kSmallArmorHeight = 0.05;

  explicit ArmorSolver(const RosParams& config);

  /// @brief 根据IMU四元数更新云台到世界的旋转矩阵
  void SetRGimbal2World(const Eigen::Quaterniond& q);

  /// @brief 获取当前云台到世界的旋转矩阵
  Eigen::Matrix3d RGimbal2World() const;

  /// @brief 执行完整坐标链：solvePnP → 相机→云台→世界 → yaw优化
  /// @param armor 装甲板（输入image points，输出世界坐标成员）
  void Solve(Armor& armor) const;

  /// @brief 根据世界坐标和yaw重投影装甲板顶点
  std::vector<cv::Point2f> ReprojectArmor(const Eigen::Vector3d& xyz_in_world, double yaw, ArmorKind kind,
                                          ArmorName name) const;

 private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  static const std::vector<cv::Point3f>& GetObjectPoints(ArmorKind kind);
  void OptimizeYaw(Armor& armor) const;
  double ArmorReprojectionError(const Armor& armor, double yaw) const;
};

}  // namespace detection
}  // namespace ia
