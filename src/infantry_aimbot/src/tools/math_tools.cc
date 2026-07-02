#include "tools/math_tools.hpp"

#include <cmath>

namespace ia {
namespace tools {

double LimitRad(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle <= -M_PI) angle += 2 * M_PI;
  return angle;
}

Eigen::Vector3d Eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic) {
  if (extrinsic) q = q.inverse();
  Eigen::Matrix3d R = q.toRotationMatrix();
  return Eulers(R, axis0, axis1, axis2, extrinsic);
}

Eigen::Vector3d Eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool /*extrinsic*/) {
  constexpr double kEpsilon = 1e-6;
  Eigen::Vector3d euler;
  int i = axis0;
  int j = axis1;
  int k = axis2;

  // 参考 evbernardes/quaternion_to_euler
  if (i == k) {
    k = 3 - i - j;  // 三个轴的和应为 0+1+2=3
    double sy = std::sqrt(R(i, i) * R(i, i) + R(i, j) * R(i, j));
    bool singular = sy < kEpsilon;
    if (!singular) {
      euler[0] = std::atan2(R(j, k), R(k, k));
      euler[1] = std::atan2(-R(i, k), sy);
      euler[2] = std::atan2(R(i, j), R(i, i));
    } else {
      euler[0] = std::atan2(-R(k, j), R(j, j));
      euler[1] = std::atan2(-R(i, k), sy);
      euler[2] = 0;
    }
  } else {
    euler[0] = std::atan2(R(j, k), R(k, k));
    double c2 = std::sqrt(R(i, i) * R(i, i) + R(i, j) * R(i, j));
    euler[1] = std::atan2(-R(i, k), c2);
    double s1 = std::sin(euler[0]);
    double c1 = std::cos(euler[0]);
    euler[2] = std::atan2(s1 * R(k, i) - c1 * R(j, i), c1 * R(j, j) - s1 * R(k, j));
  }
  // 结果顺序: axis0, axis1, axis2
  return euler;
}

Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& ypr) {
  double cy = std::cos(ypr[0]);
  double sy = std::sin(ypr[0]);
  double cp = std::cos(ypr[1]);
  double sp = std::sin(ypr[1]);
  double cr = std::cos(ypr[2]);
  double sr = std::sin(ypr[2]);

  Eigen::Matrix3d Rz, Ry, Rx;
  // clang-format off
  Rz << cy, -sy, 0,
        sy,  cy, 0,
         0,   0, 1;
  Ry << cp,  0, sp,
         0,  1,  0,
       -sp,  0, cp;
  Rx << 1,  0,   0,
        0, cr, -sr,
        0, sr,  cr;
  // clang-format on

  return Rz * Ry * Rx;
}

Eigen::Vector3d Xyz2Ypd(const Eigen::Vector3d& xyz) {
  double distance = xyz.norm();
  double yaw = std::atan2(xyz.y(), xyz.x());
  double pitch = std::asin(xyz.z() / distance);
  return {yaw, pitch, distance};
}

Eigen::MatrixXd Xyz2YpdJacobian(const Eigen::Vector3d& xyz) {
  double x = xyz.x();
  double y = xyz.y();
  double z = xyz.z();
  double d2 = x * x + y * y + z * z;
  double d = std::sqrt(d2);
  double r2 = x * x + y * y;
  double r = std::sqrt(r2);

  // clang-format off
  Eigen::MatrixXd J(3, 3);
  J << -y / r2,  x / r2,      0,
       -x * z / (d2 * r), -y * z / (d2 * r), r / d2,
        x / d,   y / d,  z / d;
  // clang-format on
  return J;
}

Eigen::Vector3d Ypd2Xyz(const Eigen::Vector3d& ypd) {
  double x = ypd[2] * std::cos(ypd[1]) * std::cos(ypd[0]);
  double y = ypd[2] * std::cos(ypd[1]) * std::sin(ypd[0]);
  double z = ypd[2] * std::sin(ypd[1]);
  return {x, y, z};
}

Eigen::MatrixXd Ypd2XyzJacobian(const Eigen::Vector3d& ypd) {
  double yaw = ypd[0];
  double pitch = ypd[1];
  double d = ypd[2];
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  double cos_pitch = std::cos(pitch);
  double sin_pitch = std::sin(pitch);

  // clang-format off
  Eigen::MatrixXd J(3, 3);
  J << -d * cos_pitch * sin_yaw, -d * sin_pitch * cos_yaw, cos_pitch * cos_yaw,
        d * cos_pitch * cos_yaw, -d * sin_pitch * sin_yaw, cos_pitch * sin_yaw,
                             0,          d * cos_pitch,          sin_pitch;
  // clang-format on
  return J;
}

double DeltaTime(const std::chrono::steady_clock::time_point& a, const std::chrono::steady_clock::time_point& b) {
  return std::chrono::duration<double>(a - b).count();
}

double GetAbsAngle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
  double dot = vec1.dot(vec2);
  double cross = vec1.x() * vec2.y() - vec1.y() * vec2.x();
  return std::atan2(std::abs(cross), dot);
}

double LimitMinMax(double input, double min, double max) {
  if (input < min) return min;
  if (input > max) return max;
  return input;
}

}  // namespace tools
}  // namespace ia
