#include "detection/armor_solver.hpp"

#include <opencv2/core/eigen.hpp>

#include "tools/math_tools.hpp"

namespace ia {
namespace detection {

// 3D模型点：{左上, 右上, 右下, 左下}
static const std::vector<cv::Point3f> kBigArmorPoints{
    cv::Point3f{-ArmorSolver::kBigArmorWidth / 2, -ArmorSolver::kBigArmorHeight / 2, 0.f},
    cv::Point3f{ArmorSolver::kBigArmorWidth / 2, -ArmorSolver::kBigArmorHeight / 2, 0.f},
    cv::Point3f{ArmorSolver::kBigArmorWidth / 2, ArmorSolver::kBigArmorHeight / 2, 0.f},
    cv::Point3f{-ArmorSolver::kBigArmorWidth / 2, ArmorSolver::kBigArmorHeight / 2, 0.f}};

static const std::vector<cv::Point3f> kSmallArmorPoints{
    cv::Point3f{-ArmorSolver::kSmallArmorWidth / 2, -ArmorSolver::kSmallArmorHeight / 2, 0.f},
    cv::Point3f{ArmorSolver::kSmallArmorWidth / 2, -ArmorSolver::kSmallArmorHeight / 2, 0.f},
    cv::Point3f{ArmorSolver::kSmallArmorWidth / 2, ArmorSolver::kSmallArmorHeight / 2, 0.f},
    cv::Point3f{-ArmorSolver::kSmallArmorWidth / 2, ArmorSolver::kSmallArmorHeight / 2, 0.f}};

const std::vector<cv::Point3f>& ArmorSolver::GetObjectPoints(ArmorKind kind) {
  return (kind == ArmorKind::kBig) ? kBigArmorPoints : kSmallArmorPoints;
}

ArmorSolver::ArmorSolver(const RosParams& config) : R_gimbal2world_{Eigen::Matrix3d::Identity()} {
  // 相机内参
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cam_mat(config.camera_info.camera_matrix.data());
  cv::eigen2cv(cam_mat, camera_matrix_);

  Eigen::Matrix<double, 1, 5> dist_vec(config.camera_info.distortion_coefficients.data());
  cv::eigen2cv(dist_vec, distort_coeffs_);

  // 外参
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(config.solver.R_gimbal2imubody.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(config.solver.R_camera2gimbal.data());
  t_camera2gimbal_ = Eigen::Vector3d(config.solver.t_camera2gimbal.data());
}

void ArmorSolver::SetRGimbal2World(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

Eigen::Matrix3d ArmorSolver::RGimbal2World() const { return R_gimbal2world_; }

void ArmorSolver::Solve(Armor& armor) const {
  const auto& object_points = GetObjectPoints(armor.kind);

  // 图像点：{左上, 右上, 右下, 左下}
  std::vector<cv::Point2f> image_points{armor.left_light.up, armor.right_light.up, armor.right_light.down,
                                        armor.left_light.down};

  cv::Vec3d rvec, tvec;
  cv::solvePnP(object_points, image_points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

  // 相机坐标系
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);

  // 云台坐标系
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

  // 世界坐标系
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  // 姿态
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;

  armor.ypr_in_gimbal = tools::Eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::Eulers(R_armor2world, 2, 1, 0);
  armor.ypd_in_world = tools::Xyz2Ypd(armor.xyz_in_world);

  // 平衡步兵（大装甲板+三/四/五号）不做yaw优化
  bool is_balance =
      (armor.kind == ArmorKind::kBig) &&
      (armor.name == ArmorName::kThree || armor.name == ArmorName::kFour || armor.name == ArmorName::kFive);
  if (!is_balance) {
    OptimizeYaw(armor);
  }
}

void ArmorSolver::OptimizeYaw(Armor& armor) const {
  Eigen::Vector3d gimbal_ypr = tools::Eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double kSearchRange = 140.0;  // degrees
  double yaw0 = tools::LimitRad(gimbal_ypr[0] - kSearchRange / 2 * CV_PI / 180.0);

  double min_error = 1e10;
  double best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < static_cast<int>(kSearchRange); i++) {
    double yaw = tools::LimitRad(yaw0 + i * CV_PI / 180.0);
    double error = ArmorReprojectionError(armor, yaw);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double ArmorSolver::ArmorReprojectionError(const Armor& armor, double yaw) const {
  auto image_points = ReprojectArmor(armor.xyz_in_world, yaw, armor.kind, armor.name);
  double error = 0.0;
  std::vector<cv::Point2f> actual_points{armor.left_light.up, armor.right_light.up, armor.right_light.down,
                                         armor.left_light.down};
  for (int i = 0; i < 4; i++) {
    error += cv::norm(actual_points[i] - image_points[i]);
  }
  return error;
}

std::vector<cv::Point2f> ArmorSolver::ReprojectArmor(const Eigen::Vector3d& xyz_in_world, double yaw, ArmorKind kind,
                                                     ArmorName name) const {
  double sin_yaw = std::sin(yaw);
  double cos_yaw = std::cos(yaw);

  // 前哨站pitch朝下，其余朝上
  double pitch = (name == ArmorName::kOutpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  double sin_pitch = std::sin(pitch);
  double cos_pitch = std::cos(pitch);

  // clang-format off
  Eigen::Matrix3d R_armor2world{
      {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
      {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
      {         -sin_pitch,        0,           cos_pitch}};
  // clang-format on

  const Eigen::Vector3d& t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
      R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  std::vector<cv::Point2f> image_points;
  const auto& object_points = GetObjectPoints(kind);
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

}  // namespace detection
}  // namespace ia
