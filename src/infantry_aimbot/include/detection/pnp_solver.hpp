
#pragma once

// std
// ros
#include <geometry_msgs/msg/pose.hpp>

// third-party
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// project
#include "typedefs.hpp"

namespace ia {
namespace detection {

class PnpSolver {
 public:
  PnpSolver() = delete;
  PnpSolver(std::vector<cv::Point3f> object_points,  ///< 左上，右上，右下，左下
            cv::Mat camera_matrix, cv::Mat dist_coeffs);

  result_sp<geometry_msgs::msg::Pose> SolvePose(const std::vector<cv::Point2f> &image_points);

 private:
  const std::vector<cv::Point3f> object_points_;
  const cv::Mat camera_matrix_;  ///< 3x3 相机内参矩阵
  const cv::Mat dist_coeffs_;    ///< 1x5 畸变系数矩阵

  struct _ {
    _() { cv_to_ros << 0, 0, 1, -1, 0, 0, 0, -1, 0; }
    Eigen::Matrix3d cv_to_ros;
    cv::Mat rvec;  ///< 旋转向量
    cv::Mat tvec;  ///< 平移向量
    Eigen::Matrix3d rvec_ros;
    Eigen::Vector3d tvec_ros;
  } intermidiate_buffer_;

  sp<geometry_msgs::msg::Pose> ret_buffer_{std::make_shared<geometry_msgs::msg::Pose>()};
};

}  // namespace detection
}  // namespace ia