
#include "detection/pnp_solver.hpp"
// std
// ros
// third-party
#include <opencv2/core/eigen.hpp>

// project

namespace ia {
namespace detection {

PnpSolver::PnpSolver(std::vector<cv::Point3f> object_points,  ///< 左上，右上，右下，左下
                     cv::Mat camera_matrix, cv::Mat dist_coeffs)
    : object_points_{object_points}, camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {}

result_sp<geometry_msgs::msg::Pose> PnpSolver::SolvePose(const std::vector<cv::Point2f> &image_points) {
  assert(image_points.size() == 4);
  if (!cv::solvePnP(object_points_, image_points, camera_matrix_, dist_coeffs_, intermidiate_buffer_.rvec,
                    intermidiate_buffer_.tvec, false, cv::SOLVEPNP_IPPE)) {
    return std::make_error_code(std::errc::no_message);
  }
  cv::Rodrigues(intermidiate_buffer_.rvec, intermidiate_buffer_.rvec);
  cv::cv2eigen(intermidiate_buffer_.rvec, intermidiate_buffer_.rvec_ros);
  cv::cv2eigen(intermidiate_buffer_.tvec, intermidiate_buffer_.tvec_ros);
  intermidiate_buffer_.tvec_ros = intermidiate_buffer_.cv_to_ros * intermidiate_buffer_.tvec_ros;
  intermidiate_buffer_.rvec_ros = intermidiate_buffer_.cv_to_ros * intermidiate_buffer_.rvec_ros;
  return ret_buffer_;
}

}  // namespace detection
}  // namespace ia