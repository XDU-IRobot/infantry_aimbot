
#pragma once

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "number_classifier.hpp"
#include "pnp_solver.hpp"
#include "traditional_detector.hpp"
#include "params.hpp"

namespace ia {
namespace detection {

class DetectionPipeline {
  constexpr static float kBigArmorWidth = 0.225;
  constexpr static float kBigArmorHeight = 0.05;
  constexpr static float kSmallArmorWidth = 0.133;
  constexpr static float kSmallArmorHeight = 0.05;

 public:
  explicit DetectionPipeline(const Params &params)
      : detector_{{params.bin_threshold, params.enemy_color},
                  {params.angle_to_vertigal_max, params.height_width_min_ratio, params.size_area_min_ratio,
                   params.is_corner_correct},
                  {params.lights_angle_max_diff, params.lights_length_max_ratio, params.lights_y_max_ratio,
                   params.width_height_min_ratio, params.width_height_max_ratio, params.max_angle,
                   params.inside_thresh}},
        big_armor_pnp_solver_{
            {
                cv::Point3f{-kBigArmorWidth / 2, -kBigArmorHeight / 2, 0.f},  ///< 左上
                cv::Point3f{kBigArmorWidth / 2, -kBigArmorHeight / 2, 0.f},   ///< 右上
                cv::Point3f{kBigArmorWidth / 2, kBigArmorHeight / 2, 0.f},    ///< 右下
                cv::Point3f{-kBigArmorWidth / 2, kBigArmorHeight / 2, 0.f}    ///< 左下
            },
            (cv::Mat_<double>(3, 3) << params.camera_matrix[0], params.camera_matrix[1], params.camera_matrix[2],
             params.camera_matrix[3], params.camera_matrix[4], params.camera_matrix[5], params.camera_matrix[6],
             params.camera_matrix[7], params.camera_matrix[8]),
            (cv::Mat_<double>(1, 5) << params.distortion_coefficients[0], params.distortion_coefficients[1],
             params.distortion_coefficients[2], params.distortion_coefficients[3], params.distortion_coefficients[4])},
        small_armor_pnp_solver_{
            {
                cv::Point3f{-kSmallArmorWidth / 2, -kSmallArmorHeight / 2, 0.f},  ///< 左上
                cv::Point3f{kSmallArmorWidth / 2, -kSmallArmorHeight / 2, 0.f},   ///< 右上
                cv::Point3f{kSmallArmorWidth / 2, kSmallArmorHeight / 2, 0.f},    ///< 右下
                cv::Point3f{-kSmallArmorWidth / 2, kSmallArmorHeight / 2, 0.f}    ///< 左下
            },
            (cv::Mat_<double>(3, 3) << params.camera_matrix[0], params.camera_matrix[1], params.camera_matrix[2],
             params.camera_matrix[3], params.camera_matrix[4], params.camera_matrix[5], params.camera_matrix[6],
             params.camera_matrix[7], params.camera_matrix[8]),
            (cv::Mat_<double>(1, 5) << params.distortion_coefficients[0], params.distortion_coefficients[1],
             params.distortion_coefficients[2], params.distortion_coefficients[3], params.distortion_coefficients[4])},
        number_classifier_(std::filesystem::path(ament_index_cpp::get_package_share_directory("infantry_aimbot")) /
                           std::filesystem::path(params.number_classify_model_path)),
        params_{params} {}
  ~DetectionPipeline() = default;

  result_sp<std::vector<Armor>> ProcessImage(const cv::Mat &image) {
    const auto detected_armors = detector_.DetectArmors(image);
    if (!detected_armors) {
      return std::make_error_code(std::errc::no_message);
    }
    for (auto armor : *detected_armors.value()) {
      // solve pose for each detected armor
      const std::vector<cv::Point2f> image_points{armor.left_light.up, armor.right_light.up, armor.right_light.down,
                                                  armor.left_light.down};
      const auto solved_pose = (armor.type == Armor::SMALL) ? small_armor_pnp_solver_.SolvePose(image_points)
                                                            : big_armor_pnp_solver_.SolvePose(image_points);
      if (solved_pose) {
        armor.pose = *solved_pose.value();
      }
    }
    for (auto &armor : *detected_armors.value()) {
      const auto recognized_number = number_classifier_.Classify(
          image, {armor.left_light.down, armor.left_light.up, armor.right_light.up, armor.right_light.down});
      if (recognized_number) {
        armor.num_id = recognized_number.value()->number;
        armor.confidence = recognized_number.value()->confidence;
      }
    }
    return detected_armors;
  }

 private:
  TraditionalDetector detector_;
  PnpSolver big_armor_pnp_solver_, small_armor_pnp_solver_;
  NumberClassifier number_classifier_;
  Params params_;
};

}  // namespace detection
}  // namespace ia