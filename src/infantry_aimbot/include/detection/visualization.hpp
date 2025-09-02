
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "typedefs.hpp"

namespace ia {
namespace detection {
inline void Print(const geometry_msgs::msg::Pose &pose) {
  std::printf("pos: %f %f %f | orit: %f %f %f %f\n", pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline auto DrawArmorToRviz(const Armor &armor) -> sp<visualization_msgs::msg::MarkerArray> {
  auto ret = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose = armor.pose;

  marker.scale.x = 0.012f;
  marker.scale.y = 0.1f;
  marker.scale.z = 0.05f;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  ret->markers.push_back(marker);
  return ret;
}
inline auto DrawArmorToRviz(const std::vector<Armor> &armors) -> sp<visualization_msgs::msg::MarkerArray> {
  auto ret = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 0.012f;
  marker.scale.y = 0.1f;
  marker.scale.z = 0.05f;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  ret->markers.reserve(armors.size());
  for (const auto &armor : armors) {
    marker.pose = armor.pose;
    ret->markers.push_back(marker);
  }

  return ret;
}

/**
 * @brief 把装甲板画在画面上
 */
inline void DrawArmor(cv::Mat &image, const Armor &armor) {
  const std::string text1 = "id: " + std::to_string(armor.num_id);
  const std::string text2 = "conf: " + std::to_string(int(armor.confidence * 100));

  cv::line(image, armor.left_light.up, armor.left_light.down, cv::Scalar(255, 0, 255), 1);
  cv::line(image, armor.left_light.down, armor.right_light.down, cv::Scalar(255, 0, 255), 1);
  cv::line(image, armor.right_light.down, armor.right_light.up, cv::Scalar(255, 0, 255), 1);
  cv::line(image, armor.right_light.up, armor.left_light.up, cv::Scalar(255, 0, 255), 1);

  const float distance =
      sqrt(pow(armor.pose.position.x, 2) + pow(armor.pose.position.y, 2) + pow(armor.pose.position.z, 2));
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << distance;

  std::string text3 = "distance: " + oss.str() + "m";
  cv::putText(image, text1, armor.right_light.down, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(154, 250, 0), 2, 3);
  cv::putText(image, text2, cv::Point(armor.right_light.down.x, armor.right_light.down.y + 30),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(154, 250, 0), 2, 3);
  cv::putText(image, text3, cv::Point(armor.right_light.down.x, armor.right_light.down.y + 60),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(154, 250, 0), 2, 3);
}

/**
 * @brief 把装甲板画在画面上
 */
inline void DrawArmor(cv::Mat &image, const std::vector<Armor> &armors) {
  for (const auto &armor : armors) {
    DrawArmor(image, armor);
  }
}
}  // namespace detection
}  // namespace ia